#include <AccelStepper.h>
#include <SPI.h>
#include <Wire.h>

/*
  ESP32 -> dual TB6600 bridge (1/8 microstep)

  The ESP32 is deliberately only a motor actuator. Map localization, path
  following, and waypoint completion belong to ROS where map->base_link can be
  observed. This prevents an ESP32 open-loop waypoint routine fighting ROS.

  Pi -> ESP32: M,<seq>,<left_sps>,<right_sps> | STOP,<seq> | PING,<seq> | ZERO,<seq>
  ESP32 -> Pi: ACK,<seq> | STAT,<ms>,<state>,<left_sps>,<right_sps>
                  | ENC,<ms>,<generated_left_steps>,<generated_right_steps>
                  | ENC_PHYS,<ms>,<left_count>,<right_count>,<left_raw>,<right_raw>
                  | IMU,<ms>,<gyro_z_rad_s>,<system_cal>,<gyro_cal>
                  | US,<ms>,<distance_cm>,<valid>
*/

#define L_STEP 25
#define L_DIR 26
#define L_EN 27
#define R_STEP 14
#define R_DIR 12
#define R_EN 13

// Two AS5048A wheel encoders share SPI and use separate chip-select pins.
#define ENC_SCK 18
#define ENC_MOSI 23
#define ENC_MISO 19
#define ENC_LEFT_CS 17
#define ENC_RIGHT_CS 16

// CJMCU-055: ATX=SDA, LRX=SCL, and the board I2C select pin is wired to GND.
#define IMU_SDA 21
#define IMU_SCL 22

// HC-SR04: ECHO must pass through a 5 V -> 3.3 V voltage divider.
#define US_TRIG 32
#define US_ECHO 33

const bool ENABLE_ACTIVE_LOW = true;
const bool INVERT_LEFT_DIR = false;
const bool INVERT_RIGHT_DIR = false;
const float MAX_STEP_SPEED = 1600.0F;
// Limit command edges that made skid steering harsh. This ramps STEP frequency,
// while runSpeed() remains responsible for accurately timed pulses.
const float MAX_STEP_ACCEL = 900.0F;  // steps/s^2; tune after wheels-off-ground test
const unsigned long COMMAND_TIMEOUT_MS = 500;
const unsigned long TELEMETRY_PERIOD_MS = 200;
const unsigned long BAUDRATE = 115200;
const size_t MAX_INPUT_LINE = 96;

// AS5048A: 14-bit absolute angle, 16384 counts/revolution.
const uint16_t AS5048A_ANGLE_REGISTER = 0x3FFF;
const uint16_t AS5048A_CLEAR_ERROR_REGISTER = 0x0001;
const uint16_t AS5048A_READ_FLAG = 0x4000;
const int32_t AS5048A_COUNTS_PER_REV = 16384;
const int32_t AS5048A_HALF_COUNTS = AS5048A_COUNTS_PER_REV / 2;
const unsigned long ENCODER_SAMPLE_PERIOD_US = 10000;  // 100 Hz
const uint32_t ENCODER_SPI_HZ = 1000000;               // reliable starting speed
const unsigned long IMU_SAMPLE_PERIOD_MS = 20;          // 50 Hz
const unsigned long US_PERIOD_MS = 100;
const unsigned long US_TIMEOUT_US = 30000;

// Keep raw wheel directions here. Direction correction is a Mode 11 launch
// parameter, so changing wheel installation does not require reflashing.
const int ENCODER_LEFT_SIGN = 1;
const int ENCODER_RIGHT_SIGN = 1;

// BNO055 page-0 registers and values used by this firmware.
const uint8_t BNO055_ADDRESS = 0x28;  // ADR/COM3 low; use 0x29 when held high.
const uint8_t BNO055_CHIP_ID = 0x00;
const uint8_t BNO055_GYRO_Z_LSB = 0x18;
const uint8_t BNO055_CALIB_STAT = 0x35;
const uint8_t BNO055_UNIT_SEL = 0x3B;
const uint8_t BNO055_OPR_MODE = 0x3D;
const uint8_t BNO055_PWR_MODE = 0x3E;
const uint8_t BNO055_SYS_TRIGGER = 0x3F;
const uint8_t BNO055_ID_VALUE = 0xA0;
const uint8_t BNO055_MODE_CONFIG = 0x00;
const uint8_t BNO055_MODE_NDOF = 0x0C;

AccelStepper leftMotor(AccelStepper::DRIVER, L_STEP, L_DIR);
AccelStepper rightMotor(AccelStepper::DRIVER, R_STEP, R_DIR);
String inputLine;
float targetLeftSps = 0.0F;
float targetRightSps = 0.0F;
float currentLeftSps = 0.0F;
float currentRightSps = 0.0F;
unsigned long lastCommandMs = 0;
unsigned long lastRampUs = 0;
unsigned long lastTelemetryMs = 0;
String driveState = "BOOT";

SPISettings encoderSpiSettings(ENCODER_SPI_HZ, MSBFIRST, SPI_MODE1);
unsigned long lastEncoderSampleUs = 0;
struct WheelEncoderState {
  uint16_t rawAngle;
  uint16_t previousRaw;
  int64_t cumulativeCounts;
  uint32_t errors;
  bool ready;
};

WheelEncoderState leftEncoder = {0, 0, 0, 0, false};
WheelEncoderState rightEncoder = {0, 0, 0, 0, false};
bool imuReady = false;
float gyroZRadPerSec = 0.0F;
uint8_t imuSystemCalibration = 0;
uint8_t imuGyroCalibration = 0;
uint32_t imuErrors = 0;
unsigned long lastImuSampleMs = 0;
volatile unsigned long echoRiseUs = 0;
volatile unsigned long echoPulseUs = 0;
volatile bool echoComplete = false;
bool ultrasonicWaiting = false;
unsigned long lastUltrasonicTriggerMs = 0;
unsigned long ultrasonicTriggerUs = 0;

void IRAM_ATTR onUltrasonicEcho() {
  if (digitalRead(US_ECHO) == HIGH) {
    echoRiseUs = micros();
  } else if (echoRiseUs != 0) {
    echoPulseUs = micros() - echoRiseUs;
    echoComplete = true;
    echoRiseUs = 0;
  }
}

void updateUltrasonic() {
  const unsigned long nowMs = millis();
  if (ultrasonicWaiting) {
    unsigned long pulseUs = 0;
    bool complete = false;
    noInterrupts();
    if (echoComplete) {
      pulseUs = echoPulseUs;
      echoComplete = false;
      complete = true;
    }
    interrupts();
    if (complete || micros() - ultrasonicTriggerUs >= US_TIMEOUT_US) {
      const float distanceCm = pulseUs / 58.0F;
      const bool valid = complete && distanceCm >= 2.0F && distanceCm <= 400.0F;
      Serial.printf("US,%lu,%.2f,%d\n", nowMs, valid ? distanceCm : 0.0F,
                    valid ? 1 : 0);
      ultrasonicWaiting = false;
    }
  }
  if (!ultrasonicWaiting && nowMs - lastUltrasonicTriggerMs >= US_PERIOD_MS) {
    lastUltrasonicTriggerMs = nowMs;
    noInterrupts();
    echoRiseUs = 0;
    echoComplete = false;
    interrupts();
    digitalWrite(US_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(US_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(US_TRIG, LOW);
    ultrasonicTriggerUs = micros();
    ultrasonicWaiting = true;
  }
}

void setEnable(bool enabled) {
  const int active = ENABLE_ACTIVE_LOW ? LOW : HIGH;
  const int inactive = ENABLE_ACTIVE_LOW ? HIGH : LOW;
  digitalWrite(L_EN, enabled ? active : inactive);
  digitalWrite(R_EN, enabled ? active : inactive);
}

float clampSps(float value) {
  return constrain(value, -MAX_STEP_SPEED, MAX_STEP_SPEED);
}

void setTargets(float left, float right) {
  targetLeftSps = clampSps(left);
  targetRightSps = clampSps(right);
  driveState = (targetLeftSps == 0.0F && targetRightSps == 0.0F) ? "STOP" : "RUN";
}

void emergencyStop() {
  targetLeftSps = targetRightSps = 0.0F;
  currentLeftSps = currentRightSps = 0.0F;
  leftMotor.setSpeed(0.0F);
  rightMotor.setSpeed(0.0F);
  driveState = "STOP";
}

float approach(float current, float target, float maximumDelta) {
  if (current < target) return min(current + maximumDelta, target);
  if (current > target) return max(current - maximumDelta, target);
  return current;
}

void updateSpeedRamp() {
  const unsigned long nowUs = micros();
  const unsigned long elapsedUs = nowUs - lastRampUs;
  if (elapsedUs < 1000) return;
  lastRampUs = nowUs;
  const float maxDelta = MAX_STEP_ACCEL * elapsedUs * 1.0e-6F;
  currentLeftSps = approach(currentLeftSps, targetLeftSps, maxDelta);
  currentRightSps = approach(currentRightSps, targetRightSps, maxDelta);
  leftMotor.setSpeed(INVERT_LEFT_DIR ? -currentLeftSps : currentLeftSps);
  rightMotor.setSpeed(INVERT_RIGHT_DIR ? -currentRightSps : currentRightSps);
}

int splitCsv(String line, String fields[], int capacity) {
  int count = 0;
  int start = 0;
  line.trim();
  while (count < capacity) {
    const int comma = line.indexOf(',', start);
    if (comma < 0) { fields[count++] = line.substring(start); break; }
    fields[count++] = line.substring(start, comma);
    start = comma + 1;
  }
  for (int i = 0; i < count; ++i) fields[i].trim();
  return count;
}

void ack(const String &seq) { Serial.println("ACK," + seq); }

bool hasEvenParity(uint16_t value) {
  bool parity = false;
  while (value != 0) {
    parity = !parity;
    value &= (value - 1);
  }
  return !parity;
}

uint16_t addEvenParity(uint16_t value) {
  value &= 0x7FFF;
  if (!hasEvenParity(value)) value |= 0x8000;
  return value;
}

uint16_t transferAs5048aFrame(uint8_t chipSelectPin, uint16_t transmitData) {
  digitalWrite(chipSelectPin, LOW);
  delayMicroseconds(1);

  const uint8_t highByte =
      SPI.transfer(static_cast<uint8_t>(transmitData >> 8));
  const uint8_t lowByte =
      SPI.transfer(static_cast<uint8_t>(transmitData & 0xFF));

  delayMicroseconds(1);
  digitalWrite(chipSelectPin, HIGH);
  delayMicroseconds(1);

  return (static_cast<uint16_t>(highByte) << 8) | lowByte;
}

void clearAs5048aError(uint8_t chipSelectPin) {
  const uint16_t clearCommand = addEvenParity(
      AS5048A_READ_FLAG | AS5048A_CLEAR_ERROR_REGISTER
  );

  SPI.beginTransaction(encoderSpiSettings);
  transferAs5048aFrame(chipSelectPin, clearCommand);
  transferAs5048aFrame(chipSelectPin, 0x0000U);
  SPI.endTransaction();
}

bool readAs5048aAngle(uint8_t chipSelectPin, uint16_t &rawAngle) {
  const uint16_t readCommand = addEvenParity(
      AS5048A_READ_FLAG | AS5048A_ANGLE_REGISTER
  );

  SPI.beginTransaction(encoderSpiSettings);
  transferAs5048aFrame(chipSelectPin, readCommand);
  const uint16_t response =
      transferAs5048aFrame(chipSelectPin, 0x0000U);
  SPI.endTransaction();

  const bool parityOk = hasEvenParity(response);
  const bool errorFlag = (response & 0x4000U) != 0U;

  if (!parityOk || errorFlag) {
    clearAs5048aError(chipSelectPin);
    return false;
  }

  rawAngle = response & 0x3FFFU;
  return true;
}

int32_t unwrapDelta(uint16_t currentRaw, uint16_t previousRaw) {
  int32_t delta = static_cast<int32_t>(currentRaw) - static_cast<int32_t>(previousRaw);
  if (delta > AS5048A_HALF_COUNTS) delta -= AS5048A_COUNTS_PER_REV;
  else if (delta < -AS5048A_HALF_COUNTS) delta += AS5048A_COUNTS_PER_REV;
  return delta;
}

bool writeBno055(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(BNO055_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readBno055(uint8_t reg, uint8_t *data, size_t length) {
  Wire.beginTransmission(BNO055_ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(BNO055_ADDRESS, static_cast<uint8_t>(length)) != length) return false;
  for (size_t i = 0; i < length; ++i) data[i] = Wire.read();
  return true;
}

bool initializeBno055() {
  Wire.begin(IMU_SDA, IMU_SCL, 400000U);
  delay(700);  // BNO055 power-on time.
  uint8_t chipId = 0;
  if (!readBno055(BNO055_CHIP_ID, &chipId, 1) || chipId != BNO055_ID_VALUE) {
    return false;
  }
  if (!writeBno055(BNO055_OPR_MODE, BNO055_MODE_CONFIG)) return false;
  delay(25);
  if (!writeBno055(BNO055_SYS_TRIGGER, 0x00)) return false;
  if (!writeBno055(BNO055_PWR_MODE, 0x00)) return false;
  if (!writeBno055(BNO055_UNIT_SEL, 0x00)) return false;  // deg/s gyro units.
  delay(10);
  if (!writeBno055(BNO055_OPR_MODE, BNO055_MODE_NDOF)) return false;
  delay(25);
  return true;
}

void updateImu() {
  const unsigned long nowMs = millis();
  if (nowMs - lastImuSampleMs < IMU_SAMPLE_PERIOD_MS) return;
  lastImuSampleMs = nowMs;
  if (!imuReady) {
    ++imuErrors;
    return;
  }
  uint8_t data[2] = {0, 0};
  uint8_t calibration = 0;
  if (!readBno055(BNO055_GYRO_Z_LSB, data, 2) ||
      !readBno055(BNO055_CALIB_STAT, &calibration, 1)) {
    ++imuErrors;
    return;
  }
  const int16_t rawGyroZ = static_cast<int16_t>(
      static_cast<uint16_t>(data[0]) |
      (static_cast<uint16_t>(data[1]) << 8));
  const float gyroZDps = static_cast<float>(rawGyroZ) / 16.0F;
  gyroZRadPerSec = gyroZDps * PI / 180.0F;
  imuSystemCalibration = (calibration >> 6) & 0x03;
  imuGyroCalibration = (calibration >> 4) & 0x03;
  Serial.printf("IMU,%lu,%.7f,%u,%u\n", nowMs, gyroZRadPerSec,
                imuSystemCalibration, imuGyroCalibration);
}

void updateWheelEncoder(uint8_t chipSelectPin, int directionSign,
                        WheelEncoderState &state) {
  uint16_t currentRaw = 0;
  if (!readAs5048aAngle(chipSelectPin, currentRaw)) {
    ++state.errors;
    return;
  }
  state.rawAngle = currentRaw;
  if (!state.ready) {
    state.previousRaw = currentRaw;
    state.ready = true;
    return;
  }
  const int32_t delta = unwrapDelta(currentRaw, state.previousRaw);
  state.cumulativeCounts += static_cast<int64_t>(delta) * directionSign;
  state.previousRaw = currentRaw;
}

void updateEncoders() {
  const unsigned long nowUs = micros();
  if (nowUs - lastEncoderSampleUs < ENCODER_SAMPLE_PERIOD_US) return;
  lastEncoderSampleUs = nowUs;

  updateWheelEncoder(ENC_LEFT_CS, ENCODER_LEFT_SIGN, leftEncoder);
  updateWheelEncoder(ENC_RIGHT_CS, ENCODER_RIGHT_SIGN, rightEncoder);
}

void zeroEncoderDistance() {
  // ZERO 명령은 누적 이동거리만 0으로 초기화한다.
  // 절대 각도(rawAngle)는 초기화하지 않는다.
  leftEncoder.cumulativeCounts = 0;
  rightEncoder.cumulativeCounts = 0;
  if (leftEncoder.ready) leftEncoder.previousRaw = leftEncoder.rawAngle;
  if (rightEncoder.ready) rightEncoder.previousRaw = rightEncoder.rawAngle;
}

void sendTelemetry() {
  Serial.printf(
    "STAT,%lu,%s,%.1f,%.1f\n",
    millis(),
    driveState.c_str(),
    currentLeftSps,
    currentRightSps
  );

  // 기존 모터 스텝 카운트 텔레메트리는 그대로 유지한다.
  const long left =
      INVERT_LEFT_DIR ? -leftMotor.currentPosition() : leftMotor.currentPosition();
  const long right =
      INVERT_RIGHT_DIR ? -rightMotor.currentPosition() : rightMotor.currentPosition();
  Serial.printf("ENC,%lu,%ld,%ld\n", millis(), left, right);

  if (leftEncoder.ready && rightEncoder.ready) {
    Serial.printf("ENC_PHYS,%lu,%lld,%lld,%u,%u\n", millis(),
                  static_cast<long long>(leftEncoder.cumulativeCounts),
                  static_cast<long long>(rightEncoder.cumulativeCounts),
                  leftEncoder.rawAngle, rightEncoder.rawAngle);
  } else {
    Serial.printf("ERR,ENCODER_NOT_READY,%lu,%lu\n",
                  static_cast<unsigned long>(leftEncoder.errors),
                  static_cast<unsigned long>(rightEncoder.errors));
  }
  if (!imuReady) {
    Serial.printf("ERR,IMU_NOT_READY,%lu\n",
                  static_cast<unsigned long>(imuErrors));
  }
}

void handleLine(String line) {
  String fields[4];
  const int count = splitCsv(line, fields, 4);
  fields[0].toUpperCase();
  if (fields[0] == "M" && count == 4) {
    setTargets(fields[2].toFloat(), fields[3].toFloat());
    lastCommandMs = millis();
    ack(fields[1]);
  } else if (fields[0] == "STOP" && count >= 2) {
    emergencyStop();
    lastCommandMs = millis();
    ack(fields[1]);
  } else if (fields[0] == "PING" && count >= 2) {
    ack(fields[1]);
    sendTelemetry();
  } else if (fields[0] == "ZERO" && count >= 2) {
    leftMotor.setCurrentPosition(0);
    rightMotor.setCurrentPosition(0);
    zeroEncoderDistance();
    ack(fields[1]);
  } else {
    Serial.println("ERR,BAD_COMMAND");
  }
}

void readSerial() {
  while (Serial.available()) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r' || c == '\n') {
      if (inputLine.length()) { handleLine(inputLine); inputLine = ""; }
    } else if (c >= 32 && c <= 126) {
      if (inputLine.length() < MAX_INPUT_LINE) inputLine += c;
      else { inputLine = ""; Serial.println("ERR,LINE_TOO_LONG"); }
    }
  }
}

void setup() {
  Serial.begin(BAUDRATE);
  inputLine.reserve(MAX_INPUT_LINE);
  pinMode(L_EN, OUTPUT); pinMode(R_EN, OUTPUT); setEnable(true);
  leftMotor.setMaxSpeed(MAX_STEP_SPEED); rightMotor.setMaxSpeed(MAX_STEP_SPEED);
  leftMotor.setMinPulseWidth(5); rightMotor.setMinPulseWidth(5);

  pinMode(ENC_LEFT_CS, OUTPUT);
  pinMode(ENC_RIGHT_CS, OUTPUT);
  digitalWrite(ENC_LEFT_CS, HIGH);
  digitalWrite(ENC_RIGHT_CS, HIGH);
  SPI.begin(ENC_SCK, ENC_MISO, ENC_MOSI, -1);
  pinMode(US_TRIG, OUTPUT);
  digitalWrite(US_TRIG, LOW);
  pinMode(US_ECHO, INPUT);
  attachInterrupt(digitalPinToInterrupt(US_ECHO), onUltrasonicEcho, CHANGE);
  imuReady = initializeBno055();
  delay(20);

  emergencyStop();
  lastCommandMs = lastTelemetryMs = millis();
  lastRampUs = micros();
  lastEncoderSampleUs = micros() - ENCODER_SAMPLE_PERIOD_US;
  lastUltrasonicTriggerMs = millis() - US_PERIOD_MS;
  lastImuSampleMs = millis() - IMU_SAMPLE_PERIOD_MS;

  // Prime the encoder before the first telemetry packet.
  for (int i = 0; i < 3; ++i) {
    updateEncoders();
    delay(10);
  }

  Serial.println("STAT,0,READY,0,0");
}

void loop() {
  readSerial();
  if (millis() - lastCommandMs > COMMAND_TIMEOUT_MS &&
      (targetLeftSps != 0.0F || targetRightSps != 0.0F)) {
    emergencyStop();
    driveState = "FAILSAFE";
    Serial.println("ERR,COMMAND_TIMEOUT_STOP");
  }
  updateSpeedRamp();
  leftMotor.runSpeed();
  rightMotor.runSpeed();
  updateEncoders();
  updateImu();
  updateUltrasonic();
  if (millis() - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = millis();
    sendTelemetry();
  }
}
