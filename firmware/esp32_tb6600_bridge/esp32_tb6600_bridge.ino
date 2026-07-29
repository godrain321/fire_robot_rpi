#include <AccelStepper.h>

/*
  ESP32 DevKit V4 (ESP32-WROOM-32E) - Dual TB6600 Serial Motor Bridge

  No physical wheel encoders are required. AccelStepper::currentPosition()
  counts the STEP pulses that this firmware actually generated and reports
  them as temporary signed encoder counts.

  Pi -> ESP32:
    M,<seq>,<left_sps>,<right_sps>
    STOP,<seq>
    PING,<seq>
    ZERO,<seq>
    K,<seq>,<W|X|A|D|S>

  ESP32 -> Pi:
    ACK,<seq>
    STAT,<millis>,<state>,<left_sps>,<right_sps>
    ENC,<millis>,<left_count>,<right_count>
    ERR,<message>
*/

// Pin mapping supplied for the current robot wiring.
#define L_STEP 25
#define L_DIR 26
#define L_EN 27

#define R_STEP 14
#define R_DIR 12
#define R_EN 13

// GPIO12 is an ESP32 strapping pin. If flashing or booting becomes unreliable,
// move R_DIR to GPIO32 or GPIO33 and update this definition.

const bool ENABLE_ACTIVE_LOW = true;
const bool INVERT_LEFT_DIR = false;
const bool INVERT_RIGHT_DIR = false;

const float MAX_STEP_SPEED = 1200.0F;
// TB6600 is set to 1/2 microstep. 75 step/s gives the same shaft speed that
// 300 step/s produced at the previously assumed 1/8 setting.
const float DEFAULT_KEY_SPS = 75.0F;
const unsigned long COMMAND_TIMEOUT_MS = 500;
const unsigned long STAT_PERIOD_MS = 100;
const unsigned long ENC_PERIOD_MS = 100;
const unsigned long BAUDRATE = 115200;
const size_t MAX_INPUT_LINE = 120;

AccelStepper leftMotor(AccelStepper::DRIVER, L_STEP, L_DIR);
AccelStepper rightMotor(AccelStepper::DRIVER, R_STEP, R_DIR);

String inputLine;
float currentLeftSps = 0.0F;
float currentRightSps = 0.0F;
unsigned long lastCommandMs = 0;
unsigned long lastStatMs = 0;
unsigned long lastEncMs = 0;
String driveState = "BOOT";

void setEnable(bool enable) {
  if (ENABLE_ACTIVE_LOW) {
    digitalWrite(L_EN, enable ? LOW : HIGH);
    digitalWrite(R_EN, enable ? LOW : HIGH);
  } else {
    digitalWrite(L_EN, enable ? HIGH : LOW);
    digitalWrite(R_EN, enable ? HIGH : LOW);
  }
}

float clampSps(float value) {
  if (value > MAX_STEP_SPEED) return MAX_STEP_SPEED;
  if (value < -MAX_STEP_SPEED) return -MAX_STEP_SPEED;
  return value;
}

void applyMotorSpeeds(float leftSps, float rightSps) {
  leftSps = clampSps(leftSps);
  rightSps = clampSps(rightSps);
  currentLeftSps = leftSps;
  currentRightSps = rightSps;

  leftMotor.setSpeed(INVERT_LEFT_DIR ? -leftSps : leftSps);
  rightMotor.setSpeed(INVERT_RIGHT_DIR ? -rightSps : rightSps);
  driveState = (leftSps == 0.0F && rightSps == 0.0F) ? "STOP" : "RUN";
}

void stopMotors() {
  applyMotorSpeeds(0.0F, 0.0F);
}

void sendAck(const String &seq) {
  Serial.print("ACK,");
  Serial.println(seq);
}

void sendErr(const String &message) {
  Serial.print("ERR,");
  Serial.println(message);
}

void sendStat() {
  Serial.print("STAT,");
  Serial.print(millis());
  Serial.print(',');
  Serial.print(driveState);
  Serial.print(',');
  Serial.print(currentLeftSps, 1);
  Serial.print(',');
  Serial.println(currentRightSps, 1);
}

long logicalLeftCount() {
  const long motorCount = leftMotor.currentPosition();
  return INVERT_LEFT_DIR ? -motorCount : motorCount;
}

long logicalRightCount() {
  const long motorCount = rightMotor.currentPosition();
  return INVERT_RIGHT_DIR ? -motorCount : motorCount;
}

void sendEnc() {
  Serial.print("ENC,");
  Serial.print(millis());
  Serial.print(',');
  Serial.print(logicalLeftCount());
  Serial.print(',');
  Serial.println(logicalRightCount());
}

void resetCounts() {
  // setCurrentPosition() also changes AccelStepper's internal speed to zero.
  // Restore the requested speeds so ZERO does not unexpectedly stop motion.
  leftMotor.setCurrentPosition(0);
  rightMotor.setCurrentPosition(0);
  leftMotor.setSpeed(INVERT_LEFT_DIR ? -currentLeftSps : currentLeftSps);
  rightMotor.setSpeed(INVERT_RIGHT_DIR ? -currentRightSps : currentRightSps);
}

int splitCsv(String line, String parts[], int maxParts) {
  int count = 0;
  int start = 0;
  line.trim();

  while (count < maxParts) {
    const int comma = line.indexOf(',', start);
    if (comma == -1) {
      parts[count++] = line.substring(start);
      break;
    }
    parts[count++] = line.substring(start, comma);
    start = comma + 1;
  }

  for (int i = 0; i < count; ++i) parts[i].trim();
  return count;
}

void handleKeyCommand(const String &seq, String key) {
  key.toUpperCase();

  if (key == "W") {
    applyMotorSpeeds(DEFAULT_KEY_SPS, DEFAULT_KEY_SPS);
  } else if (key == "X") {
    applyMotorSpeeds(-DEFAULT_KEY_SPS, -DEFAULT_KEY_SPS);
  } else if (key == "A") {
    applyMotorSpeeds(-DEFAULT_KEY_SPS, DEFAULT_KEY_SPS);
  } else if (key == "D") {
    applyMotorSpeeds(DEFAULT_KEY_SPS, -DEFAULT_KEY_SPS);
  } else if (key == "S") {
    stopMotors();
  } else {
    sendErr("UNKNOWN_KEY_" + key);
    return;
  }

  lastCommandMs = millis();
  sendAck(seq);
}

void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  String parts[6];
  const int partCount = splitCsv(line, parts, 6);
  if (partCount <= 0) return;

  String command = parts[0];
  command.toUpperCase();

  if (command == "M") {
    if (partCount != 4) {
      sendErr("BAD_M_FORMAT");
      return;
    }
    applyMotorSpeeds(parts[2].toFloat(), parts[3].toFloat());
    lastCommandMs = millis();
    sendAck(parts[1]);
    return;
  }

  if (command == "STOP") {
    const String seq = (partCount >= 2) ? parts[1] : "-1";
    stopMotors();
    lastCommandMs = millis();
    sendAck(seq);
    return;
  }

  if (command == "PING") {
    const String seq = (partCount >= 2) ? parts[1] : "-1";
    sendAck(seq);
    sendStat();
    sendEnc();
    return;
  }

  if (command == "ZERO") {
    const String seq = (partCount >= 2) ? parts[1] : "-1";
    resetCounts();
    sendAck(seq);
    return;
  }

  if (command == "K") {
    if (partCount != 3) {
      sendErr("BAD_K_FORMAT");
      return;
    }
    handleKeyCommand(parts[1], parts[2]);
    return;
  }

  sendErr("UNKNOWN_CMD_" + command);
}

void readSerialLines() {
  while (Serial.available() > 0) {
    const char character = static_cast<char>(Serial.read());

    // Accept CR, LF, and CRLF. Empty terminators are ignored. This prevents
    // picocom CR-only commands from accumulating in the receive buffer.
    if (character == '\r' || character == '\n') {
      if (inputLine.length() > 0) {
        handleLine(inputLine);
        inputLine = "";
      }
      continue;
    }

    if (character < 32 || character > 126) continue;
    if (inputLine.length() >= MAX_INPUT_LINE) {
      inputLine = "";
      sendErr("LINE_TOO_LONG");
      continue;
    }
    inputLine += character;
  }
}

void setup() {
  Serial.begin(BAUDRATE);
  inputLine.reserve(MAX_INPUT_LINE);

  pinMode(L_EN, OUTPUT);
  pinMode(R_EN, OUTPUT);
  setEnable(true);

  leftMotor.setMaxSpeed(MAX_STEP_SPEED);
  leftMotor.setMinPulseWidth(5);
  rightMotor.setMaxSpeed(MAX_STEP_SPEED);
  rightMotor.setMinPulseWidth(5);

  currentLeftSps = 0.0F;
  currentRightSps = 0.0F;
  resetCounts();
  stopMotors();

  lastCommandMs = millis();
  lastStatMs = millis();
  lastEncMs = millis();
  driveState = "STOP";

  Serial.println("STAT,0,READY,0,0");
  Serial.println("ENC,0,0,0");
}

void loop() {
  readSerialLines();
  const unsigned long now = millis();

  if ((now - lastCommandMs) > COMMAND_TIMEOUT_MS &&
      (currentLeftSps != 0.0F || currentRightSps != 0.0F)) {
    stopMotors();
    driveState = "FAILSAFE";
    Serial.println("ERR,COMMAND_TIMEOUT_STOP");
  }

  // These calls generate STEP pulses and update currentPosition().
  leftMotor.runSpeed();
  rightMotor.runSpeed();

  if ((now - lastStatMs) >= STAT_PERIOD_MS) {
    lastStatMs = now;
    sendStat();
  }
  if ((now - lastEncMs) >= ENC_PERIOD_MS) {
    lastEncMs = now;
    sendEnc();
  }
}
