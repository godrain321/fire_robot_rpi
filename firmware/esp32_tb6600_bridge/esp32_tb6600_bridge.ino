#include <AccelStepper.h>

/*
  ESP32 -> dual TB6600 bridge (1/8 microstep)

  The ESP32 is deliberately only a motor actuator. Map localization, path
  following, and waypoint completion belong to ROS where map->base_link can be
  observed. This prevents an ESP32 open-loop waypoint routine fighting ROS.

  Pi -> ESP32: M,<seq>,<left_sps>,<right_sps> | STOP,<seq> | PING,<seq> | ZERO,<seq>
  ESP32 -> Pi: ACK,<seq> | STAT,<ms>,<state>,<left_sps>,<right_sps>
                  | ENC,<ms>,<generated_left_steps>,<generated_right_steps>
*/

#define L_STEP 25
#define L_DIR 26
#define L_EN 27
#define R_STEP 14
#define R_DIR 12
#define R_EN 13

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

void sendTelemetry() {
  Serial.printf("STAT,%lu,%s,%.1f,%.1f\n", millis(), driveState.c_str(), currentLeftSps, currentRightSps);
  const long left = INVERT_LEFT_DIR ? -leftMotor.currentPosition() : leftMotor.currentPosition();
  const long right = INVERT_RIGHT_DIR ? -rightMotor.currentPosition() : rightMotor.currentPosition();
  Serial.printf("ENC,%lu,%ld,%ld\n", millis(), left, right);
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
  emergencyStop();
  lastCommandMs = lastTelemetryMs = millis();
  lastRampUs = micros();
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
  if (millis() - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = millis();
    sendTelemetry();
  }
}
