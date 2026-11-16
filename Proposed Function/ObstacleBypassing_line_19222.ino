#include <Arduino.h>
#include <util/atomic.h>
#include <Wire.h>

namespace RobotGyro {
enum Status { NOT_FOUND, STARTING, OK, ERROR, UNSUPPORTED };
Status status = NOT_FOUND;
uint8_t address = 0;
float dps[3] = {0, 0, 0};
unsigned long lastProbe = 0, lastRead = 0, started = 0;

bool readBytes(uint8_t device, uint8_t reg, uint8_t *data, uint8_t count) {
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(device);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0 || Wire.getWireTimeoutFlag()) return false;
  if (Wire.requestFrom(device, count, (uint8_t)true) != count || Wire.getWireTimeoutFlag()) return false;
  for (uint8_t i = 0; i < count; i++) {
    if (!Wire.available()) return false;
    data[i] = Wire.read();
  }
  return true;
}

bool writeRegister(uint8_t device, uint8_t reg, uint8_t value) {
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(device);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission(true) == 0 && !Wire.getWireTimeoutFlag();
}

void begin() {
  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(3000, true); // A stuck I2C device must not hang motor stopping.
  address = 0;
  status = NOT_FOUND;
  lastProbe = millis() - 2000UL;
}

void update() {
  unsigned long now = millis();
  if (!address) {
    if (now - lastProbe < 2000) return;
    lastProbe = now;
    bool unknownDevice = false;
    for (uint8_t candidate = 0x68; candidate <= 0x69; candidate++) {
      uint8_t identity;
      if (!readBytes(candidate, 0x75, &identity, 1)) continue;
      if (identity != 0x68) { unknownDevice = true; continue; }
      // Wake, enable axes, choose ±250 degrees/s, 42 Hz filtering and 50 Hz sampling.
      if (!writeRegister(candidate, 0x6B, 0x01) ||
          !writeRegister(candidate, 0x6C, 0x00) ||
          !writeRegister(candidate, 0x1B, 0x00) ||
          !writeRegister(candidate, 0x1A, 0x03) ||
          !writeRegister(candidate, 0x19, 19)) {
        status = ERROR;
        return;
      }
      address = candidate;
      status = STARTING;
      started = lastRead = now;
      return;
    }
    status = unknownDevice ? UNSUPPORTED : NOT_FOUND;
    return;
  }
  if (now - started < 100 || now - lastRead < 50) return;
  lastRead = now;
  uint8_t data[6];
  if (!readBytes(address, 0x43, data, sizeof(data))) {
    address = 0;
    status = ERROR;
    lastProbe = now;
    return;
  }
  for (uint8_t i = 0; i < 3; i++) {
    uint16_t word = ((uint16_t)data[i * 2] << 8) | data[i * 2 + 1];
    int32_t signedValue = word >= 32768 ? (int32_t)word - 65536L : word;
    dps[i] = signedValue / 131.0f;
  }
  status = OK;
}

} // namespace RobotGyro

// Logical positive speed drives forward. This incorporates DAIMA's
// FORWARD_SIGN and reproduces the attached sketch's left pivot.
struct Wheel {
  uint8_t pwm, dir1, dir2, encoder;
  bool invert;
};
const Wheel wheels[] = {
  {12, 34, 35, 18, true},   // Front left
  {8, 37, 36, 19, false},  // Front right
  {9, 43, 42, 3, true},    // Rear left
  {5, A4, A5, 2, false}    // Rear right
};
volatile uint32_t ticks[4] = {};
void tickFL() { ++ticks[0]; }
void tickFR() { ++ticks[1]; }
void tickRL() { ++ticks[2]; }
void tickRR() { ++ticks[3]; }
void (*const encoderHandlers[])() = {tickFL, tickFR, tickRL, tickRR};

const bool USE_FRONT_RIGHT = true;
const float CALIBRATION_CM = 100.0f;
const uint32_t MIN_CALIBRATION_TICKS = 16;
const int CRUISE_PWM = 60, APPROACH_PWM = 50, FOLLOW_PWM = 50, TURN_PWM = 50;
// Straight-run timing model. Assumes the measurements were made at PWM 50.
const float TIMING_REFERENCE_PWM = 50, TIMING_CM_PER_SECOND = 28;
const float TIMING_COAST_CM = 6;
const uint32_t TIMING_COAST_MS = 429;  // 6 cm with linear deceleration from 28 cm/s.
const float APPROACH_SLOW_CM = 110, APPROACH_STOP_CM = 40;
const uint32_t APPROACH_HOLD_MS = 500;
const float FRONT_STOP_CM = 40, FRONT_RESUME_CM = 70;
const float EDGE_FRONT_STOP_CM = 28;
const float RIGHT_SEEN_CM = 80, RIGHT_MIN_CM = 10;
const float CORNER_ADVANCE_CM = 7, FACE_ACQUIRE_LIMIT_CM = 30;
const float REJOIN_TOLERANCE_CM = 10;
const uint32_t BYPASS_LIMIT_MS = 45000;
const uint32_t PING_GAP_MS = 35, ECHO_TIMEOUT_US = 25000;
const int8_t GYRO_SIGN = 1;  // Left turns decrease yaw, as in the attachment.

enum Sensor : uint8_t { FRONT_LEFT, RIGHT, FRONT_RIGHT, SENSOR_COUNT };
const uint8_t triggerPins[] = {A3, A10, 32};  // Front right TRIG is D32.
const uint8_t echoPins[] = {A6, A9, A8};
struct Range {
  float cm;                  // Negative means no echo in the listening window.
  uint32_t measuredAt;
  uint8_t consecutiveClear;  // For the post-turn forward check.
};
Range ranges[SENSOR_COUNT] = {};
uint8_t sonarIndex = 0;
uint32_t lastPing = 0;
uint8_t rightHits = 0, rightMisses = 0;

enum State { STOPPED, CRUISE, TURN, FIRST_FACE, SECOND_FACE, CHECK_EXIT, RETURN_LINE };
bool sonarFresh(Sensor i);
void startTurn(float target, State next);

State state = STOPPED, afterTurn = STOPPED;
uint32_t bypassStarted = 0, stateStarted = 0, turnSettledAt = 0;
uint32_t lastApproachAt = 0;
bool bypassActive = false, turnSettling = false;
bool faceSeen = false, edgeCleared = false;
float x = 0, y = 0, pathCm = 0;
float detectedX = 0, detectedY = 0;
float faceStartPath = 0, edgeLostPath = 0;
float turnTarget = 0, cruiseHeading = 0;

uint32_t lastGyro = 0;
uint8_t biasSamples = 0;
float yaw = 0, yawRate = 0, bias = 0, previousPoseYaw = 0;
uint32_t previousTicks[4] = {}, calibrationStart[4] = {}, lastWheelTick[4] = {};
float cmPerTick[4] = {};
bool measuring = false, calibrated = false;
int driveLeft = 0, driveRight = 0;
uint32_t coastUntil = 0;
uint32_t lastPoseAt = 0, coastStartedAt = 0;
float coastDistance = 0;

void readTicks(uint32_t *counts) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    for (uint8_t i = 0; i < 4; ++i) counts[i] = ticks[i];
  }
}

void drive(int left, int right) {
  if (left == driveLeft && right == driveRight) return;
  uint32_t now = millis();
  for (uint8_t i = 0; i < 4; ++i) {
    int oldSpeed = i % 2 == 0 ? driveLeft : driveRight;
    int speed = i % 2 == 0 ? left : right;
    if (!oldSpeed && speed) lastWheelTick[i] = now;
    if (speed == oldSpeed) continue;
    const Wheel &wheel = wheels[i];
    int output = wheel.invert ? -speed : speed;
    // Remove PWM before switching direction.
    if (!oldSpeed || !speed || ((oldSpeed > 0) != (speed > 0))) {
      analogWrite(wheel.pwm, 0);
      digitalWrite(wheel.dir1, output < 0 ? HIGH : LOW);
      digitalWrite(wheel.dir2, output > 0 ? HIGH : LOW);
    }
    analogWrite(wheel.pwm, abs(output));
  }
  if (driveLeft > 0 && driveRight > 0 && left == 0 && right == 0) {
    coastUntil = now + 180;
    if (!calibrated) {
      coastStartedAt = now;
      coastDistance = TIMING_COAST_CM * (driveLeft + driveRight) /
                      (2.0f * TIMING_REFERENCE_PWM);
    }
  }
  driveLeft = left;
  driveRight = right;
}

void stop(const __FlashStringHelper *reason) {
  drive(0, 0);
  state = STOPPED;
  bypassActive = false;
  Serial.println(reason);
}

void updateYaw() {
  RobotGyro::update();
  if (RobotGyro::status != RobotGyro::OK) {
    biasSamples = 0;
    bias = yawRate = 0;
    return;
  }
  if (RobotGyro::lastRead == lastGyro) return;

  uint32_t elapsed = RobotGyro::lastRead - lastGyro;
  lastGyro = RobotGyro::lastRead;
  float raw = RobotGyro::dps[2];
  if (biasSamples < 40) {
    if (fabsf(raw) > 3) {
      biasSamples = 0;
      bias = 0;
    } else {
      ++biasSamples;
      bias += (raw - bias) / biasSamples;
      if (biasSamples == 40) Serial.println(F("GYRO_READY"));
    }
    return;
  }
  if (elapsed > 125 || fabsf(raw) > 245) {
    biasSamples = 0;
    bias = yawRate = 0;
    return;
  }
  float rate = GYRO_SIGN * (raw - bias);
  yaw += (yawRate + rate) * 0.0005f * elapsed;
  yawRate = rate;
}

bool gyroReady() {
  return biasSamples == 40 && RobotGyro::status == RobotGyro::OK &&
         millis() - lastGyro < 125;
}

void updateRanges() {
  if (millis() - lastPing < PING_GAP_MS) return;
  lastPing = millis();
  uint8_t i = sonarIndex;
  sonarIndex = (i + 1) % (USE_FRONT_RIGHT ? SENSOR_COUNT : 2);
  digitalWrite(triggerPins[i], HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPins[i], LOW);
  unsigned long duration = pulseIn(echoPins[i], HIGH, ECHO_TIMEOUT_US);

  Range &range = ranges[i];
  range.measuredAt = millis();
  range.cm = duration ? duration / 58.0f : -1;
  if (duration && i == FRONT_LEFT) {
    range.cm = (range.cm + 2.5190400f) / 0.9687973f;
  }
  if (i == RIGHT) {
    bool detected = range.cm >= 0 && range.cm <= RIGHT_SEEN_CM;
    rightHits = detected ? min(3, rightHits + 1) : 0;
    rightMisses = detected ? 0 : min(3, rightMisses + 1);
  } else {
    bool clear = range.cm < 0 || range.cm > FRONT_RESUME_CM;
    range.consecutiveClear = clear ? min(3, range.consecutiveClear + 1) : 0;
  }
}

bool sonarFresh(Sensor i) {
  return ranges[i].measuredAt != 0 && millis() - ranges[i].measuredAt < 230;
}

bool rangesReady() {
  return sonarFresh(FRONT_LEFT) && sonarFresh(RIGHT) &&
         (!USE_FRONT_RIGHT || sonarFresh(FRONT_RIGHT));
}

float nearestFront() {
  float nearest = 1000;
  if (ranges[FRONT_LEFT].cm >= 0) nearest = ranges[FRONT_LEFT].cm;
  if (USE_FRONT_RIGHT && ranges[FRONT_RIGHT].cm >= 0) {
    nearest = min(nearest, ranges[FRONT_RIGHT].cm);
  }
  return nearest;
}

bool frontClear() {
  return ranges[FRONT_LEFT].consecutiveClear >= 3 &&
         (!USE_FRONT_RIGHT || ranges[FRONT_RIGHT].consecutiveClear >= 3);
}

void updatePose() {
  uint32_t counts[4];
  readTicks(counts);
  uint32_t now = millis();
  uint32_t elapsed = now - lastPoseAt;
  lastPoseAt = now;
  bool drivingForward = driveLeft > 0 && driveRight > 0;
  bool rolling = drivingForward ||
      (driveLeft == 0 && driveRight == 0 && (int32_t)(coastUntil - now) > 0);
  float travel = 0;
  for (uint8_t i = 0; i < 4; ++i) {
    uint32_t delta = counts[i] - previousTicks[i];
    previousTicks[i] = counts[i];
    if (delta) lastWheelTick[i] = now;
    if (calibrated && rolling) travel += delta * cmPerTick[i] * 0.25f;
  }
  if (!calibrated) {
    if (drivingForward) {
      float pwmScale = (driveLeft + driveRight) / (2.0f * TIMING_REFERENCE_PWM);
      travel = elapsed * 0.001f * TIMING_CM_PER_SECOND * pwmScale;
    } else if (coastDistance > 0) {
      uint32_t coastElapsed = now - coastStartedAt;
      uint32_t previousCoastElapsed = coastElapsed > elapsed ? coastElapsed - elapsed : 0;
      float from = min(previousCoastElapsed, TIMING_COAST_MS) / (float)TIMING_COAST_MS;
      float to = min(coastElapsed, TIMING_COAST_MS) / (float)TIMING_COAST_MS;
      travel = coastDistance * (to - from) * (2.0f - to - from);
      if (coastElapsed >= TIMING_COAST_MS) coastDistance = 0;
    }
  }

  // During a pivot the encoder counts are unsigned, so ignore them for
  // translation. Integrate forward motion using the gyro's course-relative yaw.
  float angle = ((yaw + previousPoseYaw) * 0.5f - cruiseHeading) * DEG_TO_RAD;
  previousPoseYaw = yaw;
  x += travel * cosf(angle);
  y += travel * sinf(angle);
  pathCm += travel;
}

bool wheelStalled() {
  for (uint8_t i = 0; i < 4; ++i) {
    int speed = i % 2 == 0 ? driveLeft : driveRight;
    if (speed && millis() - lastWheelTick[i] > 850) return true;
  }
  return false;
}

float angleError(float target) {
  float error = fmodf(target - yaw, 360.0f);
  if (error > 180) error -= 360;
  if (error < -180) error += 360;
  return error;
}

void steer(int forward, int trim) {
  // Keep both sides rolling forward so unsigned wheel counts remain usable.
  drive(forward + trim, forward - trim);
}

void startTurn(float target, State next) {
  drive(0, 0);
  turnTarget = target;
  afterTurn = next;
  state = TURN;
  stateStarted = millis();
  turnSettling = false;
}

void finishTurn() {
  if (afterTurn == FIRST_FACE || afterTurn == SECOND_FACE || afterTurn == RETURN_LINE) {
    rightHits = rightMisses = 0;
    faceSeen = edgeCleared = false;
    faceStartPath = pathCm;
  } else {
    ranges[FRONT_LEFT].consecutiveClear = 0;
    ranges[FRONT_RIGHT].consecutiveClear = 0;
  }
  state = afterTurn;
  stateStarted = millis();
}

void updateTurn() {
  uint32_t now = millis();
  if (now - stateStarted > 6000) {
    stop(F("STOP_TURN_TIMEOUT"));
    return;
  }
  // Let the chassis coast to a stop before changing travel mode.
  if (now - stateStarted < 180 || coastDistance > 0) return;
  float error = angleError(turnTarget);
  if (turnSettling) {
    if (now - turnSettledAt < 200) return;
    if (fabsf(error) <= 6 && fabsf(yawRate) < 10) {
      finishTurn();
      return;
    }
    turnSettling = false;
  }
  if (fabsf(error) <= 3 || fabsf(error - yawRate * 0.13f) <= 3) {
    drive(0, 0);
    turnSettling = true;
    turnSettledAt = now;
    return;
  }
  int pwm = error > 0 ? TURN_PWM : -TURN_PWM;
  drive(pwm, -pwm);
}

void holdHeading(int pwm, float heading) {
  float error = angleError(heading);
  steer(pwm, constrain((int)(2 * error - 0.2f * yawRate), -12, 12));
}

bool bypassClearance(float front) {
  if (front <= EDGE_FRONT_STOP_CM) {
    stop(F("STOP_FRONT_BLOCKED_DURING_BYPASS"));
    return false;
  }
  if (ranges[RIGHT].cm >= 0 && ranges[RIGHT].cm < RIGHT_MIN_CM) {
    stop(F("STOP_RIGHT_TOO_CLOSE"));
    return false;
  }
  return true;
}

void followBoxFace(float front) {
  // Wait for a fresh side ping after the turn before moving forward.
  if (rightHits == 0 && rightMisses == 0) return;
  if (!bypassClearance(front)) return;

  bool firstFace = state == FIRST_FACE;
  if (!faceSeen) {
    if (rightHits >= 2) {
      faceSeen = true;
      Serial.println(firstFace ? F("FIRST_FACE_SEEN") : F("SECOND_FACE_SEEN"));
    } else if (pathCm - faceStartPath >= FACE_ACQUIRE_LIMIT_CM) {
      stop(firstFace ? F("STOP_FIRST_FACE_NOT_SEEN") : F("STOP_SECOND_FACE_NOT_SEEN"));
      return;
    }
  }

  if (faceSeen && !edgeCleared && rightMisses >= 2) {
    edgeCleared = true;
    edgeLostPath = pathCm;
    Serial.println(firstFace ? F("FIRST_EDGE_LOST") : F("SECOND_EDGE_LOST"));
  }
  if (edgeCleared && rightHits >= 2) {
    edgeCleared = false;  // A repeat echo can indicate a brief dropout.
  }
  if (edgeCleared && rightMisses >= 2 && pathCm - edgeLostPath >= CORNER_ADVANCE_CM) {
    startTurn(cruiseHeading + (firstFace ? 0 : 90), firstFace ? SECOND_FACE : RETURN_LINE);
    Serial.println(firstFace ? F("FIRST_CORNER_RIGHT_90") : F("SECOND_CORNER_RIGHT_90"));
    return;
  }
  holdHeading(FOLLOW_PWM, cruiseHeading + (firstFace ? -90 : 0));
}

void navigate() {
  if (!gyroReady() || !rangesReady() || wheelStalled()) {
    stop(F("STOP_GYRO_SONAR_OR_ENCODER"));
    return;
  }
  if (bypassActive && millis() - bypassStarted > BYPASS_LIMIT_MS) {
    stop(F("STOP_BYPASS_TIMEOUT"));
    return;
  }

  float front = nearestFront();
  switch (state) {
    case CRUISE: {
      if (front <= APPROACH_STOP_CM) {
        bypassActive = true;
        bypassStarted = millis();
        detectedX = x;
        detectedY = y;
        startTurn(cruiseHeading - 90, FIRST_FACE);
        Serial.print(F("OBSTACLE_TURNING_LEFT,FL_CM="));
        Serial.print(ranges[FRONT_LEFT].cm, 1);
        Serial.print(F(",FR_CM="));
        Serial.println(USE_FRONT_RIGHT ? ranges[FRONT_RIGHT].cm : -1.0f, 1);
      } else {
        uint32_t now = millis();
        if (front <= APPROACH_SLOW_CM) lastApproachAt = now;
        int forward = now - lastApproachAt < APPROACH_HOLD_MS ? APPROACH_PWM : CRUISE_PWM;
        float courseCorrection = constrain(-y * 0.65f, -14.0f, 14.0f);
        holdHeading(forward, cruiseHeading + courseCorrection);
      }
      break;
    }
    case TURN:
      updateTurn();
      break;
    case FIRST_FACE:
    case SECOND_FACE:
      followBoxFace(front);
      break;
    case RETURN_LINE:
      if (rightHits == 0 && rightMisses == 0) break;
      if (!bypassClearance(front)) break;
      // Start braking before the line to allow for the measured coast.
      if (y >= -TIMING_COAST_CM) {
        startTurn(cruiseHeading, CHECK_EXIT);
        Serial.println(F("REJOIN_LEFT_90"));
      } else {
        holdHeading(FOLLOW_PWM, cruiseHeading + 90);
      }
      break;
    case CHECK_EXIT:
      if (front <= FRONT_STOP_CM || fabsf(y) > REJOIN_TOLERANCE_CM) {
        stop(F("STOP_REJOIN_BLOCKED_OR_OFF_LINE"));
      } else if (frontClear()) {
        state = CRUISE;
        lastApproachAt = millis();
        bypassActive = false;
        Serial.println(F("BYPASS_COMPLETE"));
        Serial.print(F("BYPASS_DISPLACEMENT_CM="));
        Serial.println(hypotf(x - detectedX, y - detectedY), 1);
      } else if (millis() - stateStarted > 1800) {
        stop(F("STOP_FRONT_CLEARANCE_UNCONFIRMED"));
      }
      break;
    case STOPPED:
      break;
  }
}

void calibrateWheels() {
  if (state != STOPPED) return;
  uint32_t counts[4];
  readTicks(counts);
  if (!measuring) {
    for (uint8_t i = 0; i < 4; ++i) calibrationStart[i] = counts[i];
    measuring = true;
    Serial.println(F("PUSH_FORWARD_100_CM_THEN_SEND_C"));
    return;
  }
  measuring = false;
  calibrated = true;
  Serial.print(F("ENCODER_COUNTS FL(D18) FR(D19) RL(D3) RR(D2)"));
  for (uint8_t i = 0; i < 4; ++i) {
    counts[i] -= calibrationStart[i];
    Serial.print(' ');
    Serial.print(counts[i]);
    if (counts[i] < MIN_CALIBRATION_TICKS) calibrated = false;
  }
  Serial.println();
  if (!calibrated) {
    Serial.println(F("CALIBRATION_FAILED_CHECK_ENCODERS (minimum 16 ticks per wheel)"));
    return;
  }
  for (uint8_t i = 0; i < 4; ++i) cmPerTick[i] = CALIBRATION_CM / counts[i];
  Serial.println(F("WHEEL_CALIBRATION_READY"));
}

void readCommands() {
  while (Serial.available()) {
    char command = Serial.read();
    if (command == 'S' || command == 's') {
      measuring = false;
      stop(F("STOPPED"));
    } else if (command == 'C' || command == 'c') {
      calibrateWheels();
    } else if ((command == 'A' || command == 'a') && state == STOPPED) {
      if (!gyroReady() || !rangesReady()) {
        Serial.println(F("WAIT_FOR_GYRO_AND_SONAR"));
        continue;
      }
      if (measuring || coastDistance > 0 || (int32_t)(coastUntil - millis()) > 0) {
        Serial.println(F("FINISH_CALIBRATION_OR_WAIT_UNTIL_STOPPED"));
        continue;
      }
      readTicks(previousTicks);
      coastUntil = millis();
      lastPoseAt = coastUntil;
      lastApproachAt = millis() - APPROACH_HOLD_MS;
      x = y = pathCm = 0;
      cruiseHeading = previousPoseYaw = yaw;
      state = CRUISE;
      Serial.println(calibrated ? F("DRIVING,ENCODER_ODOMETRY") : F("DRIVING,TIMING_ODOMETRY"));
    } else if (command == 'P' || command == 'p') {
      Serial.print(F("FRONT_LEFT,RIGHT,FRONT_RIGHT,YAW,X,Y,STATE,DISPLACEMENT_CM,"));
      for (const Range &range : ranges) {
        Serial.print(range.cm, 1);
        Serial.print(',');
      }
      Serial.print(yaw - cruiseHeading, 1); Serial.print(',');
      Serial.print(x, 1); Serial.print(',');
      Serial.print(y, 1); Serial.print(',');
      Serial.print(state); Serial.print(',');
      Serial.println(hypotf(x, y), 1);
    }
  }
}

void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < 4; ++i) {
    const Wheel &wheel = wheels[i];
    pinMode(wheel.pwm, OUTPUT);
    pinMode(wheel.dir1, OUTPUT);
    pinMode(wheel.dir2, OUTPUT);
    analogWrite(wheel.pwm, 0);
    pinMode(wheel.encoder, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(wheel.encoder), encoderHandlers[i], RISING);
  }
  for (uint8_t i = 0; i < (USE_FRONT_RIGHT ? SENSOR_COUNT : 2); ++i) {
    pinMode(triggerPins[i], OUTPUT);
    digitalWrite(triggerPins[i], LOW);
    pinMode(echoPins[i], INPUT);
  }
  RobotGyro::begin();
  Serial.println(F("KEEP_STILL_FOR_GYRO. A: DRIVE, S: STOP, P: STATUS, C: OPTIONAL CALIBRATION"));
}

void loop() {
  updateYaw();
  updatePose();
  readCommands();
  if (state != STOPPED) navigate();
  updateRanges();
}
