#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>

// Full Bluetooth-started task sequence for Arduino Mega 2560 + MOEBIUS motor board.
// Bluetooth: BT-04A on Serial3 (TX3=D14, RX3=D15), SPP at 9600 baud.
// Commands: C = calibrate gyro; A = return to calibrated heading and run task;
// S = emergency stop at any stage.

#define BT Serial3

namespace StartGyro {
const int8_t Z_SIGN = -1;
const unsigned long SAMPLE_US = 5000UL;
uint8_t address = 0;
bool ready = false;
bool fault = false;
float biasDps = 0.0f;
float yawDeg = 0.0f;
float rateDps = 0.0f;
unsigned long sampledUs = 0;

bool readBytes(uint8_t reg, uint8_t *data, uint8_t count) {
#if defined(WIRE_HAS_TIMEOUT)
  Wire.clearWireTimeoutFlag();
#endif
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
#if defined(WIRE_HAS_TIMEOUT)
  if (Wire.getWireTimeoutFlag()) return false;
#endif
  if (Wire.requestFrom(address, count, (uint8_t)true) != count) return false;
#if defined(WIRE_HAS_TIMEOUT)
  if (Wire.getWireTimeoutFlag()) return false;
#endif
  for (uint8_t i = 0; i < count; ++i) {
    if (!Wire.available()) return false;
    data[i] = (uint8_t)Wire.read();
  }
  return true;
}

bool writeRegister(uint8_t reg, uint8_t value) {
#if defined(WIRE_HAS_TIMEOUT)
  Wire.clearWireTimeoutFlag();
#endif
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  bool ok = Wire.endTransmission(true) == 0;
#if defined(WIRE_HAS_TIMEOUT)
  ok = ok && !Wire.getWireTimeoutFlag();
#endif
  return ok;
}

bool readRawZ(float &dps) {
  uint8_t bytes[2];
  if (!readBytes(0x47, bytes, 2)) return false;
  uint16_t word = ((uint16_t)bytes[0] << 8) | bytes[1];
  int32_t raw = word >= 32768U ? (int32_t)word - 65536L : (int32_t)word;
  dps = raw / 131.0f;
  return true;
}

bool initializeSensor() {
  bool found = false;
  for (uint8_t candidate = 0x68; candidate <= 0x69; ++candidate) {
    address = candidate;
    uint8_t identity = 0;
    if (readBytes(0x75, &identity, 1) && identity == 0x68) {
      found = true;
      break;
    }
  }
  return found &&
      writeRegister(0x6B, 0x01) && writeRegister(0x6C, 0x00) &&
      writeRegister(0x1B, 0x00) && writeRegister(0x1A, 0x03) &&
      writeRegister(0x19, 4);
}

void update() {
  if (!ready || fault) return;
  unsigned long now = micros();
  unsigned long elapsed = now - sampledUs;
  if (elapsed < SAMPLE_US) return;

  float rawRate;
  if (!readRawZ(rawRate) || fabs(rawRate) > 245.0f) {
    fault = true;
    return;
  }

  now = micros();
  elapsed = now - sampledUs;
  float nextRate = Z_SIGN * (rawRate - biasDps);
  if (elapsed <= 100000UL)
    yawDeg += (rateDps + nextRate) * 0.5f * (elapsed / 1000000.0f);
  rateDps = nextRate;
  sampledUs = now;
}
}

// Front sonars: FL=A3/A6, FR=D32/A8.
const uint8_t FL_TRIG = A3, FL_ECHO = A6;
const uint8_t FR_TRIG = 32, FR_ECHO = A8;
// Side sonars (user-confirmed): left=D30/A2, right=A10/A9.
const uint8_t SIDE_L_TRIG = 30, SIDE_L_ECHO = A2;
const uint8_t SIDE_R_TRIG = A10, SIDE_R_ECHO = A9;

// TCS3200/TCS230 pins from PinFinder2.ino.
const uint8_t COLOUR_S0 = A14, COLOUR_S1 = A15;
const uint8_t COLOUR_S2 = A11, COLOUR_S3 = A12, COLOUR_OUT = A13;

// MOEBIUS motor board pin map.
const uint8_t PWM_PIN[4]  = {12, 8, 9, 5};
const uint8_t DIR1_PIN[4] = {34, 37, 43, A4};
const uint8_t DIR2_PIN[4] = {35, 36, 42, A5};
const bool INVERT[4]      = {false, true, false, true};

const int MOTOR_PWM = 50;
const int8_t FORWARD_SIGN = -1;

// Sensor-specific front calibration fits from UltrasonicParallelAlign.ino.
const float D32_SLOPE = 0.9622105f, D32_INTERCEPT = -1.0619189f;
const float A3_SLOPE = 0.9687973f, A3_INTERCEPT = -2.5190400f;
// Side calibration fits: raw distance = slope * actual distance + intercept.
const float SIDE_L_SLOPE = 0.9792500f, SIDE_L_INTERCEPT = 0.4285714f;
const float SIDE_R_SLOPE = 0.9916786f, SIDE_R_INTERCEPT = -0.5414286f;

const float PARALLEL_FAR_DISTANCE_CM = 50.0f;
const float PARALLEL_FAR_TOLERANCE_CM = 1.0f;
const float PARALLEL_NEAR_TOLERANCE_CM = 0.5f;
const float SIDE_TURN_THRESHOLD_CM = 0.5f;
const float DISTANCE_TARGET_TOLERANCE_CM = 0.5f;
const float TARGET_DISTANCE_1_CM = 28.0f;
// Calibrated FL/FR average converted from the requested raw average of 70.0 cm.
const float TARGET_ROUTE_DISTANCE_CM = 74.3537f;
// Calibrated targets converted from the requested raw sensor readings.
const float PRE_COLOUR_FRONT_TARGET_CM = 32.9241f;  // FL/FR raw average 30.0
const float PRE_COLOUR_RIGHT_TARGET_CM = 69.1166f;  // right raw 68.0
// Coarse calibrated approach before the final raw-value parking loop.
const float FINAL_FRONT_AVERAGE_TARGET_CM = 23.0f;
const float FINAL_LEFT_PARK_FL_RAW_CM = 15.5f;
const float FINAL_LEFT_PARK_FR_RAW_CM = 16.0f;
const float FINAL_LEFT_PARK_RIGHT_RAW_CM = 20.6f;
const float FINAL_RIGHT_PARK_FL_RAW_CM = 16.8f;
const float FINAL_RIGHT_PARK_FR_RAW_CM = 17.0f;
const float FINAL_RIGHT_PARK_LEFT_RAW_CM = 21.5f;
const float FINAL_RAW_TOLERANCE_CM = 0.3f;
const float PRE_COLOUR_TOLERANCE_CM = 1.0f;

const unsigned long ECHO_TIMEOUT_US = 25000UL;
const unsigned long PING_GAP_MS = 60UL;
const unsigned long SETTLE_AFTER_PULSE_MS = 250UL;
const unsigned long COAST_SETTLE_MS = 450UL;
const unsigned long DISTANCE_SETTLE_MS = 500UL;
const unsigned long MAX_APPROACH_MS = 30000UL;
const float COARSE_APPROACH_MARGIN_CM = 8.0f;
const float FIRST_APPROACH_COARSE_MARGIN_CM = 11.0f;
const float FINAL_APPROACH_COARSE_MARGIN_CM = 11.68f;
const unsigned long STRAFE_FRONT_BASE_MS = 450UL;
const unsigned long STRAFE_REAR_BASE_MS = 100UL;
const uint8_t STRAFE_SCALE_PERCENT[] = {100, 80, 70, 50};
const uint8_t STRAFE_SCALE_COUNT = sizeof(STRAFE_SCALE_PERCENT) /
                                   sizeof(STRAFE_SCALE_PERCENT[0]);
const uint8_t MEDIAN_SAMPLES = 5;
const uint8_t STABLE_READS_REQUIRED = 3;
const uint8_t MAX_FRONT_TURN_PULSES = 40;
const uint8_t MAX_DISTANCE_CORRECTIONS = 20;
const uint8_t MAX_PARALLEL_ATTEMPTS = 3;
const uint8_t MAX_FINAL_SIDE_PULSES = 40;
const uint8_t MAX_FINAL_ALIGNMENT_CYCLES = 20;
const uint8_t MAX_PRE_COLOUR_SIDE_PULSES = 30;
const uint8_t MAX_FINAL_RAW_WHEEL_PULSES = 60;
const uint8_t FINAL_RAW_STABLE_READS = 3;
const unsigned long FINAL_RAW_WHEEL_BASE_MS = 100UL;

// Wheel vectors follow MecanumSerialControl.ino; drivePattern() applies -vector*PWM.
const int8_t TURN_LEFT[4]     = {-1,  1, -1,  1};
const int8_t TURN_RIGHT[4]    = { 1, -1,  1, -1};
const int8_t STRAFE_LEFT[4]   = {-1,  1,  1, -1};
const int8_t STRAFE_RIGHT[4]  = { 1, -1, -1,  1};

const unsigned long RIGHT_TURN_MS = 1100UL;
const unsigned long LEFT_TURN_MS = 950UL;
const unsigned long COLOUR_PRETURN_FORWARD_MS = 160UL;
const unsigned long GYRO_PRETURN_FORWARD_MS = 1000UL;
const unsigned long PARALLEL_TURN_PULSE_MS = 100UL;
const uint8_t PARALLEL_PULSE_SCALE_PERCENT[] = {100, 80, 70, 50};
const uint8_t PARALLEL_PULSE_SCALE_COUNT =
    sizeof(PARALLEL_PULSE_SCALE_PERCENT) / sizeof(PARALLEL_PULSE_SCALE_PERCENT[0]);
const float PARALLEL_FL_OFFSET_CM = 0.5f;
const unsigned long COLOUR_PULSE_TIMEOUT_US = 30000UL;

struct FrontReading {
  float flRaw, frRaw, flCm, frCm, averageCm;
};
struct SideReading {
  float leftRaw, rightRaw, leftCm, rightCm;
};

// Minimal SSD1306 renderer for the existing 0x3C I2C OLED; no extra library.
const uint8_t OLED_GLYPHS[][5] PROGMEM = {
  {0x3e,0x51,0x49,0x45,0x3e},{0,0x42,0x7f,0x40,0},{0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4b,0x31},{0x18,0x14,0x12,0x7f,0x10},{0x27,0x45,0x45,0x45,0x39},{0x3c,0x4a,0x49,0x49,0x30},{1,0x71,9,5,3},{0x36,0x49,0x49,0x49,0x36},{6,0x49,0x49,0x29,0x1e},
  {0x7e,0x11,0x11,0x11,0x7e},{0x7f,0x49,0x49,0x49,0x36},{0x3e,0x41,0x41,0x41,0x22},{0x7f,0x41,0x41,0x22,0x1c},{0x7f,0x49,0x49,0x49,0x41},{0x7f,9,9,9,1},{0x3e,0x41,0x49,0x49,0x7a},{0x7f,8,8,8,0x7f},{0,0x41,0x7f,0x41,0},{0x20,0x40,0x41,0x3f,1},{0x7f,8,0x14,0x22,0x41},{0x7f,0x40,0x40,0x40,0x40},{0x7f,2,0x0c,2,0x7f},{0x7f,4,8,0x10,0x7f},{0x3e,0x41,0x41,0x41,0x3e},{0x7f,9,9,9,6},{0x3e,0x41,0x51,0x21,0x5e},{0x7f,9,0x19,0x29,0x46},{0x46,0x49,0x49,0x49,0x31},{1,1,0x7f,1,1},{0x3f,0x40,0x40,0x40,0x3f},{0x1f,0x20,0x40,0x20,0x1f},{0x7f,0x20,0x18,0x20,0x7f},{0x63,0x14,8,0x14,0x63},{3,4,0x78,4,3},{0x61,0x51,0x49,0x45,0x43}
};
bool oledReady = false;
float oledFl = -1.0f, oledFr = -1.0f, oledLeft = -1.0f, oledRight = -1.0f;
char oledExtra[22] = "";
char oledAction[22] = "READY";
bool oledStopped = false;

void oledCommand(uint8_t value) {
  if (!oledReady) return;
  Wire.beginTransmission(0x3C);
  Wire.write(0x00);
  Wire.write(value);
  Wire.endTransmission();
}

void oledLine(uint8_t page, const char *text) {
  if (!oledReady) return;
  oledCommand(0xB0 + page); oledCommand(0x00); oledCommand(0x10);
  const uint8_t length = strlen(text);
  for (uint8_t block = 0; block < 8; block++) {
    Wire.beginTransmission(0x3C);
    Wire.write(0x40);
    for (uint8_t column = 0; column < 16; column++) {
      const uint8_t x = block * 16 + column;
      const uint8_t index = x / 6;
      const uint8_t glyphColumn = x % 6;
      const char c = index < length ? text[index] : ' ';
      int8_t glyph = c >= '0' && c <= '9' ? c - '0' :
          c >= 'A' && c <= 'Z' ? 10 + c - 'A' : -1;
      uint8_t pixels = 0;
      if (glyphColumn < 5 && glyph >= 0)
        pixels = pgm_read_byte(&OLED_GLYPHS[glyph][glyphColumn]);
      if (c == '.' && glyphColumn == 2) pixels = 0x60;
      if (c == '-' && glyphColumn < 5) pixels = 0x08;
      if (c == '=' && glyphColumn < 5) pixels = 0x14;
      Wire.write(pixels);
    }
    Wire.endTransmission();
  }
}

void oledRefresh() {
  if (!oledReady) return;
  char text[22];
  if (oledFl >= 0.0f && oledFr >= 0.0f)
    snprintf(text, sizeof(text), "FL=%d FR=%d", (int)lroundf(oledFl),
             (int)lroundf(oledFr));
  else
    strcpy(text, "FL=-- FR=--");
  oledLine(0, text);

  if (oledLeft >= 0.0f && oledRight >= 0.0f)
    snprintf(text, sizeof(text), "L=%d R=%d", (int)lroundf(oledLeft),
             (int)lroundf(oledRight));
  else
    strcpy(text, "L=-- R=--");
  oledLine(2, text);
  oledLine(4, oledExtra);
  oledLine(7, oledAction);
}

void oledSetAction(const char *text) {
  strncpy(oledAction, text, sizeof(oledAction) - 1);
  oledAction[sizeof(oledAction) - 1] = '\0';
  oledStopped = false;
  oledRefresh();
}

void oledShowText(const char *text) {
  oledStopped = false;
  oledSetAction(text);
}

void oledStop(const char *reason) {
  snprintf(oledAction, sizeof(oledAction), "STOP %s", reason);
  oledStopped = true;
  oledRefresh();
}

void oledShowPair(const char *title, const char *firstLabel, float first,
                  const char *secondLabel, float second) {
  if (!oledReady) return;
  if (strcmp(title, "FRONT") == 0) {
    oledFl = first; oledFr = second;
  } else if (strcmp(title, "SIDE") == 0) {
    oledLeft = first; oledRight = second;
  } else {
    snprintf(oledExtra, sizeof(oledExtra), "%s=%d %s=%d", firstLabel,
             (int)lroundf(first), secondLabel, (int)lroundf(second));
  }
  oledRefresh();
}

void oledShowGyro(float zDegrees) {
  if (!oledReady) return;
  snprintf(oledExtra, sizeof(oledExtra), "Z=%d", (int)lroundf(zDegrees));
  oledRefresh();
}

void initializeOled() {
  Wire.beginTransmission(0x3C);
  oledReady = Wire.endTransmission() == 0;
  if (!oledReady) return;
  const uint8_t init[] = {0xAE,0xD5,0x80,0xA8,0x3F,0xD3,0x00,0x40,
      0x8D,0x14,0x20,0x02,0xA1,0xC8,0xDA,0x12,0x81,0x7F,0xD9,0xF1,
      0xDB,0x40,0xA4,0xA6,0xAF};
  for (uint8_t i = 0; i < sizeof(init); i++) oledCommand(init[i]);
  for (uint8_t page = 0; page < 8; page++) oledLine(page, "");
  oledShowText("READY");
}

bool stopRequested = false;
bool taskRunning = false;
bool lastColourTurnRight = false;

bool approachToDistance(float targetCm, const __FlashStringHelper *stageName,
                        float targetToleranceCm = DISTANCE_TARGET_TOLERANCE_CM,
                        float coarseMarginCm = COARSE_APPROACH_MARGIN_CM);
bool driveStraightPulse(bool forward, unsigned long durationMs);

float parallelToleranceFor(const FrontReading &r) {
  return r.averageCm > PARALLEL_FAR_DISTANCE_CM
      ? PARALLEL_FAR_TOLERANCE_CM : PARALLEL_NEAR_TOLERANCE_CM;
}

void setMotor(uint8_t index, int speed) {
  if (INVERT[index]) speed = -speed;
  digitalWrite(DIR1_PIN[index], speed < 0 ? HIGH : LOW);
  digitalWrite(DIR2_PIN[index], speed > 0 ? HIGH : LOW);
  analogWrite(PWM_PIN[index], abs(speed));
}

void stopMotors() {
  for (uint8_t i = 0; i < 4; i++) setMotor(i, 0);
}

float wrapStartAngle(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle <= -180.0f) angle += 360.0f;
  return angle;
}

bool calibrateStartGyro() {
  stopMotors();
  StartGyro::ready = false;
  StartGyro::fault = false;
  oledShowText("GYRO CAL");
  BT.println(F("CAL,KEEP_CAR_STILL"));

  if (!StartGyro::initializeSensor()) {
    oledStop("GYRO");
    BT.println(F("CAL_FAILED,MPU6050_NOT_FOUND_OR_INIT_FAILED"));
    return false;
  }

  delay(300);
  const uint16_t sampleCount = 240;
  float sum = 0.0f;
  float sumSquares = 0.0f;
  for (uint16_t i = 0; i < sampleCount; ++i) {
    float sample;
    if (!StartGyro::readRawZ(sample)) {
      oledStop("GYRO");
      BT.println(F("CAL_FAILED,GYRO_READ_ERROR"));
      return false;
    }
    sum += sample;
    sumSquares += sample * sample;
    delay(5);
  }

  const float mean = sum / sampleCount;
  const float variance = fmax(0.0f, sumSquares / sampleCount - mean * mean);
  if (fabs(mean) > 10.0f || variance > 4.0f) {
    oledStop("GYRO");
    BT.println(F("CAL_FAILED,CAR_MOVED,KEEP_STILL_AND_SEND_C_AGAIN"));
    return false;
  }

  StartGyro::biasDps = mean;
  StartGyro::yawDeg = 0.0f;
  StartGyro::rateDps = 0.0f;
  StartGyro::sampledUs = micros();
  StartGyro::ready = true;
  oledShowGyro(0.0f);
  oledSetAction("PLACE CAR");
  BT.print(F("CAL_READY,BIAS_DPS=")); BT.println(mean, 3);
  BT.println(F("PLACE_CAR_THEN_SEND_A"));
  return true;
}

bool checkStopSignal() {
  while (BT.available() > 0) {
    char c = BT.read();
    if (c == 'S' || c == 's') {
      if (!stopRequested) BT.println(F("EVENT,EMERGENCY_STOP"));
      stopRequested = true;
      stopMotors();
      oledStop("USER");
    }
    // Ignore A and line endings during an active task.
  }
  return stopRequested;
}

bool waitWithStop(unsigned long durationMs) {
  unsigned long start = millis();
  while (millis() - start < durationMs) {
    if (checkStopSignal()) return false;
    delay(1);
  }
  return !checkStopSignal();
}

void drivePatternNow(const int8_t pattern[4]) {
  for (uint8_t i = 0; i < 4; i++) setMotor(i, -pattern[i] * MOTOR_PWM);
}

bool drivePatternPulse(const int8_t pattern[4], unsigned long durationMs,
                       unsigned long settleMs = SETTLE_AFTER_PULSE_MS) {
  if (checkStopSignal()) return false;
  drivePatternNow(pattern);
  bool completed = waitWithStop(durationMs);
  stopMotors();
  if (!completed || stopRequested) return false;
  return waitWithStop(settleMs);
}

bool driveStagedStrafePulse(const int8_t pattern[4], uint8_t scaleIndex) {
  if (checkStopSignal()) return false;
  if (scaleIndex >= STRAFE_SCALE_COUNT) scaleIndex = STRAFE_SCALE_COUNT - 1;

  const uint8_t percent = STRAFE_SCALE_PERCENT[scaleIndex];
  const unsigned long frontMs = STRAFE_FRONT_BASE_MS * percent / 100UL;
  const unsigned long rearMs = STRAFE_REAR_BASE_MS * percent / 100UL;

  // Start the front pair separately because it needs more time to overcome stiction.
  for (uint8_t i = 0; i < 2; i++) setMotor(i, -pattern[i] * MOTOR_PWM);
  if (!waitWithStop(frontMs)) {
    stopMotors();
    return false;
  }
  stopMotors();

  for (uint8_t i = 2; i < 4; i++) setMotor(i, -pattern[i] * MOTOR_PWM);
  if (!waitWithStop(rearMs)) {
    stopMotors();
    return false;
  }
  stopMotors();
  return waitWithStop(DISTANCE_SETTLE_MS);
}

bool driveWheelSidePulse(bool leftSide, bool forward, uint8_t scaleIndex) {
  if (checkStopSignal()) return false;
  if (scaleIndex >= STRAFE_SCALE_COUNT) scaleIndex = STRAFE_SCALE_COUNT - 1;

  const unsigned long durationMs = FINAL_RAW_WHEEL_BASE_MS *
      STRAFE_SCALE_PERCENT[scaleIndex] / 100UL;
  const int speed = FORWARD_SIGN * (forward ? 1 : -1) * MOTOR_PWM;
  const uint8_t first = leftSide ? 0 : 1;
  const uint8_t second = leftSide ? 2 : 3;

  stopMotors();
  setMotor(first, speed);
  setMotor(second, speed);
  if (!waitWithStop(durationMs)) {
    stopMotors();
    return false;
  }
  stopMotors();
  return waitWithStop(DISTANCE_SETTLE_MS);
}

bool returnToCalibratedHeadingByTime() {
  oledSetAction("GYRO RETURN");
  if (!StartGyro::ready || StartGyro::fault) {
    oledStop("GYRO");
    BT.println(F("STOP,GYRO_NOT_READY,SEND_C_FIRST"));
    return false;
  }

  StartGyro::update();
  if (StartGyro::fault) {
    oledStop("GYRO");
    BT.println(F("STOP,GYRO_READ_FAILED,SEND_C_AGAIN"));
    return false;
  }

  const float currentAngle = wrapStartAngle(StartGyro::yawDeg);
  oledShowGyro(currentAngle);
  const float degrees = fabs(currentAngle);
  const bool turnRight = currentAngle > 0.0f;
  const unsigned long durationMs = degrees <= 0.5f ? 0UL :
      (unsigned long)lroundf(degrees *
          (turnRight ? RIGHT_TURN_MS : LEFT_TURN_MS) / 90.0f);

  BT.print(F("START_RETURN,YAW_DEG=")); BT.print(currentAngle, 2);
  BT.print(F(",TURN="));
  BT.print(durationMs == 0 ? F("NONE") : (turnRight ? F("RIGHT") : F("LEFT")));
  BT.print(F(",RETURN_DEG=")); BT.print(degrees, 2);
  BT.print(F(",TIME_MS=")); BT.println(durationMs);

  // Save the angle, then stop I2C access before any motor movement.
  StartGyro::ready = false;

  BT.print(F("EVENT,FORWARD_BEFORE_GYRO_RETURN,MS="));
  BT.println(GYRO_PRETURN_FORWARD_MS);
  oledSetAction("GYRO FORWARD");
  if (!driveStraightPulse(true, GYRO_PRETURN_FORWARD_MS)) return false;

  if (durationMs == 0) {
    BT.println(F("EVENT,CALIBRATED_HEADING_ALREADY_IN_RANGE"));
    return true;
  }

  oledSetAction("GYRO TURN");
  drivePatternNow(turnRight ? TURN_RIGHT : TURN_LEFT);
  const unsigned long started = millis();
  while (millis() - started < durationMs) {
    while (BT.available() > 0) {
      char command = BT.read();
      if (command == 'S' || command == 's') {
        stopRequested = true;
        stopMotors();
        BT.println(F("EVENT,EMERGENCY_STOP_DURING_START_RETURN"));
        return false;
      }
    }
    delay(2);
  }

  stopMotors();
  BT.println(F("EVENT,START_RETURN_COMPLETE,ESTIMATED_YAW=0"));
  return waitWithStop(300);
}

void driveStraightNow(bool forward) {
  int sign = FORWARD_SIGN * (forward ? 1 : -1);
  for (uint8_t i = 0; i < 4; i++) setMotor(i, sign * MOTOR_PWM);
}

bool driveStraightPulse(bool forward, unsigned long durationMs) {
  if (checkStopSignal()) return false;
  driveStraightNow(forward);
  bool completed = waitWithStop(durationMs);
  stopMotors();
  if (!completed || stopRequested) return false;
  return waitWithStop(DISTANCE_SETTLE_MS);
}

float calibrateA3(float rawCm) {
  return (rawCm - A3_INTERCEPT) / A3_SLOPE;
}

float calibrateD32(float rawCm) {
  return (rawCm - D32_INTERCEPT) / D32_SLOPE - 0.3f;
}

float calibrateSideLeft(float rawCm) {
  return (rawCm - SIDE_L_INTERCEPT) / SIDE_L_SLOPE;
}

float calibrateSideRight(float rawCm) {
  return (rawCm - SIDE_R_INTERCEPT) / SIDE_R_SLOPE;
}

float readRawDistanceCm(uint8_t trig, uint8_t echo) {
  if (checkStopSignal()) return -1.0f;
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long duration = pulseIn(echo, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0 || checkStopSignal()) return -1.0f;
  return duration / 58.0f;
}

bool readRawPair(uint8_t trig1, uint8_t echo1, uint8_t trig2, uint8_t echo2,
                 float &first, float &second) {
  first = readRawDistanceCm(trig1, echo1);
  if (stopRequested || !waitWithStop(PING_GAP_MS)) return false;
  second = readRawDistanceCm(trig2, echo2);
  if (stopRequested || !waitWithStop(PING_GAP_MS)) return false;
  return first > 0.0f && second > 0.0f;
}

float medianOf(float *values, uint8_t count) {
  for (uint8_t i = 1; i < count; i++) {
    float value = values[i];
    int8_t j = (int8_t)i - 1;
    while (j >= 0 && values[j] > value) {
      values[j + 1] = values[j];
      j--;
    }
    values[j + 1] = value;
  }
  return values[count / 2];
}

bool readFrontOnce(FrontReading &reading) {
  float fl, fr;
  if (!readRawPair(FL_TRIG, FL_ECHO, FR_TRIG, FR_ECHO, fl, fr)) {
    if (!stopRequested) oledStop("NO FRONT");
    return false;
  }
  reading.flRaw = fl;
  reading.frRaw = fr;
  reading.flCm = calibrateA3(fl);
  reading.frCm = calibrateD32(fr);
  reading.averageCm = (reading.flCm + reading.frCm) / 2.0f;
  oledShowPair("FRONT", "FL", reading.flRaw, "FR", reading.frRaw);
  return true;
}

bool readFrontMedian(FrontReading &reading) {
  float flValues[MEDIAN_SAMPLES], frValues[MEDIAN_SAMPLES];
  uint8_t count = 0;
  for (uint8_t i = 0; i < MEDIAN_SAMPLES; i++) {
    float fl, fr;
    if (readRawPair(FL_TRIG, FL_ECHO, FR_TRIG, FR_ECHO, fl, fr)) {
      flValues[count] = fl;
      frValues[count] = fr;
      count++;
    }
    if (stopRequested) return false;
  }
  if (count < 3) {
    if (!stopRequested) oledStop("NO FRONT");
    return false;
  }
  reading.flRaw = medianOf(flValues, count);
  reading.frRaw = medianOf(frValues, count);
  reading.flCm = calibrateA3(reading.flRaw);
  reading.frCm = calibrateD32(reading.frRaw);
  reading.averageCm = (reading.flCm + reading.frCm) / 2.0f;
  oledShowPair("FRONT", "FL", reading.flRaw, "FR", reading.frRaw);
  return true;
}

bool readSideMedian(SideReading &reading) {
  float leftValues[MEDIAN_SAMPLES], rightValues[MEDIAN_SAMPLES];
  uint8_t count = 0;
  while (count < MEDIAN_SAMPLES) {
    float left, right;
    if (readRawPair(SIDE_L_TRIG, SIDE_L_ECHO, SIDE_R_TRIG, SIDE_R_ECHO,
                    left, right)) {
      leftValues[count] = left;
      rightValues[count] = right;
      count++;
    }
    if (stopRequested) return false;
  }
  reading.leftRaw = medianOf(leftValues, count);
  reading.rightRaw = medianOf(rightValues, count);
  reading.leftCm = calibrateSideLeft(reading.leftRaw);
  reading.rightCm = calibrateSideRight(reading.rightRaw);
  oledShowPair("SIDE", "L", reading.leftRaw, "R", reading.rightRaw);
  return true;
}

bool readSelectedFinalSideMedianRaw(float &rawCm) {
  float values[MEDIAN_SAMPLES];
  uint8_t count = 0;
  const uint8_t trig = lastColourTurnRight ? SIDE_L_TRIG : SIDE_R_TRIG;
  const uint8_t echo = lastColourTurnRight ? SIDE_L_ECHO : SIDE_R_ECHO;
  while (count < MEDIAN_SAMPLES) {
    const float value = readRawDistanceCm(trig, echo);
    if (value > 0.0f) values[count++] = value;
    if (stopRequested || !waitWithStop(PING_GAP_MS)) return false;
  }
  rawCm = medianOf(values, count);
  if (lastColourTurnRight) oledLeft = rawCm;
  else oledRight = rawCm;
  oledRefresh();
  return true;
}

void printFront(const __FlashStringHelper *label, const FrontReading &r) {
  BT.print(label);
  BT.print(F(" | FL raw/cal=")); BT.print(r.flRaw, 1); BT.print('/'); BT.print(r.flCm, 1);
  BT.print(F(" | FR raw/cal=")); BT.print(r.frRaw, 1); BT.print('/'); BT.print(r.frCm, 1);
  BT.print(F(" | AVG_CAL=")); BT.println(r.averageCm, 1);
}

void printSide(const __FlashStringHelper *label, const SideReading &r) {
  BT.print(label);
  BT.print(F(" | LEFT raw/cal=")); BT.print(r.leftRaw, 1); BT.print('/'); BT.print(r.leftCm, 1);
  BT.print(F(" | RIGHT raw/cal=")); BT.print(r.rightRaw, 1); BT.print('/'); BT.print(r.rightCm, 1);
  BT.print(F(" | CAL_LEFT-RIGHT=")); BT.println(r.leftCm - r.rightCm, 1);
}

bool alignFront(float toleranceOverrideCm) {
  oledSetAction("PARALLEL");
  uint8_t stableReads = 0;
  int8_t previousDirection = 0;
  uint8_t reversalCount = 0;
  uint8_t pulseScaleIndex = 0;
  for (uint8_t pulse = 0; pulse < MAX_FRONT_TURN_PULSES; pulse++) {
    FrontReading r;
    if (!readFrontMedian(r)) {
      stopMotors();
      BT.println(stopRequested ? F("STOP,FRONT_ALIGN_CANCELLED") : F("STOP,FRONT_SONAR_NO_ECHO"));
      return false;
    }
    printFront(F("FRONT_ALIGN"), r);
    const float adjustedFlCm = r.flCm + PARALLEL_FL_OFFSET_CM;
    float difference = r.frCm - adjustedFlCm;
    float toleranceCm = toleranceOverrideCm >= 0.0f
        ? toleranceOverrideCm : parallelToleranceFor(r);
    BT.print(F("PARALLEL_FL_CAL_PLUS_0.5=")); BT.print(adjustedFlCm, 1);
    BT.print(F(",PARALLEL_DIFF=")); BT.print(fabs(difference), 1);
    BT.print(F(",TOLERANCE=")); BT.println(toleranceCm, 1);
    if (fabs(difference) <= toleranceCm) {
      if (++stableReads >= STABLE_READS_REQUIRED) {
        stopMotors();
        BT.println(F("EVENT,FRONT_PARALLEL"));
        return true;
      }
      continue;
    }

    stableReads = 0;
    const int8_t direction = difference > 0.0f ? -1 : 1;
    if (previousDirection != 0 && direction != previousDirection) {
      reversalCount++;
      if (reversalCount % 2 == 0 &&
          pulseScaleIndex + 1 < PARALLEL_PULSE_SCALE_COUNT) {
        pulseScaleIndex++;
        BT.print(F("EVENT,PARALLEL_PULSE_REDUCED,MS="));
        BT.println(PARALLEL_TURN_PULSE_MS *
                   PARALLEL_PULSE_SCALE_PERCENT[pulseScaleIndex] / 100UL);
      }
    }
    previousDirection = direction;

    const unsigned long pulseMs = PARALLEL_TURN_PULSE_MS *
        PARALLEL_PULSE_SCALE_PERCENT[pulseScaleIndex] / 100UL;
    BT.print(direction < 0 ? F("CORRECT,TURN_LEFT,PULSE_MS=")
                           : F("CORRECT,TURN_RIGHT,PULSE_MS="));
    BT.println(pulseMs);
    if (!drivePatternPulse(direction < 0 ? TURN_LEFT : TURN_RIGHT, pulseMs))
      return false;
  }
  stopMotors();
  oledStop("PAR LIMIT");
  BT.println(F("STOP,FRONT_ALIGN_PULSE_LIMIT"));
  return false;
}

bool alignFrontVerified(float toleranceOverrideCm) {
  for (uint8_t attempt = 1; attempt <= MAX_PARALLEL_ATTEMPTS; attempt++) {
    BT.print(F("STAGE,FRONT_PARALLEL_ATTEMPT,")); BT.println(attempt);
    if (alignFront(toleranceOverrideCm)) {
      BT.println(F("EVENT,FRONT_PARALLEL_VERIFIED_ROTATION_ONLY"));
      return true;
    }
    if (stopRequested) return false;
    FrontReading retryCheck;
    if (!readFrontMedian(retryCheck)) {
      stopMotors();
      BT.println(F("STOP,FRONT_SONAR_NO_ECHO"));
      return false;
    }
    BT.println(F("FRONT_ALIGN_ATTEMPT_FAILED,RETRY"));
  }
  stopMotors();
  FrontReading beforeRecovery;
  if (!readFrontMedian(beforeRecovery)) {
    BT.println(stopRequested ? F("STOP,PARALLEL_RECOVERY_CANCELLED") : F("STOP,FRONT_SONAR_NO_ECHO"));
    return false;
  }

  const float recoveryTargetCm = beforeRecovery.averageCm - 30.0f;
  if (recoveryTargetCm < FINAL_FRONT_AVERAGE_TARGET_CM) {
    BT.println(F("STOP,PARALLEL_FAILED_3_TIMES_NO_ROOM_TO_ADVANCE_30CM"));
    return false;
  }

  BT.print(F("RECOVERY,ADVANCE_30CM,START_AVG=")); BT.print(beforeRecovery.averageCm, 1);
  BT.print(F(",TARGET_AVG=")); BT.println(recoveryTargetCm, 1);
  if (!approachToDistance(recoveryTargetCm, F("PARALLEL_RECOVERY_ADVANCE_30CM"))) return false;

  BT.println(F("STAGE,PARALLEL_RETRY_AFTER_30CM"));
  if (alignFront(toleranceOverrideCm)) {
    BT.println(F("EVENT,FRONT_PARALLEL_VERIFIED_AFTER_30CM_ROTATION_ONLY"));
    return true;
  }

  stopMotors();
  oledStop("PAR FAIL");
  BT.println(F("STOP,FRONT_PARALLEL_NOT_VERIFIED_AFTER_30CM"));
  return false;
}

unsigned long correctionPulseForDistance(float errorCm) {
  float magnitude = fabs(errorCm);
  if (magnitude > 10.0f) return 180;
  if (magnitude > 5.0f) return 120;
  if (magnitude > 2.0f) return 70;
  return 35;
}

bool approachToDistance(float targetCm, const __FlashStringHelper *stageName,
                        float targetToleranceCm, float coarseMarginCm) {
  oledSetAction("FRONT MOVE");
  BT.print(F("STAGE,APPROACH_TARGET_CM,")); BT.println(targetCm, 1);
  FrontReading r;
  if (!readFrontOnce(r)) {
    BT.println(stopRequested ? F("STOP,APPROACH_CANCELLED") : F("STOP,FRONT_SONAR_NO_ECHO"));
    return false;
  }

  const float coarseStopCm = targetCm + coarseMarginCm;
  unsigned long approachStart = millis();
  if (r.averageCm > coarseStopCm) driveStraightNow(true);
  while (r.averageCm > coarseStopCm) {
    if (checkStopSignal()) { stopMotors(); return false; }
    if (millis() - approachStart >= MAX_APPROACH_MS) {
      stopMotors(); oledStop("TIMEOUT");
      BT.println(F("STOP,APPROACH_TIMEOUT")); return false;
    }
    if (!readFrontOnce(r)) {
      stopMotors();
      BT.println(stopRequested ? F("STOP,APPROACH_CANCELLED") : F("STOP,FRONT_SONAR_NO_ECHO"));
      return false;
    }
  }

  stopMotors();
  BT.print(F("EVENT,COARSE_APPROACH_STOPPED,")); BT.println(r.averageCm, 1);
  if (!waitWithStop(COAST_SETTLE_MS)) return false;

  for (uint8_t correction = 0; correction < MAX_DISTANCE_CORRECTIONS; correction++) {
    if (!readFrontMedian(r)) {
      stopMotors();
      BT.println(stopRequested ? F("STOP,DISTANCE_CORRECTION_CANCELLED") : F("STOP,FRONT_SONAR_NO_ECHO"));
      return false;
    }
    printFront(stageName, r);
    float errorCm = r.averageCm - targetCm; // Positive = too far; negative = too close.

    if (fabs(errorCm) <= targetToleranceCm) {
      stopMotors();
      BT.print(F("EVENT,DISTANCE_TARGET_REACHED,")); BT.println(targetCm, 1);
      return true;
    }

    bool moveForward = errorCm > 0.0f;
    unsigned long pulseMs = correctionPulseForDistance(errorCm);
    BT.print(F("CORRECT_DISTANCE,"));
    BT.print(moveForward ? F("FORWARD,") : F("REVERSE,"));
    BT.print(F("ERROR_CM=")); BT.print(errorCm, 1);
    BT.print(F(",PULSE_MS=")); BT.println(pulseMs);
    if (!driveStraightPulse(moveForward, pulseMs)) return false;
  }

  stopMotors();
  oledStop("FRONT LIMIT");
  BT.println(F("STOP,DISTANCE_CORRECTION_LIMIT"));
  return false;
}

bool turnBySideAndReturn() {
  oledSetAction("SIDE TURN");
  BT.println(F("STAGE,SIDE_COMPARE_THEN_TURN"));
  SideReading r;
  if (!readSideMedian(r)) {
    stopMotors();
    BT.println(stopRequested ? F("STOP,SIDE_READ_CANCELLED") : F("STOP,SIDE_SONAR_NO_ECHO"));
    return false;
  }

  printSide(F("SIDE_COMPARE"), r);
  float difference = r.leftCm - r.rightCm;
  bool turnedLeft = false;
  bool turnedRight = false;

  if (difference > SIDE_TURN_THRESHOLD_CM) {
    BT.println(F("EVENT,SIDE_LEFT_GREATER,TURN_LEFT_MS=950"));
    if (!drivePatternPulse(TURN_LEFT, LEFT_TURN_MS, 300)) return false;
    turnedLeft = true;
  } else if (difference < -SIDE_TURN_THRESHOLD_CM) {
    BT.println(F("EVENT,SIDE_RIGHT_GREATER,TURN_RIGHT_MS=1100"));
    if (!drivePatternPulse(TURN_RIGHT, RIGHT_TURN_MS, 300)) return false;
    turnedRight = true;
  } else {
    BT.println(F("EVENT,SIDE_DIFFERENCE_WITHIN_0.5CM,NO_TURN"));
  }

  if (!approachToDistance(TARGET_ROUTE_DISTANCE_CM,
                          F("ROUTE_FRONT_CAL_FROM_RAW_AVG_70"))) return false;

  if (turnedLeft) {
    BT.println(F("EVENT,RETURN_TURN_RIGHT_MS=1100"));
    if (!drivePatternPulse(TURN_RIGHT, RIGHT_TURN_MS, 300)) return false;
  } else if (turnedRight) {
    BT.println(F("EVENT,RETURN_TURN_LEFT_MS=950"));
    if (!drivePatternPulse(TURN_LEFT, LEFT_TURN_MS, 300)) return false;
  }

  BT.println(F("EVENT,RETURN_TURN_COMPLETE"));
  return true;
}

bool alignRightSideBeforeColour() {
  oledSetAction("PRE COLOR POS");
  BT.print(F("STAGE,PRE_COLOUR_RIGHT_CAL_TARGET_CM,"));
  BT.println(PRE_COLOUR_RIGHT_TARGET_CM, 2);
  uint8_t strafeScaleIndex = 0;
  int8_t previousErrorSign = 0;

  for (uint8_t pulse = 0; pulse < MAX_PRE_COLOUR_SIDE_PULSES; ++pulse) {
    SideReading r;
    if (!readSideMedian(r)) {
      stopMotors();
      BT.println(stopRequested ? F("STOP,PRE_COLOUR_SIDE_CANCELLED")
                               : F("STOP,PRE_COLOUR_SIDE_NO_ECHO"));
      return false;
    }
    printSide(F("PRE_COLOUR_SIDE_CHECK"), r);

    const float rightErrorCm = r.rightCm - PRE_COLOUR_RIGHT_TARGET_CM;
    if (fabs(rightErrorCm) <= PRE_COLOUR_TOLERANCE_CM) {
      BT.print(F("EVENT,PRE_COLOUR_RIGHT_IN_RANGE,CM=")); BT.println(r.rightCm, 1);
      return true;
    }

    const int8_t errorSign = rightErrorCm > 0.0f ? 1 : -1;
    if (previousErrorSign != 0 && errorSign != previousErrorSign) {
      if (strafeScaleIndex + 1 < STRAFE_SCALE_COUNT) strafeScaleIndex++;
      BT.print(F("EVENT,PRE_COLOUR_SIDE_REVERSED,STRAFE_SCALE_PERCENT="));
      BT.println(STRAFE_SCALE_PERCENT[strafeScaleIndex]);
    }
    previousErrorSign = errorSign;

    // Right too far: move right. Right too close: move left.
    const bool strafeRight = rightErrorCm > 0.0f;
    const int8_t *strafePattern = strafeRight ? STRAFE_RIGHT : STRAFE_LEFT;
    const uint8_t percent = STRAFE_SCALE_PERCENT[strafeScaleIndex];
    const unsigned long frontMs = STRAFE_FRONT_BASE_MS * percent / 100UL;
    const unsigned long rearMs = STRAFE_REAR_BASE_MS * percent / 100UL;

    BT.print(F("CORRECT,PRE_COLOUR_RIGHT,"));
    BT.print(rightErrorCm > 0.0f ? F("RIGHT_DISTANCE_HIGH,")
                                : F("RIGHT_DISTANCE_LOW,"));
    BT.print(strafeRight ? F("STRAFE_RIGHT,") : F("STRAFE_LEFT,"));
    BT.print(F("SCALE=")); BT.print(percent);
    BT.print(F("%,FRONT_MS=")); BT.print(frontMs);
    BT.print(F(",REAR_MS=")); BT.println(rearMs);

    if (!driveStagedStrafePulse(strafePattern, strafeScaleIndex)) return false;
  }

  stopMotors();
  oledStop("SIDE LIMIT");
  BT.println(F("STOP,PRE_COLOUR_RIGHT_CORRECTION_LIMIT"));
  return false;
}

float selectedFinalSideRawDistance(const SideReading &r) {
  return lastColourTurnRight ? r.leftRaw : r.rightRaw;
}

float selectedFinalSideRawTargetCm() {
  return lastColourTurnRight ? FINAL_RIGHT_PARK_LEFT_RAW_CM
                             : FINAL_LEFT_PARK_RIGHT_RAW_CM;
}

bool alignSelectedSideToParkingTarget(uint8_t &strafeScaleIndex,
                                      int8_t &previousErrorSign) {
  oledSetAction("SIDE ADJUST");
  const __FlashStringHelper *sensorName = lastColourTurnRight
      ? F("LEFT") : F("RIGHT");
  BT.print(F("STAGE,FINAL_SIDE_TARGET,"));
  BT.print(sensorName);
  BT.print(F(",RAW_CM="));
  BT.println(selectedFinalSideRawTargetCm(), 2);

  for (uint8_t pulse = 0; pulse < MAX_FINAL_SIDE_PULSES; pulse++) {
    float currentRawCm;
    if (!readSelectedFinalSideMedianRaw(currentRawCm)) {
      stopMotors();
      BT.println(F("STOP,FINAL_SIDE_CANCELLED"));
      return false;
    }
    BT.print(F("FINAL_SIDE_CHECK,"));
    BT.print(sensorName); BT.print(F("_RAW=")); BT.println(currentRawCm, 1);
    float errorCm = currentRawCm - selectedFinalSideRawTargetCm();
    if (fabs(errorCm) <= FINAL_RAW_TOLERANCE_CM) {
      BT.print(F("EVENT,FINAL_SIDE_RAW_IN_RANGE,CM=")); BT.println(currentRawCm, 1);
      return true;
    }

    int8_t errorSign = errorCm > 0.0f ? 1 : -1;
    if (previousErrorSign != 0 && errorSign != previousErrorSign) {
      if (strafeScaleIndex + 1 < STRAFE_SCALE_COUNT) strafeScaleIndex++;
      BT.print(F("EVENT,FINAL_SIDE_DIRECTION_REVERSED,STRAFE_SCALE_PERCENT="));
      BT.println(STRAFE_SCALE_PERCENT[strafeScaleIndex]);
    }
    previousErrorSign = errorSign;

    const bool strafeLeft = lastColourTurnRight ? errorCm > 0.0f : errorCm < 0.0f;
    const int8_t *strafePattern = strafeLeft ? STRAFE_LEFT : STRAFE_RIGHT;
    const uint8_t percent = STRAFE_SCALE_PERCENT[strafeScaleIndex];
    const unsigned long frontMs = STRAFE_FRONT_BASE_MS * percent / 100UL;
    const unsigned long rearMs = STRAFE_REAR_BASE_MS * percent / 100UL;
    BT.print(F("CORRECT,FINAL_SIDE,"));
    BT.print(errorCm > 0.0f ? F("DISTANCE_HIGH,") : F("DISTANCE_LOW,"));
    BT.print(strafeLeft ? F("STRAFE_LEFT,") : F("STRAFE_RIGHT,"));
    BT.print(F("SCALE=")); BT.print(percent);
    BT.print(F("%,FRONT_MS=")); BT.print(frontMs);
    BT.print(F(",REAR_MS=")); BT.println(rearMs);
    if (!driveStagedStrafePulse(strafePattern, strafeScaleIndex)) return false;
  }

  stopMotors();
  oledStop("SIDE LIMIT");
  BT.println(F("STOP,FINAL_SIDE_RAW_CORRECTION_LIMIT"));
  return false;
}

float selectedFinalFLRawTargetCm() {
  return lastColourTurnRight ? FINAL_RIGHT_PARK_FL_RAW_CM
                             : FINAL_LEFT_PARK_FL_RAW_CM;
}

float selectedFinalFRRawTargetCm() {
  return lastColourTurnRight ? FINAL_RIGHT_PARK_FR_RAW_CM
                             : FINAL_LEFT_PARK_FR_RAW_CM;
}

bool alignFinalFrontRawByWheelSides(uint8_t &leftScaleIndex,
                                    int8_t &leftPreviousErrorSign,
                                    uint8_t &rightScaleIndex,
                                    int8_t &rightPreviousErrorSign) {
  oledSetAction("FRONT ADJUST");
  uint8_t stableReads = 0;
  BT.print(F("STAGE,FINAL_RAW_FRONT_TARGETS,FL="));
  BT.print(selectedFinalFLRawTargetCm(), 1);
  BT.print(F(",FR="));
  BT.println(selectedFinalFRRawTargetCm(), 1);

  for (uint8_t pulse = 0; pulse < MAX_FINAL_RAW_WHEEL_PULSES; pulse++) {
    FrontReading front;
    if (!readFrontMedian(front)) {
      stopMotors();
      BT.println(stopRequested ? F("STOP,FINAL_RAW_FRONT_CANCELLED")
                               : F("STOP,FRONT_SONAR_NO_ECHO"));
      return false;
    }
    printFront(F("FINAL_RAW_FRONT_CHECK"), front);

    const float flError = front.flRaw - selectedFinalFLRawTargetCm();
    const float frError = front.frRaw - selectedFinalFRRawTargetCm();
    const bool flOk = fabs(flError) <= FINAL_RAW_TOLERANCE_CM;
    const bool frOk = fabs(frError) <= FINAL_RAW_TOLERANCE_CM;

    if (flOk && frOk) {
      stableReads++;
      BT.print(F("EVENT,FINAL_RAW_FRONT_STABLE,"));
      BT.print(stableReads); BT.print('/'); BT.println(FINAL_RAW_STABLE_READS);
      if (stableReads >= FINAL_RAW_STABLE_READS) {
        stopMotors();
        BT.println(F("EVENT,FINAL_RAW_FRONT_PASSED_3_READS"));
        return true;
      }
      continue;
    }
    stableReads = 0;

    const bool adjustLeft = !flOk;
    const float error = adjustLeft ? flError : frError;
    const int8_t errorSign = error > 0.0f ? 1 : -1;
    uint8_t &scaleIndex = adjustLeft ? leftScaleIndex : rightScaleIndex;
    int8_t &previousErrorSign = adjustLeft
        ? leftPreviousErrorSign : rightPreviousErrorSign;

    if (previousErrorSign != 0 && errorSign != previousErrorSign) {
      if (scaleIndex + 1 < STRAFE_SCALE_COUNT) scaleIndex++;
      BT.print(adjustLeft ? F("EVENT,FL_DIRECTION_REVERSED,SCALE_PERCENT=")
                          : F("EVENT,FR_DIRECTION_REVERSED,SCALE_PERCENT="));
      BT.println(STRAFE_SCALE_PERCENT[scaleIndex]);
    }
    previousErrorSign = errorSign;

    const bool moveForward = error > 0.0f;
    const unsigned long pulseMs = FINAL_RAW_WHEEL_BASE_MS *
        STRAFE_SCALE_PERCENT[scaleIndex] / 100UL;
    BT.print(F("CORRECT,FINAL_RAW_FRONT,"));
    BT.print(adjustLeft ? F("LEFT_WHEELS,") : F("RIGHT_WHEELS,"));
    BT.print(moveForward ? F("FORWARD,") : F("REVERSE,"));
    BT.print(F("ERROR_CM=")); BT.print(error, 1);
    BT.print(F(",PULSE_MS=")); BT.println(pulseMs);

    if (!driveWheelSidePulse(adjustLeft, moveForward, scaleIndex)) return false;
  }

  stopMotors();
  oledStop("FRONT LIMIT");
  BT.println(F("STOP,FINAL_RAW_FRONT_CORRECTION_LIMIT"));
  return false;
}

bool finalPositionFeedbackLoop() {
  oledSetAction("FINAL PARK");
  BT.println(F("STAGE,FINAL_RAW_PARKING_LOOP,SIDE_THEN_FL_FR_THEN_SIDE_RECHECK"));
  uint8_t sideStrafeScaleIndex = 0;
  int8_t sidePreviousErrorSign = 0;
  uint8_t leftWheelScaleIndex = 0;
  int8_t leftWheelPreviousErrorSign = 0;
  uint8_t rightWheelScaleIndex = 0;
  int8_t rightWheelPreviousErrorSign = 0;

  for (uint8_t cycle = 1; cycle <= MAX_FINAL_ALIGNMENT_CYCLES; cycle++) {
    BT.print(F("FINAL_RAW_CYCLE,")); BT.println(cycle);

    if (!alignSelectedSideToParkingTarget(sideStrafeScaleIndex,
                                          sidePreviousErrorSign)) return false;
    if (!alignFinalFrontRawByWheelSides(leftWheelScaleIndex,
                                        leftWheelPreviousErrorSign,
                                        rightWheelScaleIndex,
                                        rightWheelPreviousErrorSign)) return false;

    float currentSideRawCm;
    if (!readSelectedFinalSideMedianRaw(currentSideRawCm)) {
      stopMotors();
      BT.println(F("STOP,FINAL_SIDE_RAW_RECHECK_CANCELLED"));
      return false;
    }
    BT.print(F("FINAL_SIDE_RAW_RECHECK=")); BT.println(currentSideRawCm, 1);
    const float sideError = currentSideRawCm - selectedFinalSideRawTargetCm();
    if (fabs(sideError) <= FINAL_RAW_TOLERANCE_CM) {
      stopMotors();
      BT.println(F("EVENT,FINAL_RAW_PARKING_COMPLETE"));
      return true;
    }

    BT.println(F("EVENT,FINAL_SIDE_RAW_MOVED,REPEAT_CYCLE"));
  }

  stopMotors();
  oledStop("PARK LIMIT");
  BT.println(F("STOP,FINAL_RAW_PARKING_CYCLE_LIMIT"));
  return false;
}

void configureColourSensor() {
  pinMode(COLOUR_OUT, INPUT);
  pinMode(COLOUR_S0, OUTPUT); pinMode(COLOUR_S1, OUTPUT);
  pinMode(COLOUR_S2, OUTPUT); pinMode(COLOUR_S3, OUTPUT);
  digitalWrite(COLOUR_S0, HIGH);
  digitalWrite(COLOUR_S1, LOW); // 20% frequency scale, same as PinFinder2.
  digitalWrite(COLOUR_S2, LOW);
  digitalWrite(COLOUR_S3, LOW);
  delay(10);
}

bool readColourChannel(bool s2, bool s3, float &value) {
  digitalWrite(COLOUR_S2, s2);
  digitalWrite(COLOUR_S3, s3);
  if (!waitWithStop(3)) return false;
  unsigned long samples[5];
  for (uint8_t i = 0; i < 5; i++) {
    if (checkStopSignal()) return false;
    samples[i] = pulseIn(COLOUR_OUT, LOW, COLOUR_PULSE_TIMEOUT_US);
    if (samples[i] == 0 || checkStopSignal()) return false;
  }
  for (uint8_t i = 1; i < 5; i++) {
    unsigned long v = samples[i];
    int8_t j = (int8_t)i - 1;
    while (j >= 0 && samples[j] > v) { samples[j + 1] = samples[j]; j--; }
    samples[j + 1] = v;
  }
  value = 100000.0f / samples[2];
  return true;
}

bool readRGB(float rgb[3]) {
  float r, g, b;
  if (!readColourChannel(LOW, LOW, r) ||
      !readColourChannel(HIGH, HIGH, g) ||
      !readColourChannel(LOW, HIGH, b)) return false;
  float total = r + g + b;
  if (total <= 0.0f) return false;
  rgb[0] = r / total; rgb[1] = g / total; rgb[2] = b / total;
  return true;
}

bool readAverageRGB(float rgb[3]) {
  float sum[3] = {0.0f, 0.0f, 0.0f};
  for (uint8_t sample = 0; sample < 5; sample++) {
    float one[3];
    if (!readRGB(one)) return false;
    for (uint8_t channel = 0; channel < 3; channel++) sum[channel] += one[channel];
  }
  for (uint8_t channel = 0; channel < 3; channel++) rgb[channel] = sum[channel] / 5.0f;
  return true;
}

const char *classifyColour(const float rgb[3]) {
  if (rgb[1] > rgb[0]) return "GREEN";
  if (rgb[0] > rgb[1]) return "RED";
  return "UNKNOWN";
}

bool readPadAndTurn() {
  oledSetAction("READ COLOR");
  BT.println(F("STAGE,READ_COLOUR_PAD"));
  float rgb[3];
  if (!readAverageRGB(rgb)) {
    stopMotors();
    if (!stopRequested) oledStop("COLOR");
    BT.println(stopRequested ? F("STOP,COLOUR_READ_CANCELLED") : F("STOP,COLOUR_SENSOR_ERROR"));
    return false;
  }

  const char *colour = classifyColour(rgb);
  oledShowPair("COLOR", "R", rgb[0] * 100.0f, "G", rgb[1] * 100.0f);
  BT.print(F("COLOUR,")); BT.print(colour);
  BT.print(F(",RGB=")); BT.print(rgb[0], 3); BT.print(',');
  BT.print(rgb[1], 3); BT.print(','); BT.println(rgb[2], 3);
  if (strcmp(colour, "UNKNOWN") == 0) {
    oledStop("COLOR UNKNOWN");
    BT.println(F("STOP,UNKNOWN_COLOUR_NO_TURN"));
    return false;
  }

  bool turnRight = strcmp(colour, "RED") == 0;
  lastColourTurnRight = turnRight;
  const int8_t *pattern = turnRight ? TURN_RIGHT : TURN_LEFT;
  unsigned long duration = turnRight ? RIGHT_TURN_MS : LEFT_TURN_MS;
  BT.print(F("STAGE,COLOUR_FORWARD,MS="));
  BT.println(COLOUR_PRETURN_FORWARD_MS);
  oledSetAction("COLOR FORWARD");
  if (!driveStraightPulse(true, COLOUR_PRETURN_FORWARD_MS)) return false;

  BT.print(F("STAGE,COLOUR_TURN,"));
  BT.print(turnRight ? F("RIGHT,MS=") : F("LEFT,MS="));
  BT.println(duration);

  oledSetAction("COLOR TURN");
  drivePatternNow(pattern);
  bool completed = waitWithStop(duration);
  stopMotors();
  if (!completed || stopRequested) return false;
  BT.println(F("EVENT,COLOUR_TURN_COMPLETE"));
  return waitWithStop(250);
}

bool runFullTask() {
  BT.println(F("TASK_START"));

  if (!returnToCalibratedHeadingByTime()) return false;
  if (checkStopSignal()) return false;

  if (!approachToDistance(TARGET_DISTANCE_1_CM, F("SETTLED_AT_28CM"),
                          DISTANCE_TARGET_TOLERANCE_CM,
                          FIRST_APPROACH_COARSE_MARGIN_CM)) return false;
  if (checkStopSignal()) return false;

  BT.println(F("STAGE,FRONT_PARALLEL_AT_28CM"));
  if (!alignFrontVerified(-1.0f)) return false;
  if (checkStopSignal()) return false;

  if (!turnBySideAndReturn()) return false;
  if (checkStopSignal()) return false;

  BT.println(F("STAGE,FRONT_PARALLEL_AFTER_RETURN_TURN"));
  if (!alignFrontVerified(-1.0f)) return false;
  if (checkStopSignal()) return false;

  if (!approachToDistance(PRE_COLOUR_FRONT_TARGET_CM,
                          F("PRE_COLOUR_FRONT_CAL_FROM_RAW_AVG_30"),
                          PRE_COLOUR_TOLERANCE_CM)) return false;
  if (checkStopSignal()) return false;

  if (!alignRightSideBeforeColour()) return false;
  if (checkStopSignal()) return false;

  BT.println(F("STAGE,FRONT_PARALLEL_BEFORE_COLOUR"));
  if (!alignFrontVerified(-1.0f)) return false;
  if (checkStopSignal()) return false;

  if (!readPadAndTurn()) return false;
  if (checkStopSignal()) return false;

  if (!alignFrontVerified(0.8f)) return false;
  if (checkStopSignal()) return false;

  if (!approachToDistance(FINAL_FRONT_AVERAGE_TARGET_CM,
                          F("SETTLED_AT_FINAL_FRONT_AVERAGE"),
                          DISTANCE_TARGET_TOLERANCE_CM,
                          FINAL_APPROACH_COARSE_MARGIN_CM)) return false;
  if (checkStopSignal()) return false;

  if (!finalPositionFeedbackLoop()) return false;
  if (checkStopSignal()) return false;

  stopMotors();
  oledShowText("FINISH");
  BT.println(F("TASK_COMPLETE"));
  return true;
}

void runTaskFromBluetooth() {
  stopRequested = false;
  oledStopped = false;
  taskRunning = true;
  bool success = runFullTask();
  stopMotors();
  taskRunning = false;
  StartGyro::ready = false;
  if (!success) {
    if (!oledStopped) oledStop(stopRequested ? "USER" : "STAGE FAIL");
    BT.println(stopRequested ? F("TASK_ABORTED,USER_STOP") : F("TASK_ABORTED,STAGE_FAILED"));
  }
  BT.println(F("READY,SEND_C_BEFORE_NEXT_TASK"));
}

void setup() {
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(PWM_PIN[i], OUTPUT);
    pinMode(DIR1_PIN[i], OUTPUT);
    pinMode(DIR2_PIN[i], OUTPUT);
  }
  stopMotors();

  pinMode(FL_TRIG, OUTPUT); pinMode(FL_ECHO, INPUT);
  pinMode(FR_TRIG, OUTPUT); pinMode(FR_ECHO, INPUT);
  pinMode(SIDE_L_TRIG, OUTPUT); pinMode(SIDE_L_ECHO, INPUT);
  pinMode(SIDE_R_TRIG, OUTPUT); pinMode(SIDE_R_ECHO, INPUT);
  digitalWrite(FL_TRIG, LOW); digitalWrite(FR_TRIG, LOW);
  digitalWrite(SIDE_L_TRIG, LOW); digitalWrite(SIDE_R_TRIG, LOW);
  configureColourSensor();

  BT.begin(9600);
  Wire.begin();
  Wire.setClock(100000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000UL, true);
#endif
  initializeOled();
  BT.println(F("BUILD,FULL_TASK_C_CALIBRATE_A_START"));
  BT.println(F("READY,SEND_C_TO_CALIBRATE"));
  BT.println(F("COMMANDS,C=CALIBRATE,A=START_FULL_TASK,S=EMERGENCY_STOP"));
}

void loop() {
  static unsigned long lastGyroReportMs = 0;
  if (!taskRunning && StartGyro::ready && !StartGyro::fault) {
    StartGyro::update();
    if (millis() - lastGyroReportMs >= 500UL) {
      BT.print(F("GYRO,")); BT.print(millis());
      BT.print(','); BT.print(wrapStartAngle(StartGyro::yawDeg), 2);
      BT.print(','); BT.print(StartGyro::rateDps, 2);
      BT.println(F(",MONITOR"));
      oledShowGyro(wrapStartAngle(StartGyro::yawDeg));
      lastGyroReportMs = millis();
    }
  }

  while (BT.available() > 0) {
    char command = BT.read();
    if (command == 'C' || command == 'c') {
      if (taskRunning) {
        BT.println(F("EVENT,TASK_BUSY"));
      } else {
        stopRequested = false;
        calibrateStartGyro();
      }
    } else if (command == 'A' || command == 'a') {
      if (taskRunning) {
        BT.println(F("EVENT,TASK_BUSY"));
      } else if (!StartGyro::ready || StartGyro::fault) {
        BT.println(F("ERROR,SEND_C_TO_CALIBRATE_BEFORE_A"));
      } else {
        runTaskFromBluetooth();
      }
    } else if (command == 'S' || command == 's') {
      stopMotors();
      if (taskRunning) {
        stopRequested = true;
        BT.println(F("EVENT,EMERGENCY_STOP"));
      } else {
        BT.println(F("EVENT,STOPPED"));
      }
    }
  }
}
