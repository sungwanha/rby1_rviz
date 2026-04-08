#include <Wire.h>

#define SDA_PIN 6
#define SCL_PIN 5
#define AS5600_ADDR 0x36

// ====== 사용자 조정 변수 ======
const float ROTATION_DEG_FOR_TRAVEL = 80.0;   // 몇 도 회전할 때
const float LINEAR_TRAVEL_MM        = 80.0;   // 몇 mm 이동하는지

const float ZERO_OFFSET_DEG = 266.75;            // 영점 보정값(필요시 수정)
const bool INVERT_DIRECTION = false;          // 방향 반전 필요하면 true
// ==============================

uint16_t readRawAngle() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0C);  // RAW ANGLE high byte
  if (Wire.endTransmission(false) != 0) {
    return 0xFFFF;
  }

  if (Wire.requestFrom(AS5600_ADDR, (uint8_t)2) != 2) {
    return 0xFFFF;
  }

  uint8_t highByte = Wire.read();
  uint8_t lowByte  = Wire.read();

  return ((highByte << 8) | lowByte) & 0x0FFF;
}

float rawToAngleDeg(uint16_t raw) {
  return raw * 360.0f / 4096.0f;
}

float getAdjustedAngle(float angleDeg) {
  float adjustedAngle = angleDeg - ZERO_OFFSET_DEG;

  if (INVERT_DIRECTION) {
    adjustedAngle = -adjustedAngle;
  }

  return adjustedAngle;
}

float angleToLinearMM(float angleDeg) {
  float adjustedAngle = getAdjustedAngle(angleDeg);
  float mmPerDeg = LINEAR_TRAVEL_MM / ROTATION_DEG_FOR_TRAVEL;
  return adjustedAngle * mmPerDeg;
}

float angleToNormalized(float angleDeg) {
  float adjustedAngle = getAdjustedAngle(angleDeg);

  float normalized = adjustedAngle / ROTATION_DEG_FOR_TRAVEL;

  // 0~1 범위로 제한
  if (normalized < 0.0f) normalized = 0.0f;
  if (normalized > 1.0f) normalized = 1.0f;

  return normalized;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  Serial.println("AS5600 Normalized Position Start");
}

void loop() {
  uint16_t raw = readRawAngle();

  if (raw == 0xFFFF) {
    Serial.println("Read error");
  } else {
    float angleDeg    = rawToAngleDeg(raw);
    float positionMM  = angleToLinearMM(angleDeg);
    float normalized  = angleToNormalized(angleDeg);

    Serial.print("RAW: ");
    Serial.print(raw);

    Serial.print("  Angle: ");
    Serial.print(angleDeg, 2);
    Serial.print(" deg");

    Serial.print("  Position: ");
    Serial.print(positionMM, 2);
    Serial.print(" mm");

    Serial.print("  Normalized: ");
    Serial.println(normalized, 4);
  }

  delay(200);
}[오후 2:38]#include <Wire.h>

#define SDA_PIN 6
#define SCL_PIN 5
#define AS5600_ADDR 0x36

// ====== 사용자 조정 변수 ======
const float ROTATION_DEG_FOR_TRAVEL = 80.0;   // 몇 도 회전할 때
const float LINEAR_TRAVEL_MM        = 80.0;   // 몇 mm 이동하는지

const float ZERO_OFFSET_DEG = 266.75;            // 영점 보정값(필요시 수정)
const bool INVERT_DIRECTION = false;          // 방향 반전 필요하면 true
// ==============================

uint16_t readRawAngle() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0C);  // RAW ANGLE high byte
  if (Wire.endTransmission(false) != 0) {
    return 0xFFFF;
  }

  if (Wire.requestFrom(AS5600_ADDR, (uint8_t)2) != 2) {
    return 0xFFFF;
  }

  uint8_t highByte = Wire.read();
  uint8_t lowByte  = Wire.read();

  return ((highByte << 8) | lowByte) & 0x0FFF;
}

float rawToAngleDeg(uint16_t raw) {
  return raw * 360.0f / 4096.0f;
}

float getAdjustedAngle(float angleDeg) {
  float adjustedAngle = angleDeg - ZERO_OFFSET_DEG;

  if (INVERT_DIRECTION) {
    adjustedAngle = -adjustedAngle;
  }

  return adjustedAngle;
}

float angleToLinearMM(float angleDeg) {
  float adjustedAngle = getAdjustedAngle(angleDeg);
  float mmPerDeg = LINEAR_TRAVEL_MM / ROTATION_DEG_FOR_TRAVEL;
  return adjustedAngle * mmPerDeg;
}

float angleToNormalized(float angleDeg) {
  float adjustedAngle = getAdjustedAngle(angleDeg);

  float normalized = adjustedAngle / ROTATION_DEG_FOR_TRAVEL;

  // 0~1 범위로 제한
  if (normalized < 0.0f) normalized = 0.0f;
  if (normalized > 1.0f) normalized = 1.0f;

  return normalized;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  Serial.println("AS5600 Normalized Position Start");
}

void loop() {
  uint16_t raw = readRawAngle();

  if (raw == 0xFFFF) {
    Serial.println("Read error");
  } else {
    float angleDeg    = rawToAngleDeg(raw);
    float positionMM  = angleToLinearMM(angleDeg);
    float normalized  = angleToNormalized(angleDeg);

    Serial.print("RAW: ");
    Serial.print(raw);

    Serial.print("  Angle: ");
    Serial.print(angleDeg, 2);
    Serial.print(" deg");

    Serial.print("  Position: ");
    Serial.print(positionMM, 2);
    Serial.print(" mm");

    Serial.print("  Normalized: ");
    Serial.println(normalized, 4);
  }

  delay(200);
}