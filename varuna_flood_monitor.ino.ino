#include <Wire.h>

// ─── PIN DEFINITIONS ───
#define SDA_0  8
#define SCL_0  9
#define SDA_1  4
#define SCL_1  5
#define STATUS_LED 3

// ─── MPU6050 CONSTANTS ───
#define MPU6050_ADDR        0x68
#define REG_PWR_MGMT_1      0x6B
#define REG_SMPLRT_DIV       0x19
#define REG_CONFIG           0x1A
#define REG_GYRO_CONFIG      0x1B
#define REG_ACCEL_CONFIG     0x1C
#define REG_WHO_AM_I         0x75
#define REG_ACCEL_XOUT_H     0x3B

#define ACCEL_SCALE  16384.0
#define GYRO_SCALE   131.0

// ─── COMPLEMENTARY FILTER ───
#define ALPHA  0.98            // gyro weight (98% gyro, 2% accel)
#define DT_MAX 2.0             // if dt exceeds this, clamp to 0.01

// ─── I2C BUSES ───
TwoWire I2C_0 = TwoWire(0);
TwoWire I2C_1 = TwoWire(1);

// ─── MPU6050 RAW DATA ───
float ax, ay, az;              // accelerometer in g
float gx, gy, gz;              // gyroscope in °/s
float mpuTemp;                 // die temperature °C
bool  mpuHealthy = false;

// ─── CALIBRATION DATA ───
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;
float refAccX = 0, refAccY = 0, refAccZ = 0;
float refTiltX = 0, refTiltY = 0;
bool  calibrated = false;

#define GYRO_CAL_SAMPLES    1000
#define GYRO_CAL_DELAY_MS   2
#define ACCEL_CAL_SAMPLES   500
#define ACCEL_CAL_DELAY_MS  3
#define G_MIN_VALID         0.9
#define G_MAX_VALID         1.1

// ─── COMPLEMENTARY FILTER STATE ───
float filtTiltX = 0, filtTiltY = 0;    // fused tilt angles (°)
float correctedTiltX = 0, correctedTiltY = 0;  // after reference subtraction
float theta = 0;                        // combined angle from vertical (°)
unsigned long prevTime = 0;             // for dt calculation

// ──────────────────────────────────────────────
//  LOW-LEVEL I2C HELPERS
// ──────────────────────────────────────────────

void mpuWriteReg(uint8_t reg, uint8_t value) {
  I2C_0.beginTransmission(MPU6050_ADDR);
  I2C_0.write(reg);
  I2C_0.write(value);
  I2C_0.endTransmission();
}

uint8_t mpuReadReg(uint8_t reg) {
  I2C_0.beginTransmission(MPU6050_ADDR);
  I2C_0.write(reg);
  I2C_0.endTransmission(false);
  I2C_0.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)1);
  if (I2C_0.available()) {
    return I2C_0.read();
  }
  return 0xFF;
}

// ──────────────────────────────────────────────
//  INIT
// ──────────────────────────────────────────────

bool initMPU6050() {
  Serial.println("  Writing PWR_MGMT_1 = 0x00  (wake up)");
  mpuWriteReg(REG_PWR_MGMT_1, 0x00);
  delay(100);

  Serial.println("  Writing SMPLRT_DIV = 0x07  (125 Hz sample rate)");
  mpuWriteReg(REG_SMPLRT_DIV, 0x07);

  Serial.println("  Writing CONFIG     = 0x03  (DLPF 44 Hz)");
  mpuWriteReg(REG_CONFIG, 0x03);

  Serial.println("  Writing GYRO_CFG   = 0x00  (±250 °/s)");
  mpuWriteReg(REG_GYRO_CONFIG, 0x00);

  Serial.println("  Writing ACCEL_CFG  = 0x00  (±2 g)");
  mpuWriteReg(REG_ACCEL_CONFIG, 0x00);

  uint8_t whoami = mpuReadReg(REG_WHO_AM_I);
  Serial.printf("  WHO_AM_I register  = 0x%02X  ", whoami);

  if (whoami == 0x68 || whoami == 0x72) {
    Serial.println("✓ valid");
    mpuHealthy = true;
    return true;
  }

  Serial.println("✗ UNEXPECTED — check wiring");
  mpuHealthy = false;
  return false;
}

// ──────────────────────────────────────────────
//  RAW READ
// ──────────────────────────────────────────────

bool readMPU6050() {
  I2C_0.beginTransmission(MPU6050_ADDR);
  I2C_0.write(REG_ACCEL_XOUT_H);
  if (I2C_0.endTransmission(false) != 0) {
    return false;
  }

  uint8_t bytesReceived = I2C_0.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)14);
  if (bytesReceived < 14) {
    return false;
  }

  uint8_t buf[14];
  for (int i = 0; i < 14; i++) {
    buf[i] = I2C_0.read();
  }

  int16_t rawAx   = ((int16_t)buf[0]  << 8) | buf[1];
  int16_t rawAy   = ((int16_t)buf[2]  << 8) | buf[3];
  int16_t rawAz   = ((int16_t)buf[4]  << 8) | buf[5];
  int16_t rawTemp = ((int16_t)buf[6]  << 8) | buf[7];
  int16_t rawGx   = ((int16_t)buf[8]  << 8) | buf[9];
  int16_t rawGy   = ((int16_t)buf[10] << 8) | buf[11];
  int16_t rawGz   = ((int16_t)buf[12] << 8) | buf[13];

  ax = rawAx / ACCEL_SCALE;
  ay = rawAy / ACCEL_SCALE;
  az = rawAz / ACCEL_SCALE;

  gx = rawGx / GYRO_SCALE;
  gy = rawGy / GYRO_SCALE;
  gz = rawGz / GYRO_SCALE;

  mpuTemp = rawTemp / 340.0 + 36.53;

  return true;
}

// ──────────────────────────────────────────────
//  GYROSCOPE CALIBRATION
// ──────────────────────────────────────────────

bool calibrateGyro() {
  Serial.println("\n  ── GYRO CALIBRATION ──");
  Serial.println("  ⚠ Keep sensor COMPLETELY STILL");
  Serial.printf("  Collecting %d samples...\n", GYRO_CAL_SAMPLES);

  float sumGx = 0, sumGy = 0, sumGz = 0;
  int goodSamples = 0;

  for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
    if (!readMPU6050()) continue;
    sumGx += gx;
    sumGy += gy;
    sumGz += gz;
    goodSamples++;

    if ((i + 1) % 200 == 0) {
      Serial.printf("    %d / %d samples\n", i + 1, GYRO_CAL_SAMPLES);
    }
    delay(GYRO_CAL_DELAY_MS);
  }

  if (goodSamples < GYRO_CAL_SAMPLES / 2) {
    Serial.printf("  ✗ Too many failures: only %d good samples\n", goodSamples);
    return false;
  }

  gyroOffsetX = sumGx / goodSamples;
  gyroOffsetY = sumGy / goodSamples;
  gyroOffsetZ = sumGz / goodSamples;

  Serial.println("\n  Gyro offsets (°/s):");
  Serial.printf("    X: %+.4f\n", gyroOffsetX);
  Serial.printf("    Y: %+.4f\n", gyroOffsetY);
  Serial.printf("    Z: %+.4f\n", gyroOffsetZ);
  Serial.printf("    Good samples: %d / %d\n", goodSamples, GYRO_CAL_SAMPLES);

  return true;
}

// ──────────────────────────────────────────────
//  ACCELEROMETER CALIBRATION
// ──────────────────────────────────────────────

bool calibrateAccel() {
  Serial.println("\n  ── ACCEL CALIBRATION ──");
  Serial.println("  ⚠ Keep sensor STILL and LEVEL");
  Serial.printf("  Collecting %d samples...\n", ACCEL_CAL_SAMPLES);

  float sumAx = 0, sumAy = 0, sumAz = 0;
  int goodSamples = 0;
  int rejected = 0;

  for (int i = 0; i < ACCEL_CAL_SAMPLES; i++) {
    if (!readMPU6050()) { rejected++; continue; }

    float totalG = sqrt(ax * ax + ay * ay + az * az);
    if (totalG < G_MIN_VALID || totalG > G_MAX_VALID) { rejected++; continue; }

    sumAx += ax;
    sumAy += ay;
    sumAz += az;
    goodSamples++;

    if ((i + 1) % 100 == 0) {
      Serial.printf("    %d / %d  (accepted: %d, rejected: %d)\n",
                    i + 1, ACCEL_CAL_SAMPLES, goodSamples, rejected);
    }
    delay(ACCEL_CAL_DELAY_MS);
  }

  if (goodSamples < ACCEL_CAL_SAMPLES / 2) {
    Serial.printf("  ✗ Too many rejections: only %d good\n", goodSamples);
    return false;
  }

  refAccX = sumAx / goodSamples;
  refAccY = sumAy / goodSamples;
  refAccZ = sumAz / goodSamples;

  refTiltX = atan2(refAccY, sqrt(refAccX * refAccX + refAccZ * refAccZ)) * 180.0 / PI;
  refTiltY = atan2(-refAccX, sqrt(refAccY * refAccY + refAccZ * refAccZ)) * 180.0 / PI;

  float refG = sqrt(refAccX * refAccX + refAccY * refAccY + refAccZ * refAccZ);

  Serial.println("\n  Reference accel (g):");
  Serial.printf("    X: %+.5f   Y: %+.5f   Z: %+.5f   |G|: %.5f\n",
                refAccX, refAccY, refAccZ, refG);
  Serial.println("  Reference tilt (°):");
  Serial.printf("    refTiltX: %+.3f°   refTiltY: %+.3f°\n", refTiltX, refTiltY);

  return true;
}

// ──────────────────────────────────────────────
//  FULL CALIBRATION SEQUENCE
// ──────────────────────────────────────────────

bool recalibrate() {
  Serial.println("\n┌─────────────────────────────────────────┐");
  Serial.println("│         MPU6050 CALIBRATION              │");
  Serial.println("└─────────────────────────────────────────┘");

  Serial.println("  LED blink: 3 seconds — place sensor flat and still...");
  pinMode(STATUS_LED, OUTPUT);
  for (int i = 0; i < 6; i++) {
    digitalWrite(STATUS_LED, HIGH); delay(250);
    digitalWrite(STATUS_LED, LOW);  delay(250);
  }

  bool gyroOk  = calibrateGyro();
  bool accelOk = calibrateAccel();

  if (gyroOk && accelOk) {
    calibrated = true;

    // seed the filter with the current accel angle so it
    // doesn't start at 0 and ramp toward the real value
    filtTiltX = refTiltX;
    filtTiltY = refTiltY;

    Serial.println("\n  ✓ CALIBRATION COMPLETE");
    return true;
  }

  calibrated = false;
  Serial.println("\n  ✗ CALIBRATION FAILED");
  return false;
}

// ──────────────────────────────────────────────
//  COMPLEMENTARY FILTER — ONE ITERATION
//
//  Called every loop. Fuses gyro (short-term
//  accurate, drifts) with accel (noisy but
//  drift-free) into stable tilt angles.
//
//  filtTilt = 0.98 × (filtTilt + gyroRate×dt)
//           + 0.02 × accelAngle
//
//  Then subtract reference mounting offset.
//  Then combine X and Y into single theta.
// ──────────────────────────────────────────────

void updateComplementaryFilter() {
  // ── dt calculation ──
  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0;    // seconds

  // first call or long gap (sleep recovery): use safe small dt
  if (prevTime == 0 || dt > DT_MAX) {
    dt = 0.01;
  }
  prevTime = now;

  // ── accel-derived tilt angles (drift-free, noisy) ──
  float accelTiltX = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
  float accelTiltY = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

  // ── gyro angular rates (offset-corrected) ──
  float gyroRateX = gx - gyroOffsetX;     // °/s
  float gyroRateY = gy - gyroOffsetY;

  // ── fuse ──
  filtTiltX = ALPHA * (filtTiltX + gyroRateX * dt) + (1.0 - ALPHA) * accelTiltX;
  filtTiltY = ALPHA * (filtTiltY + gyroRateY * dt) + (1.0 - ALPHA) * accelTiltY;

  // ── subtract reference (mounting offset) ──
  correctedTiltX = filtTiltX - refTiltX;
  correctedTiltY = filtTiltY - refTiltY;

  // ── combined angle from vertical ──
  theta = sqrt(correctedTiltX * correctedTiltX + correctedTiltY * correctedTiltY);
}

// ──────────────────────────────────────────────
//  SETUP
// ──────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("┌──────────────────────────────────────────────┐");
  Serial.println("│   VARUNA — Step 1.4: Complementary Filter    │");
  Serial.println("└──────────────────────────────────────────────┘");

  I2C_0.begin(SDA_0, SCL_0, 100000);
  I2C_1.begin(SDA_1, SCL_1, 100000);
  Serial.println("\nI2C buses initialised");

  Serial.println("\n── MPU6050 INIT ──");
  if (!initMPU6050()) {
    Serial.println("── MPU6050 FAILED — halting ──");
    while (1) delay(1000);
  }

  recalibrate();

  // initialise filter timestamp
  prevTime = millis();

  Serial.println("\n── COMPLEMENTARY FILTER ACTIVE ──");
  Serial.println("Output at 10 Hz.  θ = combined angle from vertical.\n");
  Serial.println("VERIFY:");
  Serial.println("  Flat on table        → θ ≈ 0°");
  Serial.println("  Tilt ~30°            → θ ≈ 30°");
  Serial.println("  Tilt ~45°            → θ ≈ 45°");
  Serial.println("  Tilt ~90°            → θ ≈ 90°");
  Serial.println("  Shake while flat     → θ stays near 0° (stable)");
  Serial.println("  Hold at angle 60s    → θ stays steady (no drift)");
  Serial.println();
}

// ──────────────────────────────────────────────
//  LOOP
// ──────────────────────────────────────────────

unsigned long lastPrint = 0;
unsigned long loopCount = 0;

// drift monitoring: record theta at 10s mark, compare at 70s
float   driftStartTheta = -1;
unsigned long driftStartTime = 0;
bool    driftTestDone = false;

void loop() {
  if (!mpuHealthy || !calibrated) {
    delay(1000);
    return;
  }

  // ── read sensors ──
  if (!readMPU6050()) {
    delay(10);
    return;
  }

  // ── run filter ──
  updateComplementaryFilter();
  loopCount++;

  // ── print at 10 Hz ──
  if (millis() - lastPrint >= 100) {
    lastPrint = millis();

    float totalG = sqrt(ax * ax + ay * ay + az * az);

    // single clean line with the most important value prominent
    Serial.printf("θ = %6.2f°   |  tiltX %+6.2f°  tiltY %+6.2f°  |  "
                  "|G| %.3f  |  gyro %+6.2f %+6.2f %+6.2f  |  loops %lu\n",
                  theta,
                  correctedTiltX, correctedTiltY,
                  totalG,
                  gx - gyroOffsetX, gy - gyroOffsetY, gz - gyroOffsetZ,
                  loopCount);

    // ── automatic drift check ──
    // at 10 seconds after boot, record theta
    // at 70 seconds, compare — drift should be < 1°
    unsigned long uptime = millis() / 1000;

    if (!driftTestDone && uptime >= 10 && driftStartTheta < 0) {
      driftStartTheta = theta;
      driftStartTime = millis();
      Serial.println("  ──── DRIFT TEST: baseline recorded ────");
    }

    if (!driftTestDone && driftStartTheta >= 0 && (millis() - driftStartTime) >= 60000) {
      float drift = theta - driftStartTheta;
      Serial.println();
      Serial.println("  ══════════════════════════════════════════════════");
      Serial.printf( "  DRIFT TEST (60 seconds, sensor STILL):\n");
      Serial.printf( "    Start: %.2f°   End: %.2f°   Drift: %+.3f°\n",
                     driftStartTheta, theta, drift);
      if (fabs(drift) < 1.0) {
        Serial.println("    ✓ PASS — drift < 1°");
      } else if (fabs(drift) < 2.0) {
        Serial.println("    ~ MARGINAL — drift 1-2°");
      } else {
        Serial.println("    ✗ FAIL — drift > 2° — check sensor/filter");
      }
      Serial.println("  ══════════════════════════════════════════════════\n");
      driftTestDone = true;
    }
  }

  delay(8);   // ~125 Hz loop rate
}