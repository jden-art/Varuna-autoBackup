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
#define ALPHA  0.98
#define DT_MAX 2.0

// ─── BMP280 CONSTANTS ───
#define BMP280_ADDR         0x76
#define BMP280_REG_CHIP_ID  0xD0
#define BMP280_REG_CALIB     0x88   // 26 bytes of calibration data
#define BMP280_REG_CTRL_MEAS 0xF4
#define BMP280_REG_CONFIG_R  0xF5
#define BMP280_REG_PRESS_MSB 0xF7   // 6 bytes: press[3] + temp[3]

// ─── I2C BUSES ───
TwoWire I2C_0 = TwoWire(0);
TwoWire I2C_1 = TwoWire(1);

// ─── MPU6050 DATA ───
float ax, ay, az;
float gx, gy, gz;
float mpuTemp;
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
float filtTiltX = 0, filtTiltY = 0;
float correctedTiltX = 0, correctedTiltY = 0;
float theta = 0;
unsigned long prevTime = 0;
bool filterSeeded = false;

// ─── BMP280 DATA ───
bool bmpAvailable = false;
float currentPressure = 0;     // hPa
float currentTemperature = 0;  // °C

// BMP280 calibration coefficients (from sensor registers)
uint16_t bmpDigT1;
int16_t  bmpDigT2, bmpDigT3;
uint16_t bmpDigP1;
int16_t  bmpDigP2, bmpDigP3, bmpDigP4, bmpDigP5;
int16_t  bmpDigP6, bmpDigP7, bmpDigP8, bmpDigP9;
int32_t  bmpTFine;  // shared between temp and pressure compensation


// ══════════════════════════════════════════════
//  MPU6050 SECTION
// ══════════════════════════════════════════════

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
  if (I2C_0.available()) return I2C_0.read();
  return 0xFF;
}

bool initMPU6050() {
  Serial.println("  Writing PWR_MGMT_1 = 0x00  (wake up)");
  mpuWriteReg(REG_PWR_MGMT_1, 0x00);
  delay(100);

  Serial.println("  Writing SMPLRT_DIV = 0x07  (125 Hz)");
  mpuWriteReg(REG_SMPLRT_DIV, 0x07);

  Serial.println("  Writing CONFIG     = 0x03  (DLPF 44 Hz)");
  mpuWriteReg(REG_CONFIG, 0x03);

  Serial.println("  Writing GYRO_CFG   = 0x00  (±250 °/s)");
  mpuWriteReg(REG_GYRO_CONFIG, 0x00);

  Serial.println("  Writing ACCEL_CFG  = 0x00  (±2 g)");
  mpuWriteReg(REG_ACCEL_CONFIG, 0x00);

  uint8_t whoami = mpuReadReg(REG_WHO_AM_I);
  Serial.printf("  WHO_AM_I = 0x%02X  ", whoami);

  if (whoami == 0x68 || whoami == 0x72) {
    Serial.println("✓");
    mpuHealthy = true;
    return true;
  }
  Serial.println("✗");
  mpuHealthy = false;
  return false;
}

bool readMPU6050() {
  I2C_0.beginTransmission(MPU6050_ADDR);
  I2C_0.write(REG_ACCEL_XOUT_H);
  if (I2C_0.endTransmission(false) != 0) return false;

  if (I2C_0.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)14) < 14) return false;

  uint8_t buf[14];
  for (int i = 0; i < 14; i++) buf[i] = I2C_0.read();

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
//  CALIBRATION
// ──────────────────────────────────────────────

bool calibrateGyro() {
  Serial.println("\n  ── GYRO CALIBRATION ──");
  Serial.println("  ⚠ Keep sensor COMPLETELY STILL");

  float sumGx = 0, sumGy = 0, sumGz = 0;
  int good = 0;

  for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
    if (!readMPU6050()) continue;
    sumGx += gx; sumGy += gy; sumGz += gz;
    good++;
    if ((i + 1) % 250 == 0) Serial.printf("    %d / %d\n", i + 1, GYRO_CAL_SAMPLES);
    delay(GYRO_CAL_DELAY_MS);
  }

  if (good < GYRO_CAL_SAMPLES / 2) return false;

  gyroOffsetX = sumGx / good;
  gyroOffsetY = sumGy / good;
  gyroOffsetZ = sumGz / good;

  Serial.printf("  Offsets (°/s): X %+.4f  Y %+.4f  Z %+.4f  (%d samples)\n",
                gyroOffsetX, gyroOffsetY, gyroOffsetZ, good);
  return true;
}

bool calibrateAccel() {
  Serial.println("\n  ── ACCEL CALIBRATION ──");
  Serial.println("  ⚠ Keep sensor STILL and LEVEL");

  float sumAx = 0, sumAy = 0, sumAz = 0;
  int good = 0, rejected = 0;

  for (int i = 0; i < ACCEL_CAL_SAMPLES; i++) {
    if (!readMPU6050()) { rejected++; continue; }
    float totalG = sqrt(ax * ax + ay * ay + az * az);
    if (totalG < G_MIN_VALID || totalG > G_MAX_VALID) { rejected++; continue; }
    sumAx += ax; sumAy += ay; sumAz += az;
    good++;
    if ((i + 1) % 100 == 0)
      Serial.printf("    %d / %d  (good: %d, rejected: %d)\n",
                    i + 1, ACCEL_CAL_SAMPLES, good, rejected);
    delay(ACCEL_CAL_DELAY_MS);
  }

  if (good < ACCEL_CAL_SAMPLES / 2) return false;

  refAccX = sumAx / good;
  refAccY = sumAy / good;
  refAccZ = sumAz / good;

  refTiltX = atan2(refAccY, sqrt(refAccX * refAccX + refAccZ * refAccZ)) * 180.0 / PI;
  refTiltY = atan2(-refAccX, sqrt(refAccY * refAccY + refAccZ * refAccZ)) * 180.0 / PI;

  Serial.printf("  Ref accel (g): X %+.5f  Y %+.5f  Z %+.5f\n", refAccX, refAccY, refAccZ);
  Serial.printf("  Ref tilt  (°): X %+.3f  Y %+.3f\n", refTiltX, refTiltY);
  return true;
}

bool recalibrate() {
  Serial.println("\n┌─────────────────────────────────────────┐");
  Serial.println("│         MPU6050 CALIBRATION              │");
  Serial.println("└─────────────────────────────────────────┘");

  pinMode(STATUS_LED, OUTPUT);
  for (int i = 0; i < 6; i++) {
    digitalWrite(STATUS_LED, HIGH); delay(250);
    digitalWrite(STATUS_LED, LOW);  delay(250);
  }

  if (calibrateGyro() && calibrateAccel()) {
    calibrated = true;
    filterSeeded = false;  // force filter to re-seed from first live reading
    Serial.println("\n  ✓ CALIBRATION COMPLETE");
    return true;
  }

  calibrated = false;
  Serial.println("\n  ✗ CALIBRATION FAILED");
  return false;
}

// ──────────────────────────────────────────────
//  COMPLEMENTARY FILTER  (FIXED)
//
//  Key fix: the filter works entirely in the
//  CORRECTED domain. Both the gyro integration
//  and the accel angle are referenced to the
//  calibrated zero, so they agree.
//
//  filtTiltX tracks corrected tilt, not raw tilt.
//  This prevents the 98% gyro path and the 2%
//  accel path from fighting each other.
// ──────────────────────────────────────────────

void updateComplementaryFilter() {
  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0;

  if (prevTime == 0 || dt > DT_MAX) {
    dt = 0.01;
  }
  prevTime = now;

  // accel-derived tilt (raw)
  float accelTiltX = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
  float accelTiltY = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

  // subtract reference NOW — so accel input is in corrected frame
  float accelCorrX = accelTiltX - refTiltX;
  float accelCorrY = accelTiltY - refTiltY;

  // gyro rates (offset-corrected)
  float gyroRateX = gx - gyroOffsetX;
  float gyroRateY = gy - gyroOffsetY;

  // seed filter on first call with accel angle (prevents ramp-up)
  if (!filterSeeded) {
    filtTiltX = accelCorrX;
    filtTiltY = accelCorrY;
    filterSeeded = true;
  }

  // fuse — both sides now in corrected frame
  filtTiltX = ALPHA * (filtTiltX + gyroRateX * dt) + (1.0 - ALPHA) * accelCorrX;
  filtTiltY = ALPHA * (filtTiltY + gyroRateY * dt) + (1.0 - ALPHA) * accelCorrY;

  // filtTiltX/Y ARE the corrected values — no further subtraction needed
  correctedTiltX = filtTiltX;
  correctedTiltY = filtTiltY;

  // combined angle from vertical
  theta = sqrt(correctedTiltX * correctedTiltX + correctedTiltY * correctedTiltY);
}


// ══════════════════════════════════════════════
//  BMP280 SECTION
// ══════════════════════════════════════════════

void bmpWriteReg(uint8_t reg, uint8_t value) {
  I2C_1.beginTransmission(BMP280_ADDR);
  I2C_1.write(reg);
  I2C_1.write(value);
  I2C_1.endTransmission();
}

uint8_t bmpReadReg(uint8_t reg) {
  I2C_1.beginTransmission(BMP280_ADDR);
  I2C_1.write(reg);
  I2C_1.endTransmission(false);
  I2C_1.requestFrom((uint8_t)BMP280_ADDR, (uint8_t)1);
  if (I2C_1.available()) return I2C_1.read();
  return 0xFF;
}

// read N bytes starting from reg into buffer
void bmpReadBytes(uint8_t reg, uint8_t *buf, uint8_t len) {
  I2C_1.beginTransmission(BMP280_ADDR);
  I2C_1.write(reg);
  I2C_1.endTransmission(false);
  I2C_1.requestFrom((uint8_t)BMP280_ADDR, len);
  for (uint8_t i = 0; i < len && I2C_1.available(); i++) {
    buf[i] = I2C_1.read();
  }
}

bool initBMP280() {
  Serial.println("\n── BMP280 INIT ──");

  // check chip ID
  uint8_t chipId = bmpReadReg(BMP280_REG_CHIP_ID);
  Serial.printf("  Chip ID = 0x%02X  ", chipId);

  if (chipId != 0x58 && chipId != 0x56 && chipId != 0x57 && chipId != 0x60) {
    Serial.println("✗ not a BMP280");
    bmpAvailable = false;
    return false;
  }
  Serial.println("✓ BMP280 detected");

  // read 26 bytes of calibration data starting at 0x88
  uint8_t calib[26];
  bmpReadBytes(BMP280_REG_CALIB, calib, 26);

  // parse calibration coefficients (little-endian in registers)
  bmpDigT1 = (uint16_t)(calib[1] << 8) | calib[0];
  bmpDigT2 = (int16_t)((calib[3] << 8) | calib[2]);
  bmpDigT3 = (int16_t)((calib[5] << 8) | calib[4]);

  bmpDigP1 = (uint16_t)(calib[7] << 8) | calib[6];
  bmpDigP2 = (int16_t)((calib[9]  << 8) | calib[8]);
  bmpDigP3 = (int16_t)((calib[11] << 8) | calib[10]);
  bmpDigP4 = (int16_t)((calib[13] << 8) | calib[12]);
  bmpDigP5 = (int16_t)((calib[15] << 8) | calib[14]);
  bmpDigP6 = (int16_t)((calib[17] << 8) | calib[16]);
  bmpDigP7 = (int16_t)((calib[19] << 8) | calib[18]);
  bmpDigP8 = (int16_t)((calib[21] << 8) | calib[20]);
  bmpDigP9 = (int16_t)((calib[23] << 8) | calib[22]);

  Serial.println("  Calibration coefficients loaded:");
  Serial.printf("    T1=%u  T2=%d  T3=%d\n", bmpDigT1, bmpDigT2, bmpDigT3);
  Serial.printf("    P1=%u  P2=%d  P3=%d  P4=%d  P5=%d\n",
                bmpDigP1, bmpDigP2, bmpDigP3, bmpDigP4, bmpDigP5);
  Serial.printf("    P6=%d  P7=%d  P8=%d  P9=%d\n",
                bmpDigP6, bmpDigP7, bmpDigP8, bmpDigP9);

  // configure:
  //   reg 0xF5 (config):    standby 500ms, filter coeff 4, SPI off
  //                         bits: 100 010 0 0 = 0x88... 
  //                         per roadmap: 0x10 = standby 0.5ms, filter off
  //   reg 0xF4 (ctrl_meas): temp oversampling x4, press oversampling x16, normal mode
  //                         bits: 010 101 11 = 0x57
  //
  // IMPORTANT: write config (0xF5) BEFORE ctrl_meas (0xF4)
  // because writing ctrl_meas can trigger a measurement

  Serial.println("  Writing CONFIG    = 0x10  (standby 0.5ms, filter off)");
  bmpWriteReg(BMP280_REG_CONFIG_R, 0x10);

  Serial.println("  Writing CTRL_MEAS = 0x57  (T×4, P×16, normal mode)");
  bmpWriteReg(BMP280_REG_CTRL_MEAS, 0x57);

  delay(100);  // allow first measurement cycle

  // verify registers
  uint8_t vCtrl = bmpReadReg(BMP280_REG_CTRL_MEAS);
  uint8_t vConf = bmpReadReg(BMP280_REG_CONFIG_R);
  Serial.printf("  Readback: CTRL_MEAS=0x%02X %s  CONFIG=0x%02X %s\n",
                vCtrl, (vCtrl == 0x57) ? "✓" : "✗",
                vConf, (vConf == 0x10) ? "✓" : "✗");

  bmpAvailable = true;
  Serial.println("  ✓ BMP280 READY\n");
  return true;
}

// ──────────────────────────────────────────────
//  BMP280 COMPENSATION (Bosch datasheet formulas)
//  Integer arithmetic — exact copy from datasheet
// ──────────────────────────────────────────────

// Temperature compensation — returns °C × 100 as int32
// Also sets bmpTFine for pressure compensation
int32_t bmpCompensateTemp(int32_t adcT) {
  int32_t var1 = ((((adcT >> 3) - ((int32_t)bmpDigT1 << 1))) * ((int32_t)bmpDigT2)) >> 11;
  int32_t var2 = (((((adcT >> 4) - ((int32_t)bmpDigT1)) *
                    ((adcT >> 4) - ((int32_t)bmpDigT1))) >> 12) *
                  ((int32_t)bmpDigT3)) >> 14;
  bmpTFine = var1 + var2;
  return (bmpTFine * 5 + 128) >> 8;  // °C × 100
}

// Pressure compensation — returns Pa as uint32
// Must call bmpCompensateTemp first to set bmpTFine
uint32_t bmpCompensatePress(int32_t adcP) {
  int64_t var1 = ((int64_t)bmpTFine) - 128000;
  int64_t var2 = var1 * var1 * (int64_t)bmpDigP6;
  var2 = var2 + ((var1 * (int64_t)bmpDigP5) << 17);
  var2 = var2 + (((int64_t)bmpDigP4) << 35);
  var1 = ((var1 * var1 * (int64_t)bmpDigP3) >> 8) +
         ((var1 * (int64_t)bmpDigP2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bmpDigP1) >> 33;

  if (var1 == 0) return 0;  // avoid division by zero

  int64_t p = 1048576 - adcP;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)bmpDigP9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)bmpDigP8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)bmpDigP7) << 4);

  return (uint32_t)p;  // Pa × 256
}

// ──────────────────────────────────────────────
//  BMP280 READ — temperature and pressure
// ──────────────────────────────────────────────

bool bmpReadData(float *temperature, float *pressure) {
  // read 6 bytes: press_msb, press_lsb, press_xlsb, temp_msb, temp_lsb, temp_xlsb
  uint8_t buf[6];
  bmpReadBytes(BMP280_REG_PRESS_MSB, buf, 6);

  int32_t adcP = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | ((int32_t)buf[2] >> 4);
  int32_t adcT = ((int32_t)buf[3] << 12) | ((int32_t)buf[4] << 4) | ((int32_t)buf[5] >> 4);

  // check for invalid readings (all 1s or all 0s)
  if (adcT == 0 || adcT == 0xFFFFF || adcP == 0 || adcP == 0xFFFFF) {
    Serial.println("  ✗ BMP280 raw data invalid");
    return false;
  }

  // compensate temperature first (sets bmpTFine)
  int32_t tempRaw = bmpCompensateTemp(adcT);   // °C × 100
  *temperature = tempRaw / 100.0;

  // compensate pressure (uses bmpTFine)
  uint32_t pressRaw = bmpCompensatePress(adcP); // Pa × 256
  *pressure = pressRaw / 256.0 / 100.0;         // convert to hPa

  return true;
}


// ══════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("┌────────────────────────────────────────────────────────┐");
  Serial.println("│   VARUNA — Step 1.4 + 1.5: Comp Filter + BMP280      │");
  Serial.println("└────────────────────────────────────────────────────────┘");

  I2C_0.begin(SDA_0, SCL_0, 100000);
  I2C_1.begin(SDA_1, SCL_1, 100000);
  Serial.println("\nI2C buses initialised");

  // ── MPU6050 ──
  Serial.println("\n── MPU6050 INIT ──");
  if (!initMPU6050()) {
    Serial.println("── MPU6050 FAILED — halting ──");
    while (1) delay(1000);
  }

  recalibrate();

  // ── BMP280 ──
  initBMP280();

  // ── start filter ──
  prevTime = millis();

  Serial.println("\n══════════════════════════════════════════════════════════");
  Serial.println("  LIVE OUTPUT — 10 Hz");
  Serial.println("  θ = combined tilt angle from vertical");
  Serial.println();
  Serial.println("  TEST PLAN:");
  Serial.println("    1. Leave flat 70 seconds → drift test auto-runs");
  Serial.println("    2. Tilt to 30°, 45°, 90° → verify θ");
  Serial.println("    3. Shake while flat → θ should stay near 0°");
  Serial.println("    4. BMP280 reads every 2 seconds below θ line");
  Serial.println("══════════════════════════════════════════════════════════\n");
}

// ══════════════════════════════════════════════
//  LOOP
// ══════════════════════════════════════════════

unsigned long lastPrint = 0;
unsigned long lastBmpRead = 0;
unsigned long loopCount = 0;

// drift test
float   driftStartTheta = -1;
unsigned long driftStartTime = 0;
bool    driftTestDone = false;

void loop() {
  if (!mpuHealthy || !calibrated) { delay(1000); return; }

  if (!readMPU6050()) { delay(10); return; }

  updateComplementaryFilter();
  loopCount++;

  // ── BMP280 read every 2 seconds ──
  if (bmpAvailable && (millis() - lastBmpRead >= 2000)) {
    lastBmpRead = millis();
    bmpReadData(&currentTemperature, &currentPressure);
  }

  // ── print at 10 Hz ──
  if (millis() - lastPrint >= 100) {
    lastPrint = millis();

    float totalG = sqrt(ax * ax + ay * ay + az * az);

    Serial.printf("θ = %6.2f°  |  X %+6.2f°  Y %+6.2f°  |  "
                  "|G| %.3f  |  gyro %+5.2f %+5.2f %+5.2f",
                  theta,
                  correctedTiltX, correctedTiltY,
                  totalG,
                  gx - gyroOffsetX, gy - gyroOffsetY, gz - gyroOffsetZ);

    // append BMP data on same line if available
    if (bmpAvailable && currentPressure > 0) {
      Serial.printf("  |  %.2f hPa  %.1f°C", currentPressure, currentTemperature);
    }

    Serial.println();

    // ── drift test ──
    unsigned long uptime = millis() / 1000;

    if (!driftTestDone && uptime >= 10 && driftStartTheta < 0) {
      driftStartTheta = theta;
      driftStartTime = millis();
      Serial.println("  ──── DRIFT TEST: baseline recorded (keep sensor STILL for 60s) ────");
    }

    if (!driftTestDone && driftStartTheta >= 0 && (millis() - driftStartTime) >= 60000) {
      float drift = theta - driftStartTheta;
      Serial.println();
      Serial.println("  ══════════════════════════════════════════════════");
      Serial.printf( "  DRIFT TEST (60 seconds, sensor STILL):\n");
      Serial.printf( "    Start: %.2f°   End: %.2f°   Drift: %+.3f°\n",
                     driftStartTheta, theta, drift);
      if (fabs(drift) < 1.0)
        Serial.println("    ✓ PASS — drift < 1°");
      else if (fabs(drift) < 2.0)
        Serial.println("    ~ MARGINAL — drift 1-2°");
      else
        Serial.println("    ✗ FAIL — drift > 2°");
      Serial.println("  ══════════════════════════════════════════════════\n");
      driftTestDone = true;
    }
  }

  delay(8);
}