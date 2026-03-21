#include <Wire.h>

// ─── PIN DEFINITIONS ───
#define SDA_0  8
#define SCL_0  9
#define SDA_1  4
#define SCL_1  5

// ─── MPU6050 CONSTANTS ───
#define MPU6050_ADDR        0x68
#define REG_PWR_MGMT_1      0x6B
#define REG_SMPLRT_DIV       0x19
#define REG_CONFIG           0x1A
#define REG_GYRO_CONFIG      0x1B
#define REG_ACCEL_CONFIG     0x1C
#define REG_WHO_AM_I         0x75
#define REG_ACCEL_XOUT_H     0x3B

// Sensitivity scale factors (from datasheet)
// ±2g  → 16384 LSB/g
// ±250°/s → 131 LSB/(°/s)
#define ACCEL_SCALE  16384.0
#define GYRO_SCALE   131.0

// ─── I2C BUSES ───
TwoWire I2C_0 = TwoWire(0);
TwoWire I2C_1 = TwoWire(1);

// ─── MPU6050 LIVE DATA ───
float ax, ay, az;       // accelerometer in g
float gx, gy, gz;       // gyroscope in °/s
float mpuTemp;           // die temperature °C
bool  mpuHealthy = false;

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
  I2C_0.endTransmission(false);          // repeated start
  I2C_0.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)1);
  if (I2C_0.available()) {
    return I2C_0.read();
  }
  return 0xFF;                            // read failed
}

// ──────────────────────────────────────────────
//  INIT
// ──────────────────────────────────────────────

bool initMPU6050() {
  Serial.println("  Writing PWR_MGMT_1 = 0x00  (wake up)");
  mpuWriteReg(REG_PWR_MGMT_1, 0x00);
  delay(100);                             // datasheet: wait after wake

  Serial.println("  Writing SMPLRT_DIV = 0x07  (125 Hz sample rate)");
  mpuWriteReg(REG_SMPLRT_DIV, 0x07);     // 1 kHz / (1+7) = 125 Hz

  Serial.println("  Writing CONFIG     = 0x03  (DLPF 44 Hz)");
  mpuWriteReg(REG_CONFIG, 0x03);

  Serial.println("  Writing GYRO_CFG   = 0x00  (±250 °/s)");
  mpuWriteReg(REG_GYRO_CONFIG, 0x00);

  Serial.println("  Writing ACCEL_CFG  = 0x00  (±2 g)");
  mpuWriteReg(REG_ACCEL_CONFIG, 0x00);

  // ── verify identity ──
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
//  READ 14 BYTES — ACCEL + TEMP + GYRO
// ──────────────────────────────────────────────

bool readMPU6050() {
  I2C_0.beginTransmission(MPU6050_ADDR);
  I2C_0.write(REG_ACCEL_XOUT_H);
  if (I2C_0.endTransmission(false) != 0) {
    Serial.println("  ✗ I2C transmit error");
    return false;
  }

  uint8_t bytesReceived = I2C_0.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)14);
  if (bytesReceived < 14) {
    Serial.printf("  ✗ Expected 14 bytes, got %d\n", bytesReceived);
    return false;
  }

  // read into buffer first — avoids undefined evaluation order
  uint8_t buf[14];
  for (int i = 0; i < 14; i++) {
    buf[i] = I2C_0.read();
  }

  // registers are big-endian 16-bit signed
  int16_t rawAx   = ((int16_t)buf[0]  << 8) | buf[1];
  int16_t rawAy   = ((int16_t)buf[2]  << 8) | buf[3];
  int16_t rawAz   = ((int16_t)buf[4]  << 8) | buf[5];
  int16_t rawTemp = ((int16_t)buf[6]  << 8) | buf[7];
  int16_t rawGx   = ((int16_t)buf[8]  << 8) | buf[9];
  int16_t rawGy   = ((int16_t)buf[10] << 8) | buf[11];
  int16_t rawGz   = ((int16_t)buf[12] << 8) | buf[13];

  // convert to physical units
  ax = rawAx / ACCEL_SCALE;              // g
  ay = rawAy / ACCEL_SCALE;
  az = rawAz / ACCEL_SCALE;

  gx = rawGx / GYRO_SCALE;              // °/s
  gy = rawGy / GYRO_SCALE;
  gz = rawGz / GYRO_SCALE;

  mpuTemp = rawTemp / 340.0 + 36.53;    // °C  (datasheet formula)

  return true;
}

// ──────────────────────────────────────────────
//  READ-BACK VERIFICATION
//  confirm registers hold what we wrote
// ──────────────────────────────────────────────

void verifyRegisters() {
  Serial.println("\n  Register verification:");

  uint8_t v;
  v = mpuReadReg(REG_PWR_MGMT_1);
  Serial.printf("    PWR_MGMT_1  = 0x%02X  %s\n", v, (v == 0x00) ? "✓" : "✗");

  v = mpuReadReg(REG_SMPLRT_DIV);
  Serial.printf("    SMPLRT_DIV  = 0x%02X  %s\n", v, (v == 0x07) ? "✓" : "✗");

  v = mpuReadReg(REG_CONFIG);
  Serial.printf("    CONFIG      = 0x%02X  %s\n", v, (v == 0x03) ? "✓" : "✗");

  v = mpuReadReg(REG_GYRO_CONFIG);
  Serial.printf("    GYRO_CFG    = 0x%02X  %s\n", v, (v == 0x00) ? "✓" : "✗");

  v = mpuReadReg(REG_ACCEL_CONFIG);
  Serial.printf("    ACCEL_CFG   = 0x%02X  %s\n", v, (v == 0x00) ? "✓" : "✗");
}

// ──────────────────────────────────────────────
//  SETUP
// ──────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("┌─────────────────────────────────────────┐");
  Serial.println("│     VARUNA — Step 1.2: MPU6050 Driver   │");
  Serial.println("└─────────────────────────────────────────┘");

  I2C_0.begin(SDA_0, SCL_0, 100000);
  I2C_1.begin(SDA_1, SCL_1, 100000);
  Serial.println("\nI2C buses initialised");

  Serial.println("\n── MPU6050 INIT ──");
  if (initMPU6050()) {
    verifyRegisters();
    Serial.println("\n── MPU6050 READY ──\n");
  } else {
    Serial.println("\n── MPU6050 FAILED — halting ──\n");
  }
}

// ──────────────────────────────────────────────
//  LOOP — continuous read at 5 Hz
// ──────────────────────────────────────────────

unsigned long lastPrint = 0;
unsigned long readCount = 0;

void loop() {
  if (!mpuHealthy) {
    delay(1000);
    return;
  }

  if (!readMPU6050()) {
    delay(500);
    return;
  }

  readCount++;

  // print at 5 Hz (every 200 ms) for readability
  if (millis() - lastPrint >= 200) {
    lastPrint = millis();

    float totalG = sqrt(ax * ax + ay * ay + az * az);

    Serial.println("────────────────────────────────────────────────");
    Serial.printf("  Accel (g)  :  X %+7.3f   Y %+7.3f   Z %+7.3f   |G| %.3f\n",
                  ax, ay, az, totalG);
    Serial.printf("  Gyro (°/s) :  X %+7.2f   Y %+7.2f   Z %+7.2f\n",
                  gx, gy, gz);
    Serial.printf("  Temp (°C)  :  %.1f\n", mpuTemp);
    Serial.printf("  Reads      :  %lu\n", readCount);
  }

  delay(8);   // ~125 Hz read rate matches SMPLRT_DIV setting
}