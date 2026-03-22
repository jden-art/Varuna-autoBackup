#include <Wire.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <HTTPClient.h>

#define SDA_0   8
#define SCL_0   9
#define SDA_1   4
#define SCL_1   5
#define STATUS_LED 3
#define GPS_RX  6
#define GPS_TX  7
#define SIM_RX  15
#define SIM_TX  16
#define SIM_RST 17
#define BATTERY_PIN 2

#define MPU6050_ADDR 0x68
#define REG_PWR_MGMT_1 0x6B
#define REG_SMPLRT_DIV 0x19
#define REG_CONFIG 0x1A
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_WHO_AM_I 0x75
#define REG_ACCEL_XOUT_H 0x3B
#define ACCEL_SCALE 16384.0
#define GYRO_SCALE 131.0
#define ALPHA 0.98
#define DT_MAX 2.0

#define BMP280_ADDR 0x76
#define BMP280_REG_CHIP_ID 0xD0
#define BMP280_REG_CALIB 0x88
#define BMP280_REG_CTRL_MEAS 0xF4
#define BMP280_REG_CONFIG_R 0xF5
#define BMP280_REG_PRESS_MSB 0xF7

#define DS1307_ADDR 0x68
#define DS1307_REG_SEC 0x00
#define DS1307_REG_CONTROL 0x07

#define NMEA_BUF_SIZE 120
#define GPS_SYNC_INTERVAL_MS 86400000UL

#define SIM_RESPONSE_BUF_SIZE 256
#define AT_SUCCESS 0
#define AT_TIMEOUT 1
#define AT_ERROR   2

#define BATTERY_DIVIDER_RATIO  1.33
#define BATTERY_ADC_SAMPLES    16
#define BATTERY_LOW_THRESH     20.0
#define BATTERY_CRITICAL_THRESH 10.0
#define BATTERY_CUTOFF_MV      3000.0
#define BATTERY_CUTOFF_COUNT   3

#define TETHER_LATERAL_THRESH_MS2  0.15
#define TETHER_ANGLE_THRESH_DEG    3.0

#define MODE_SLACK     0
#define MODE_TAUT      1
#define MODE_FLOOD     2
#define MODE_SUBMERGED 3

#define SUBMERSION_PRESSURE_THRESH_HPA  5.0
#define FLOOD_ANGLE_THRESH_DEG          10.0
#define FLOOD_HEIGHT_RATIO              0.95

#define BASELINE_SIZE 48
#define BASELINE_INTERVAL 1800000UL

#define ZONE_NORMAL  0
#define ZONE_ALERT   1
#define ZONE_WARNING 2
#define ZONE_DANGER  3

#define RATE_SLOW     0
#define RATE_MODERATE 1
#define RATE_FAST     2

#define RATE_CLAMP_MAX 200.0
#define RATE_MIN_ELAPSED_SEC 60.0

#define SUSTAINED_BUF_SIZE       4
#define SUSTAINED_RISE_THRESH    0.5
#define SUSTAINED_RISE_MIN_PAIRS 2

#define FIREBASE_HOST   "varuna-flood-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_AUTH   ""
#define FIREBASE_PATH   "/varuna/live"
#define GPRS_APN        "bsnlnet"
#define STATION_ID      "VARUNA_TEST_01"

#define WIFI_SSID       "YOUR_WIFI_SSID"
#define WIFI_PASS       "YOUR_WIFI_PASSWORD"

#define TRANSPORT_NONE    0
#define TRANSPORT_GPRS    1
#define TRANSPORT_WIFI    2
int activeTransport = TRANSPORT_NONE;
int transportFailCount = 0;
#define TRANSPORT_FAIL_THRESHOLD  3

TwoWire I2C_0 = TwoWire(0);
TwoWire I2C_1 = TwoWire(1);
HardwareSerial GPS_Serial(1);
HardwareSerial SIM_Serial(2);

float ax, ay, az, gx, gy, gz, mpuTemp;
bool mpuHealthy = false;
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;
float refAccX = 0, refAccY = 0, refAccZ = 0, refTiltX = 0, refTiltY = 0;
bool calibrated = false;
#define GYRO_CAL_SAMPLES 1000
#define GYRO_CAL_DELAY_MS 2
#define ACCEL_CAL_SAMPLES 500
#define ACCEL_CAL_DELAY_MS 3
#define G_MIN_VALID 0.9
#define G_MAX_VALID 1.1
float filtTiltX = 0, filtTiltY = 0, correctedTiltX = 0, correctedTiltY = 0, theta = 0;
unsigned long prevTime = 0;
bool filterSeeded = false;

bool bmpAvailable = false;
float currentPressure = 0, currentTemperature = 0;
uint16_t bmpDigT1; int16_t bmpDigT2, bmpDigT3;
uint16_t bmpDigP1; int16_t bmpDigP2, bmpDigP3, bmpDigP4, bmpDigP5;
int16_t bmpDigP6, bmpDigP7, bmpDigP8, bmpDigP9;
int32_t bmpTFine;

bool ds1307Available = false, rtcTimeValid = false, softRtcSet = false;
uint32_t currentUnixTime = 0, softRtcBaseUnix = 0;
unsigned long softRtcBaseMillis = 0;

float gpsLat = 0, gpsLon = 0, gpsAlt = 0, gpsSpeed = 0, gpsHdop = 99.9;
int gpsSatellites = 0;
bool gpsFixValid = false, gpsTimeValid = false, gpsTimeSynced = false;
int gpsHour = 0, gpsMin = 0, gpsSec = 0, gpsDay = 0, gpsMonth = 0, gpsYear = 0;
unsigned long lastGpsSyncMillis = 0;
char nmeaBuffer[NMEA_BUF_SIZE];
int nmeaIndex = 0;
unsigned long gpsGGACount = 0, gpsRMCCount = 0, gpsChecksumFail = 0, gpsTotalSentences = 0, gpsBytesReceived = 0;

char simResponseBuffer[SIM_RESPONSE_BUF_SIZE];
bool simAvailable = false, simRegistered = false, simCardReady = false;
int simSignalRSSI = 0;
bool gprsConnected = false, sslSupported = false;

bool wifiConnected = false;

int postSuccessCount = 0, postFailCount = 0;

float batteryVoltage_mV = 0;
float batteryPercent = 0;
int   lowVoltageCount = 0;
String batteryState = "UNKNOWN";

float olpLength = 200.0;
float waterHeight = 0.0;
float horizontalDist = 0.0;

float lateralAccel = 0.0;
float lateralAccel_ms2 = 0.0;
bool  tetherTaut = false;

int   currentMode = MODE_SLACK;
float pressureDeviation = 0.0;
float estimatedDepth = 0.0;
float baselinePressure = 0.0;
float baselineBuffer[BASELINE_SIZE];
int   baselineIndex = 0;
int   baselineCount = 0;
unsigned long lastBaselineUpdate = 0;

const char* modeNames[] = {"SLACK", "TAUT", "FLOOD", "SUBMERGED"};

float alertLevelCm = 120.0;
float warningLevelCm = 180.0;
float dangerLevelCm = 250.0;
int   currentZone = ZONE_NORMAL;

const char* zoneNames[] = {"NORMAL", "ALERT", "WARNING", "DANGER"};

float previousHeight = -1.0;
unsigned long previousHeightTime = 0;
float ratePer15Min = 0.0;
int   currentRateCategory = RATE_SLOW;

const char* rateNames[] = {"SLOW", "MODERATE", "FAST"};

float sustainedBuffer[SUSTAINED_BUF_SIZE];
uint32_t sustainedTimeBuffer[SUSTAINED_BUF_SIZE];
int sustainedBufIndex = 0;
int sustainedBufCount = 0;
bool sustainedRise = false;


void mpuWriteReg(uint8_t r, uint8_t v) {
  I2C_0.beginTransmission(MPU6050_ADDR);
  I2C_0.write(r);
  I2C_0.write(v);
  I2C_0.endTransmission();
}

uint8_t mpuReadReg(uint8_t r) {
  I2C_0.beginTransmission(MPU6050_ADDR);
  I2C_0.write(r);
  I2C_0.endTransmission(false);
  I2C_0.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)1);
  return I2C_0.available() ? I2C_0.read() : 0xFF;
}

bool initMPU6050() {
  mpuWriteReg(REG_PWR_MGMT_1, 0x00);
  delay(100);
  mpuWriteReg(REG_SMPLRT_DIV, 0x07);
  mpuWriteReg(REG_CONFIG, 0x03);
  mpuWriteReg(REG_GYRO_CONFIG, 0x00);
  mpuWriteReg(REG_ACCEL_CONFIG, 0x00);
  uint8_t w = mpuReadReg(REG_WHO_AM_I);
  mpuHealthy = (w == 0x68 || w == 0x72);
  Serial.printf("  WHO_AM_I=0x%02X %s\n", w, mpuHealthy ? "✓" : "✗");
  return mpuHealthy;
}

bool readMPU6050() {
  I2C_0.beginTransmission(MPU6050_ADDR);
  I2C_0.write(REG_ACCEL_XOUT_H);
  if (I2C_0.endTransmission(false) != 0) return false;
  if (I2C_0.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)14) < 14) return false;
  uint8_t b[14];
  for (int i = 0; i < 14; i++) b[i] = I2C_0.read();
  ax = ((int16_t)((b[0] << 8) | b[1])) / ACCEL_SCALE;
  ay = ((int16_t)((b[2] << 8) | b[3])) / ACCEL_SCALE;
  az = ((int16_t)((b[4] << 8) | b[5])) / ACCEL_SCALE;
  gx = ((int16_t)((b[8] << 8) | b[9])) / GYRO_SCALE;
  gy = ((int16_t)((b[10] << 8) | b[11])) / GYRO_SCALE;
  gz = ((int16_t)((b[12] << 8) | b[13])) / GYRO_SCALE;
  mpuTemp = ((int16_t)((b[6] << 8) | b[7])) / 340.0 + 36.53;
  return true;
}

bool calibrateGyro() {
  Serial.println("\n  ── GYRO CAL ──");
  float sx = 0, sy = 0, sz = 0;
  int g = 0;
  for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
    if (readMPU6050()) { sx += gx; sy += gy; sz += gz; g++; }
    delay(GYRO_CAL_DELAY_MS);
  }
  if (g < GYRO_CAL_SAMPLES / 2) return false;
  gyroOffsetX = sx / g;
  gyroOffsetY = sy / g;
  gyroOffsetZ = sz / g;
  Serial.printf("  Offsets: X%+.4f Y%+.4f Z%+.4f\n", gyroOffsetX, gyroOffsetY, gyroOffsetZ);
  return true;
}

bool calibrateAccel() {
  Serial.println("  ── ACCEL CAL ──");
  float sx = 0, sy = 0, sz = 0;
  int g = 0;
  for (int i = 0; i < ACCEL_CAL_SAMPLES; i++) {
    if (readMPU6050()) {
      float t = sqrt(ax * ax + ay * ay + az * az);
      if (t >= G_MIN_VALID && t <= G_MAX_VALID) { sx += ax; sy += ay; sz += az; g++; }
    }
    delay(ACCEL_CAL_DELAY_MS);
  }
  if (g < ACCEL_CAL_SAMPLES / 2) return false;
  refAccX = sx / g; refAccY = sy / g; refAccZ = sz / g;
  refTiltX = atan2(refAccY, sqrt(refAccX * refAccX + refAccZ * refAccZ)) * 180.0 / PI;
  refTiltY = atan2(-refAccX, sqrt(refAccY * refAccY + refAccZ * refAccZ)) * 180.0 / PI;
  Serial.printf("  Ref tilt: X%+.3f° Y%+.3f°\n", refTiltX, refTiltY);
  return true;
}

bool recalibrate() {
  Serial.println("\n┌──────────────────────────────────────┐");
  Serial.println("│       MPU6050 CALIBRATION            │");
  Serial.println("└──────────────────────────────────────┘");
  pinMode(STATUS_LED, OUTPUT);
  for (int i = 0; i < 6; i++) {
    digitalWrite(STATUS_LED, HIGH); delay(250);
    digitalWrite(STATUS_LED, LOW); delay(250);
  }
  if (calibrateGyro() && calibrateAccel()) {
    calibrated = true; filterSeeded = false;
    Serial.println("  ✓ COMPLETE");
    return true;
  }
  calibrated = false;
  return false;
}

void updateComplementaryFilter() {
  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0;
  if (prevTime == 0 || dt > DT_MAX) dt = 0.01;
  prevTime = now;
  float acX = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI - refTiltX;
  float acY = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI - refTiltY;
  float grX = gx - gyroOffsetX, grY = gy - gyroOffsetY;
  if (!filterSeeded) { filtTiltX = acX; filtTiltY = acY; filterSeeded = true; }
  filtTiltX = ALPHA * (filtTiltX + grX * dt) + (1.0 - ALPHA) * acX;
  filtTiltY = ALPHA * (filtTiltY + grY * dt) + (1.0 - ALPHA) * acY;
  correctedTiltX = filtTiltX;
  correctedTiltY = filtTiltY;
  theta = sqrt(correctedTiltX * correctedTiltX + correctedTiltY * correctedTiltY);
}

void bmpWriteReg(uint8_t r, uint8_t v) {
  I2C_1.beginTransmission(BMP280_ADDR);
  I2C_1.write(r);
  I2C_1.write(v);
  I2C_1.endTransmission();
}

uint8_t bmpReadReg(uint8_t r) {
  I2C_1.beginTransmission(BMP280_ADDR);
  I2C_1.write(r);
  I2C_1.endTransmission(false);
  I2C_1.requestFrom((uint8_t)BMP280_ADDR, (uint8_t)1);
  return I2C_1.available() ? I2C_1.read() : 0xFF;
}

void bmpReadBytes(uint8_t r, uint8_t *buf, uint8_t len) {
  I2C_1.beginTransmission(BMP280_ADDR);
  I2C_1.write(r);
  I2C_1.endTransmission(false);
  I2C_1.requestFrom((uint8_t)BMP280_ADDR, len);
  for (uint8_t i = 0; i < len && I2C_1.available(); i++) buf[i] = I2C_1.read();
}

bool initBMP280() {
  Serial.println("\n── BMP280 INIT ──");
  uint8_t id = bmpReadReg(BMP280_REG_CHIP_ID);
  Serial.printf("  Chip ID=0x%02X %s\n", id, (id == 0x58 || id == 0x56 || id == 0x57 || id == 0x60) ? "✓" : "✗");
  if (id != 0x58 && id != 0x56 && id != 0x57 && id != 0x60) { bmpAvailable = false; return false; }
  uint8_t c[26];
  bmpReadBytes(BMP280_REG_CALIB, c, 26);
  bmpDigT1 = (uint16_t)(c[1] << 8) | c[0];
  bmpDigT2 = (int16_t)((c[3] << 8) | c[2]);
  bmpDigT3 = (int16_t)((c[5] << 8) | c[4]);
  bmpDigP1 = (uint16_t)(c[7] << 8) | c[6];
  bmpDigP2 = (int16_t)((c[9] << 8) | c[8]);
  bmpDigP3 = (int16_t)((c[11] << 8) | c[10]);
  bmpDigP4 = (int16_t)((c[13] << 8) | c[12]);
  bmpDigP5 = (int16_t)((c[15] << 8) | c[14]);
  bmpDigP6 = (int16_t)((c[17] << 8) | c[16]);
  bmpDigP7 = (int16_t)((c[19] << 8) | c[18]);
  bmpDigP8 = (int16_t)((c[21] << 8) | c[20]);
  bmpDigP9 = (int16_t)((c[23] << 8) | c[22]);
  bmpWriteReg(BMP280_REG_CONFIG_R, 0x10);
  bmpWriteReg(BMP280_REG_CTRL_MEAS, 0x57);
  delay(100);
  bmpAvailable = true;
  Serial.println("  ✓ BMP280 READY");
  return true;
}

int32_t bmpCompensateTemp(int32_t adcT) {
  int32_t v1 = ((((adcT >> 3) - ((int32_t)bmpDigT1 << 1))) * ((int32_t)bmpDigT2)) >> 11;
  int32_t v2 = (((((adcT >> 4) - ((int32_t)bmpDigT1)) * ((adcT >> 4) - ((int32_t)bmpDigT1))) >> 12) * ((int32_t)bmpDigT3)) >> 14;
  bmpTFine = v1 + v2;
  return (bmpTFine * 5 + 128) >> 8;
}

uint32_t bmpCompensatePress(int32_t adcP) {
  int64_t v1 = ((int64_t)bmpTFine) - 128000;
  int64_t v2 = v1 * v1 * (int64_t)bmpDigP6;
  v2 = v2 + ((v1 * (int64_t)bmpDigP5) << 17);
  v2 = v2 + (((int64_t)bmpDigP4) << 35);
  v1 = ((v1 * v1 * (int64_t)bmpDigP3) >> 8) + ((v1 * (int64_t)bmpDigP2) << 12);
  v1 = (((((int64_t)1) << 47) + v1)) * ((int64_t)bmpDigP1) >> 33;
  if (v1 == 0) return 0;
  int64_t p = 1048576 - adcP;
  p = (((p << 31) - v2) * 3125) / v1;
  v1 = (((int64_t)bmpDigP9) * (p >> 13) * (p >> 13)) >> 25;
  v2 = (((int64_t)bmpDigP8) * p) >> 19;
  p = ((p + v1 + v2) >> 8) + (((int64_t)bmpDigP7) << 4);
  return (uint32_t)p;
}

bool bmpReadData(float *temperature, float *pressure) {
  uint8_t b[6];
  bmpReadBytes(BMP280_REG_PRESS_MSB, b, 6);
  int32_t adcP = ((int32_t)b[0] << 12) | ((int32_t)b[1] << 4) | ((int32_t)b[2] >> 4);
  int32_t adcT = ((int32_t)b[3] << 12) | ((int32_t)b[4] << 4) | ((int32_t)b[5] >> 4);
  if (adcT == 0 || adcT == 0xFFFFF || adcP == 0 || adcP == 0xFFFFF) return false;
  *temperature = bmpCompensateTemp(adcT) / 100.0;
  *pressure = bmpCompensatePress(adcP) / 256.0 / 100.0;
  return true;
}

void initPressureBaseline() {
  Serial.println("\n── PRESSURE BASELINE INIT ──");
  if (!bmpAvailable) {
    Serial.println("  ✗ BMP280 not available");
    return;
  }
  float sum = 0;
  int good = 0;
  for (int i = 0; i < 10; i++) {
    float t, p;
    if (bmpReadData(&t, &p)) {
      if (p > 300.0 && p < 1200.0) {
        sum += p;
        good++;
      }
    }
    delay(100);
  }
  if (good > 0) {
    baselinePressure = sum / good;
    for (int i = 0; i < BASELINE_SIZE; i++) {
      baselineBuffer[i] = baselinePressure;
    }
    baselineCount = 1;
    baselineIndex = 1;
    lastBaselineUpdate = millis();
    Serial.printf("  Baseline: %.2f hPa (%d samples)\n", baselinePressure, good);
  } else {
    Serial.println("  ✗ Could not establish baseline");
  }
  Serial.println("  ✓ BASELINE READY");
}

void updatePressureBaseline() {
  if (!bmpAvailable) return;
  if (currentMode == MODE_SUBMERGED) return;
  if (millis() - lastBaselineUpdate < BASELINE_INTERVAL) return;
  if (currentPressure < 300.0 || currentPressure > 1200.0) return;
  baselineBuffer[baselineIndex] = currentPressure;
  baselineIndex = (baselineIndex + 1) % BASELINE_SIZE;
  if (baselineCount < BASELINE_SIZE) baselineCount++;
  float sum = 0;
  for (int i = 0; i < baselineCount; i++) {
    sum += baselineBuffer[i];
  }
  baselinePressure = sum / baselineCount;
  lastBaselineUpdate = millis();
}

void detectMode() {
  if (bmpAvailable && currentPressure > 0 && baselinePressure > 0) {
    pressureDeviation = currentPressure - baselinePressure;
  } else {
    pressureDeviation = 0.0;
  }
  if (pressureDeviation > SUBMERSION_PRESSURE_THRESH_HPA) {
    currentMode = MODE_SUBMERGED;
    estimatedDepth = pressureDeviation / 0.0981;
    waterHeight = olpLength + estimatedDepth;
    horizontalDist = 0.0;
    tetherTaut = true;
  }
  else if (lateralAccel_ms2 > TETHER_LATERAL_THRESH_MS2 && theta > TETHER_ANGLE_THRESH_DEG) {
    float tautHeight = olpLength * cos(theta * PI / 180.0);
    if (theta < FLOOD_ANGLE_THRESH_DEG && tautHeight > FLOOD_HEIGHT_RATIO * olpLength) {
      currentMode = MODE_FLOOD;
      waterHeight = olpLength;
      horizontalDist = olpLength * sin(theta * PI / 180.0);
      tetherTaut = true;
      estimatedDepth = 0.0;
    } else {
      currentMode = MODE_TAUT;
      waterHeight = tautHeight;
      horizontalDist = olpLength * sin(theta * PI / 180.0);
      tetherTaut = true;
      estimatedDepth = 0.0;
    }
  }
  else {
    currentMode = MODE_SLACK;
    waterHeight = 0.0;
    horizontalDist = 0.0;
    tetherTaut = false;
    estimatedDepth = 0.0;
  }
}

int classifyZone(float heightCm) {
  if (heightCm >= dangerLevelCm) return ZONE_DANGER;
  if (heightCm >= warningLevelCm) return ZONE_WARNING;
  if (heightCm >= alertLevelCm) return ZONE_ALERT;
  return ZONE_NORMAL;
}

void calculateRateOfChange(float currentHeight, unsigned long currentTime) {
  if (previousHeight < 0) {
    previousHeight = currentHeight;
    previousHeightTime = currentTime;
    ratePer15Min = 0.0;
    return;
  }
  float elapsed = (currentTime - previousHeightTime) / 1000.0;
  if (elapsed < RATE_MIN_ELAPSED_SEC) {
    return;
  }
  float change = currentHeight - previousHeight;
  ratePer15Min = change * (900.0 / elapsed);
  if (ratePer15Min > RATE_CLAMP_MAX || ratePer15Min < -RATE_CLAMP_MAX) {
    ratePer15Min = 0.0;
  }
  previousHeight = currentHeight;
  previousHeightTime = currentTime;
}

int classifyRate(float rate) {
  if (rate < 0.0) return RATE_SLOW;
  if (rate < 2.0) return RATE_SLOW;
  if (rate < 5.0) return RATE_MODERATE;
  return RATE_FAST;
}

void updateSustainedBuffer(float height, uint32_t timestamp) {
  sustainedBuffer[sustainedBufIndex] = height;
  sustainedTimeBuffer[sustainedBufIndex] = timestamp;
  sustainedBufIndex = (sustainedBufIndex + 1) % SUSTAINED_BUF_SIZE;
  if (sustainedBufCount < SUSTAINED_BUF_SIZE) sustainedBufCount++;

  if (sustainedBufCount < SUSTAINED_BUF_SIZE) {
    sustainedRise = false;
    return;
  }

  float ordered[SUSTAINED_BUF_SIZE];
  for (int i = 0; i < SUSTAINED_BUF_SIZE; i++) {
    ordered[i] = sustainedBuffer[(sustainedBufIndex + i) % SUSTAINED_BUF_SIZE];
  }

  bool netRising = (ordered[SUSTAINED_BUF_SIZE - 1] > ordered[0] + SUSTAINED_RISE_THRESH);

  int riseCount = 0;
  for (int i = 1; i < SUSTAINED_BUF_SIZE; i++) {
    if (ordered[i] > ordered[i - 1] + SUSTAINED_RISE_THRESH) riseCount++;
  }

  sustainedRise = netRising && (riseCount >= SUSTAINED_RISE_MIN_PAIRS);
}

uint8_t bcdToDec(uint8_t v) { return ((v >> 4) * 10) + (v & 0x0F); }
uint8_t decToBcd(uint8_t v) { return ((v / 10) << 4) | (v % 10); }

uint32_t dateToUnix(int yr, int mo, int dy, int hr, int mi, int sc) {
  uint32_t days = 0;
  for (int y = 1970; y < yr; y++) {
    bool l = (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0));
    days += l ? 366 : 365;
  }
  static const int md[] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  for (int m = 1; m < mo; m++) {
    days += md[m];
    if (m == 2 && (yr % 4 == 0 && (yr % 100 != 0 || yr % 400 == 0))) days++;
  }
  days += (dy - 1);
  return days * 86400UL + (uint32_t)hr * 3600UL + (uint32_t)mi * 60UL + (uint32_t)sc;
}

void unixToDate(uint32_t ut, int &y, int &mo, int &d, int &h, int &mi, int &s) {
  s = ut % 60; ut /= 60;
  mi = ut % 60; ut /= 60;
  h = ut % 24; ut /= 24;
  uint32_t td = ut; y = 1970;
  while (true) {
    bool l = (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0));
    uint16_t dy = l ? 366 : 365;
    if (td < dy) break;
    td -= dy; y++;
  }
  static const int md[] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  bool l = (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0));
  mo = 1;
  while (mo <= 12) {
    int dm = md[mo];
    if (mo == 2 && l) dm = 29;
    if (td < (uint32_t)dm) break;
    td -= dm; mo++;
  }
  d = td + 1;
}

void formatDateTime(uint32_t ut, char *buf, size_t len) {
  int y, mo, d, h, mi, s;
  unixToDate(ut, y, mo, d, h, mi, s);
  snprintf(buf, len, "%04d-%02d-%02d %02d:%02d:%02d", y, mo, d, h, mi, s);
}

bool ds1307Probe() {
  I2C_1.beginTransmission(DS1307_ADDR);
  return (I2C_1.endTransmission() == 0);
}

bool ds1307Read(int &y, int &mo, int &d, int &h, int &mi, int &s) {
  I2C_1.beginTransmission(DS1307_ADDR);
  I2C_1.write(DS1307_REG_SEC);
  if (I2C_1.endTransmission() != 0) return false;
  if (I2C_1.requestFrom((uint8_t)DS1307_ADDR, (uint8_t)7) < 7) return false;
  uint8_t r[7];
  for (int i = 0; i < 7; i++) r[i] = I2C_1.read();
  s = bcdToDec(r[0] & 0x7F);
  mi = bcdToDec(r[1] & 0x7F);
  if (r[2] & 0x40) {
    h = bcdToDec(r[2] & 0x1F);
    if (r[2] & 0x20) h += 12;
    if (h == 12) h = 0;
    if (h == 24) h = 12;
  } else {
    h = bcdToDec(r[2] & 0x3F);
  }
  d = bcdToDec(r[4] & 0x3F);
  mo = bcdToDec(r[5] & 0x1F);
  y = 2000 + bcdToDec(r[6]);
  return true;
}

bool ds1307Write(int y, int mo, int d, int h, int mi, int s) {
  I2C_1.beginTransmission(DS1307_ADDR);
  I2C_1.write(DS1307_REG_SEC);
  I2C_1.write(decToBcd(s) & 0x7F);
  I2C_1.write(decToBcd(mi));
  I2C_1.write(decToBcd(h));
  I2C_1.write(decToBcd(1));
  I2C_1.write(decToBcd(d));
  I2C_1.write(decToBcd(mo));
  I2C_1.write(decToBcd(y - 2000));
  return (I2C_1.endTransmission() == 0);
}

bool ds1307Init() {
  I2C_1.beginTransmission(DS1307_ADDR);
  I2C_1.write(DS1307_REG_SEC);
  if (I2C_1.endTransmission() != 0) return false;
  if (I2C_1.requestFrom((uint8_t)DS1307_ADDR, (uint8_t)1) < 1) return false;
  uint8_t sec = I2C_1.read();
  if (sec & 0x80) {
    I2C_1.beginTransmission(DS1307_ADDR);
    I2C_1.write(DS1307_REG_SEC);
    I2C_1.write(sec & 0x7F);
    I2C_1.endTransmission();
  }
  I2C_1.beginTransmission(DS1307_ADDR);
  I2C_1.write(DS1307_REG_CONTROL);
  I2C_1.write(0x00);
  I2C_1.endTransmission();
  return true;
}

void softRtcWrite(int y, int mo, int d, int h, int mi, int s) {
  softRtcBaseUnix = dateToUnix(y, mo, d, h, mi, s);
  softRtcBaseMillis = millis();
  softRtcSet = true;
}

uint32_t softRtcRead() {
  if (!softRtcSet) return 0;
  return softRtcBaseUnix + (millis() - softRtcBaseMillis) / 1000;
}

bool initRTC() {
  Serial.println("\n── RTC INIT ──");
  if (ds1307Probe()) {
    if (ds1307Init()) {
      ds1307Available = true;
      int y, mo, d, h, mi, s;
      if (ds1307Read(y, mo, d, h, mi, s) && y >= 2024) {
        currentUnixTime = dateToUnix(y, mo, d, h, mi, s);
        rtcTimeValid = true;
        softRtcWrite(y, mo, d, h, mi, s);
        Serial.println("  ✓ DS1307 active");
        return true;
      }
    }
  }
  ds1307Available = false;
  softRtcWrite(2025, 1, 1, 0, 0, 0);
  rtcTimeValid = false;
  Serial.println("  Software RTC active");
  return true;
}

void readRTC(int &y, int &mo, int &d, int &h, int &mi, int &s) {
  if (ds1307Available && ds1307Read(y, mo, d, h, mi, s)) {
    currentUnixTime = dateToUnix(y, mo, d, h, mi, s);
    return;
  }
  currentUnixTime = softRtcRead();
  if (currentUnixTime > 0) unixToDate(currentUnixTime, y, mo, d, h, mi, s);
  else { y = 2025; mo = 1; d = 1; h = 0; mi = 0; s = 0; }
}

void writeRTC(int y, int mo, int d, int h, int mi, int s) {
  softRtcWrite(y, mo, d, h, mi, s);
  currentUnixTime = dateToUnix(y, mo, d, h, mi, s);
  rtcTimeValid = true;
  if (ds1307Available) ds1307Write(y, mo, d, h, mi, s);
}

uint32_t getBestTimestamp() {
  if (ds1307Available) {
    int y, mo, d, h, mi, s;
    if (ds1307Read(y, mo, d, h, mi, s)) return dateToUnix(y, mo, d, h, mi, s);
  }
  if (softRtcSet) return softRtcRead();
  return millis() / 1000;
}

float nmeaToDecimal(float raw, char dir) {
  int deg = (int)(raw / 100);
  float min = raw - (deg * 100.0);
  float dec = deg + min / 60.0;
  if (dir == 'S' || dir == 'W') dec = -dec;
  return dec;
}

bool nmeaGetField(const char *s, int fi, char *out, int ol) {
  int cf = 0, i = 0, len = strlen(s);
  while (i < len && cf < fi) { if (s[i] == ',') cf++; i++; }
  if (cf != fi) { out[0] = '\0'; return false; }
  int j = 0;
  while (i < len && s[i] != ',' && s[i] != '*' && j < ol - 1) { out[j++] = s[i++]; }
  out[j] = '\0';
  return (j > 0);
}

bool validateNMEAChecksum(const char *s) {
  const char *st = strchr(s, '$');
  const char *star = strchr(s, '*');
  if (!st || !star || star <= st + 1) return false;
  uint8_t comp = 0;
  for (const char *p = st + 1; p < star; p++) comp ^= (uint8_t)*p;
  char hs[3] = {star[1], star[2], '\0'};
  return (comp == (uint8_t)strtoul(hs, NULL, 16));
}

void parseGGA(const char *s) {
  char f[20]; char d[4];
  if (nmeaGetField(s, 1, f, sizeof(f)) && strlen(f) >= 6) {
    gpsHour = (f[0] - '0') * 10 + (f[1] - '0');
    gpsMin = (f[2] - '0') * 10 + (f[3] - '0');
    gpsSec = (f[4] - '0') * 10 + (f[5] - '0');
  }
  if (nmeaGetField(s, 6, f, sizeof(f))) { gpsFixValid = (atoi(f) >= 1); }
  else gpsFixValid = false;
  if (!gpsFixValid) return;
  if (nmeaGetField(s, 2, f, sizeof(f)) && nmeaGetField(s, 3, d, sizeof(d)))
    gpsLat = nmeaToDecimal(atof(f), d[0]);
  if (nmeaGetField(s, 4, f, sizeof(f)) && nmeaGetField(s, 5, d, sizeof(d)))
    gpsLon = nmeaToDecimal(atof(f), d[0]);
  if (nmeaGetField(s, 7, f, sizeof(f))) gpsSatellites = atoi(f);
  if (nmeaGetField(s, 8, f, sizeof(f))) gpsHdop = atof(f);
  if (nmeaGetField(s, 9, f, sizeof(f))) gpsAlt = atof(f);
  gpsGGACount++;
}

void parseRMC(const char *s) {
  char f[20]; char d[4];
  if (nmeaGetField(s, 1, f, sizeof(f)) && strlen(f) >= 6) {
    gpsHour = (f[0] - '0') * 10 + (f[1] - '0');
    gpsMin = (f[2] - '0') * 10 + (f[3] - '0');
    gpsSec = (f[4] - '0') * 10 + (f[5] - '0');
  }
  if (nmeaGetField(s, 2, f, sizeof(f))) { gpsFixValid = (f[0] == 'A'); }
  if (!gpsFixValid) return;
  if (nmeaGetField(s, 3, f, sizeof(f)) && nmeaGetField(s, 4, d, sizeof(d)))
    gpsLat = nmeaToDecimal(atof(f), d[0]);
  if (nmeaGetField(s, 5, f, sizeof(f)) && nmeaGetField(s, 6, d, sizeof(d)))
    gpsLon = nmeaToDecimal(atof(f), d[0]);
  if (nmeaGetField(s, 7, f, sizeof(f))) gpsSpeed = atof(f) * 1.852;
  if (nmeaGetField(s, 9, f, sizeof(f)) && strlen(f) >= 6) {
    gpsDay = (f[0] - '0') * 10 + (f[1] - '0');
    gpsMonth = (f[2] - '0') * 10 + (f[3] - '0');
    gpsYear = 2000 + (f[4] - '0') * 10 + (f[5] - '0');
    gpsTimeValid = (gpsYear >= 2024);
  }
  gpsRMCCount++;
}

void parseNMEA(const char *s) {
  gpsTotalSentences++;
  if (strncmp(s + 1, "GPGGA", 5) == 0 || strncmp(s + 1, "GNGGA", 5) == 0) parseGGA(s);
  else if (strncmp(s + 1, "GPRMC", 5) == 0 || strncmp(s + 1, "GNRMC", 5) == 0) parseRMC(s);
}

void processGPS() {
  while (GPS_Serial.available()) {
    char c = GPS_Serial.read();
    gpsBytesReceived++;
    if (c == '\n' || c == '\r') {
      if (nmeaIndex > 0) {
        nmeaBuffer[nmeaIndex] = '\0';
        if (nmeaBuffer[0] == '$') {
          if (validateNMEAChecksum(nmeaBuffer)) parseNMEA(nmeaBuffer);
          else gpsChecksumFail++;
        }
        nmeaIndex = 0;
      }
    } else {
      if (nmeaIndex < NMEA_BUF_SIZE - 1) nmeaBuffer[nmeaIndex++] = c;
      else nmeaIndex = 0;
    }
  }
}

void syncRTCfromGPS() {
  if (!gpsFixValid || !gpsTimeValid || gpsYear < 2024 || gpsHdop > 5.0 || gpsSatellites < 4) return;
  if (gpsTimeSynced && (millis() - lastGpsSyncMillis < GPS_SYNC_INTERVAL_MS)) return;
  writeRTC(gpsYear, gpsMonth, gpsDay, gpsHour, gpsMin, gpsSec);
  gpsTimeSynced = true;
  lastGpsSyncMillis = millis();
  Serial.println("  ✓ RTC synced from GPS");
}

void initGPS() {
  Serial.println("\n── GPS INIT ──");
  GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  nmeaIndex = 0;
  Serial.println("  ✓ GPS UART active");
}

int send_at_command(const char *cmd, const char *expected, unsigned long timeout_ms) {
  memset(simResponseBuffer, 0, SIM_RESPONSE_BUF_SIZE);
  int bi = 0;
  while (SIM_Serial.available()) SIM_Serial.read();
  SIM_Serial.print(cmd);
  SIM_Serial.print("\r\n");
  Serial.printf("    TX: %s\n", cmd);
  unsigned long start = millis();
  while (millis() - start < timeout_ms) {
    while (SIM_Serial.available()) {
      char c = SIM_Serial.read();
      if (bi < SIM_RESPONSE_BUF_SIZE - 1) {
        simResponseBuffer[bi++] = c;
        simResponseBuffer[bi] = '\0';
      }
      if (expected && strstr(simResponseBuffer, expected)) {
        Serial.printf("    RX: ");
        for (int i = 0; i < bi; i++) {
          char ch = simResponseBuffer[i];
          if (ch == '\r') continue;
          if (ch == '\n') { Serial.print(" | "); continue; }
          Serial.print(ch);
        }
        Serial.println("  ✓");
        return AT_SUCCESS;
      }
      if (strstr(simResponseBuffer, "ERROR")) {
        Serial.printf("    RX: ");
        for (int i = 0; i < bi; i++) {
          char ch = simResponseBuffer[i];
          if (ch == '\r') continue;
          if (ch == '\n') { Serial.print(" | "); continue; }
          Serial.print(ch);
        }
        Serial.println("  ✗ ERROR");
        return AT_ERROR;
      }
    }
    delay(10);
  }
  Serial.print("    RX: ");
  if (bi > 0) {
    for (int i = 0; i < bi; i++) {
      char ch = simResponseBuffer[i];
      if (ch == '\r') continue;
      if (ch == '\n') { Serial.print(" | "); continue; }
      Serial.print(ch);
    }
  } else {
    Serial.print("(none)");
  }
  Serial.println("  ✗ TIMEOUT");
  return AT_TIMEOUT;
}

void simHardwareReset() {
  pinMode(SIM_RST, OUTPUT);
  digitalWrite(SIM_RST, LOW);
  delay(200);
  digitalWrite(SIM_RST, HIGH);
  delay(3000);
}

int parseSignalRSSI(const char *r) {
  const char *p = strstr(r, "+CSQ:");
  if (!p) return 0;
  p += 5;
  while (*p == ' ') p++;
  return atoi(p);
}

int parseRegStatus(const char *r) {
  const char *p = strstr(r, "+CREG:");
  if (!p) return -1;
  p += 6;
  while (*p == ' ') p++;
  while (*p && *p != ',') p++;
  if (*p == ',') p++;
  return atoi(p);
}

bool initSIM800L() {
  Serial.println("\n── SIM800L INIT ──");
  simHardwareReset();
  bool atOk = false;
  for (int i = 1; i <= 5; i++) {
    Serial.printf("  AT attempt %d/5:\n", i);
    if (send_at_command("AT", "OK", 2000) == AT_SUCCESS) { atOk = true; break; }
    delay(1000);
  }
  if (!atOk) { simAvailable = false; Serial.println("  ✗ No response from SIM800L"); return false; }
  simAvailable = true;
  send_at_command("ATE0", "OK", 1000);
  send_at_command("AT+CMGF=1", "OK", 1000);
  if (send_at_command("AT+CPIN?", "READY", 5000) == AT_SUCCESS) simCardReady = true;
  else { Serial.println("  ✗ SIM not ready"); return false; }
  simRegistered = false;
  for (int i = 1; i <= 5; i++) {
    if (send_at_command("AT+CREG?", "+CREG:", 5000) == AT_SUCCESS) {
      int reg = parseRegStatus(simResponseBuffer);
      if (reg == 1 || reg == 5) { simRegistered = true; break; }
    }
    delay(3000);
  }
  if (send_at_command("AT+CSQ", "+CSQ:", 2000) == AT_SUCCESS)
    simSignalRSSI = parseSignalRSSI(simResponseBuffer);
  Serial.printf("  SIM:%s Reg:%s RSSI:%d\n", simCardReady ? "✓" : "✗", simRegistered ? "✓" : "✗", simSignalRSSI);
  return simAvailable && simCardReady;
}

bool gprsInit() {
  Serial.println("\n── GPRS INIT ──");
  if (!simAvailable || !simCardReady) { Serial.println("  ✗ SIM not ready"); return false; }
  send_at_command("AT+HTTPTERM", "OK", 1000);
  send_at_command("AT+SAPBR=0,1", "OK", 3000);
  delay(1000);
  send_at_command("AT+SAPBR=3,1,\"Contype\",\"GPRS\"", "OK", 2000);
  char apnCmd[80];
  snprintf(apnCmd, sizeof(apnCmd), "AT+SAPBR=3,1,\"APN\",\"%s\"", GPRS_APN);
  send_at_command(apnCmd, "OK", 2000);
  Serial.println("  Opening bearer (30s)...");
  send_at_command("AT+SAPBR=1,1", "OK", 30000);
  if (send_at_command("AT+SAPBR=2,1", "+SAPBR:", 5000) == AT_SUCCESS) {
    if (!strstr(simResponseBuffer, "0.0.0.0")) {
      gprsConnected = true;
      Serial.println("  ✓ GPRS CONNECTED");
      send_at_command("AT+HTTPINIT", "OK", 2000);
      sslSupported = (send_at_command("AT+HTTPSSL=1", "OK", 2000) == AT_SUCCESS);
      send_at_command("AT+HTTPTERM", "OK", 1000);
      Serial.printf("  SSL: %s\n", sslSupported ? "supported ✓" : "NOT supported ✗");
      return true;
    }
  }
  gprsConnected = false;
  Serial.println("  ✗ GPRS failed");
  return false;
}

bool gprsPostToFirebase(const char *payload, int payloadLen) {
  if (!gprsConnected) return false;
  char url[350];
  if (strlen(FIREBASE_AUTH) > 0)
    snprintf(url, sizeof(url), "https://%s%s.json?auth=%s", FIREBASE_HOST, FIREBASE_PATH, FIREBASE_AUTH);
  else
    snprintf(url, sizeof(url), "https://%s%s.json", FIREBASE_HOST, FIREBASE_PATH);
  if (send_at_command("AT+HTTPINIT", "OK", 2000) != AT_SUCCESS) return false;
  send_at_command("AT+HTTPPARA=\"CID\",1", "OK", 1000);
  if (sslSupported) send_at_command("AT+HTTPSSL=1", "OK", 2000);
  char urlCmd[400];
  snprintf(urlCmd, sizeof(urlCmd), "AT+HTTPPARA=\"URL\",\"%s\"", url);
  send_at_command(urlCmd, "OK", 2000);
  send_at_command("AT+HTTPPARA=\"CONTENT\",\"application/json\"", "OK", 1000);
  send_at_command("AT+HTTPPARA=\"USERDATA\",\"X-HTTP-Method-Override: PUT\"", "OK", 1000);
  char dataCmd[32];
  snprintf(dataCmd, sizeof(dataCmd), "AT+HTTPDATA=%d,10000", payloadLen);
  if (send_at_command(dataCmd, "DOWNLOAD", 5000) != AT_SUCCESS) {
    send_at_command("AT+HTTPTERM", "OK", 1000);
    return false;
  }
  SIM_Serial.write((const uint8_t *)payload, payloadLen);
  delay(2000);
  bool success = false;
  int httpStatus = 0;
  if (send_at_command("AT+HTTPACTION=1", "+HTTPACTION:", 30000) == AT_SUCCESS) {
    const char *p = strstr(simResponseBuffer, "+HTTPACTION:");
    if (p) {
      p += 12;
      while (*p && *p != ',') p++;
      if (*p == ',') p++;
      httpStatus = atoi(p);
    }
    success = (httpStatus == 200);
    Serial.printf("  HTTP %d %s\n", httpStatus, success ? "✓" : "✗");
  }
  send_at_command("AT+HTTPTERM", "OK", 1000);
  return success;
}

bool initWiFi() {
  Serial.println("\n── WiFi INIT ──");
  if (strcmp(WIFI_SSID, "YOUR_WIFI_SSID") == 0) {
    Serial.println("  ✗ WiFi SSID not configured");
    return false;
  }
  Serial.printf("  Connecting to: %s\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.printf("  ✓ WiFi CONNECTED\n");
    Serial.printf("  IP: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("  RSSI: %d dBm\n", WiFi.RSSI());
    return true;
  }
  Serial.println("  ✗ WiFi connection failed");
  wifiConnected = false;
  return false;
}

bool wifiPostToFirebase(const char *payload, int payloadLen) {
  if (WiFi.status() != WL_CONNECTED) {
    wifiConnected = false;
    return false;
  }
  char url[350];
  if (strlen(FIREBASE_AUTH) > 0)
    snprintf(url, sizeof(url), "https://%s%s.json?auth=%s", FIREBASE_HOST, FIREBASE_PATH, FIREBASE_AUTH);
  else
    snprintf(url, sizeof(url), "https://%s%s.json", FIREBASE_HOST, FIREBASE_PATH);
  HTTPClient http;
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("X-HTTP-Method-Override", "PUT");
  int httpCode = http.PUT((uint8_t *)payload, payloadLen);
  String response = http.getString();
  http.end();
  if (httpCode == 200) {
    return true;
  }
  return false;
}

int buildSensorPayload(char *buf, int bufSize) {
  uint32_t ts = getBestTimestamp();
  return snprintf(buf, bufSize,
    "{\"station\":\"%s\","
    "\"theta\":%.2f,"
    "\"tiltX\":%.2f,\"tiltY\":%.2f,"
    "\"waterHeight\":%.1f,\"olpLength\":%.1f,\"horizontalDist\":%.1f,"
    "\"lateralAccel\":%.4f,\"lateralAccel_ms2\":%.3f,\"tetherTaut\":%s,"
    "\"mode\":%d,\"modeName\":\"%s\","
    "\"zone\":%d,\"zoneName\":\"%s\","
    "\"ratePer15Min\":%.2f,\"rateCategory\":%d,\"rateName\":\"%s\","
    "\"sustainedRise\":%s,"
    "\"pressureDeviation\":%.2f,\"estimatedDepth\":%.1f,"
    "\"baselinePressure\":%.2f,"
    "\"pressure\":%.2f,\"temperature\":%.1f,"
    "\"gpsLat\":%.6f,\"gpsLon\":%.6f,"
    "\"gpsFix\":%s,\"satellites\":%d,"
    "\"battery_mV\":%.0f,\"battery_pct\":%.1f,\"battery_state\":\"%s\","
    "\"transport\":\"%s\","
    "\"timestamp\":%u,\"uptime\":%lu}",
    STATION_ID, theta, correctedTiltX, correctedTiltY,
    waterHeight, olpLength, horizontalDist,
    lateralAccel, lateralAccel_ms2, tetherTaut ? "true" : "false",
    currentMode, modeNames[currentMode],
    currentZone, zoneNames[currentZone],
    ratePer15Min, currentRateCategory, rateNames[currentRateCategory],
    sustainedRise ? "true" : "false",
    pressureDeviation, estimatedDepth,
    baselinePressure,
    currentPressure, currentTemperature,
    gpsLat, gpsLon,
    gpsFixValid ? "true" : "false", gpsSatellites,
    batteryVoltage_mV, batteryPercent, batteryState.c_str(),
    activeTransport == TRANSPORT_GPRS ? "GPRS" : "WiFi",
    ts, millis() / 1000);
}

bool postToFirebase() {
  char payload[700];
  int len = buildSensorPayload(payload, sizeof(payload));
  bool success = false;
  if (activeTransport == TRANSPORT_GPRS) {
    success = gprsPostToFirebase(payload, len);
  } else if (activeTransport == TRANSPORT_WIFI) {
    success = wifiPostToFirebase(payload, len);
  }
  if (success) {
    postSuccessCount++;
    transportFailCount = 0;
  } else {
    postFailCount++;
    transportFailCount++;
    if (transportFailCount >= TRANSPORT_FAIL_THRESHOLD) {
      transportFailCount = 0;
      if (activeTransport == TRANSPORT_GPRS) {
        if (initWiFi()) { activeTransport = TRANSPORT_WIFI; }
        else { gprsInit(); }
      } else if (activeTransport == TRANSPORT_WIFI) {
        if (simAvailable && gprsInit()) { activeTransport = TRANSPORT_GPRS; }
        else { initWiFi(); }
      }
    }
  }
  return success;
}

void initTransport() {
  Serial.println("\n┌──────────────────────────────────────────────────┐");
  Serial.println("│         TRANSPORT INITIALIZATION                 │");
  Serial.println("└──────────────────────────────────────────────────┘");
  SIM_Serial.begin(9600, SERIAL_8N1, SIM_RX, SIM_TX);
  bool simOk = initSIM800L();
  if (simOk && simRegistered) {
    if (gprsInit() && gprsConnected) {
      activeTransport = TRANSPORT_GPRS;
      Serial.println("\n  ✓ PRIMARY TRANSPORT: GPRS\n");
      return;
    }
  }
  if (initWiFi()) {
    activeTransport = TRANSPORT_WIFI;
    Serial.println("\n  ✓ FALLBACK TRANSPORT: WiFi\n");
    return;
  }
  activeTransport = TRANSPORT_NONE;
  Serial.println("\n  ✗ NO TRANSPORT AVAILABLE\n");
}

float readBatteryVoltage() {
  uint32_t sum = 0;
  for (int i = 0; i < BATTERY_ADC_SAMPLES; i++) {
    sum += analogRead(BATTERY_PIN);
    delayMicroseconds(500);
  }
  float avgReading = (float)sum / BATTERY_ADC_SAMPLES;
  float adcVoltage_mV = avgReading * 3300.0 / 4095.0;
  float battery_mV = adcVoltage_mV * BATTERY_DIVIDER_RATIO;
  return battery_mV;
}

float batteryPercentage(float voltage_mV) {
  float pct;
  if (voltage_mV >= 4200.0) pct = 100.0;
  else if (voltage_mV >= 4100.0) pct = 90.0 + (voltage_mV - 4100.0) * 10.0 / 100.0;
  else if (voltage_mV >= 3850.0) pct = 70.0 + (voltage_mV - 3850.0) * 20.0 / 250.0;
  else if (voltage_mV >= 3700.0) pct = 40.0 + (voltage_mV - 3700.0) * 30.0 / 150.0;
  else if (voltage_mV >= 3500.0) pct = 20.0 + (voltage_mV - 3500.0) * 20.0 / 200.0;
  else if (voltage_mV >= 3300.0) pct = 5.0 + (voltage_mV - 3300.0) * 15.0 / 200.0;
  else if (voltage_mV >= 3000.0) pct = 0.0 + (voltage_mV - 3000.0) * 5.0 / 300.0;
  else pct = 0.0;
  if (pct > 100.0) pct = 100.0;
  if (pct < 0.0) pct = 0.0;
  return pct;
}

String classifyBatteryState(float voltage_mV, float percent) {
  if (voltage_mV >= 3850.0) return "GOOD";
  if (percent >= BATTERY_LOW_THRESH) return "OK";
  if (percent >= BATTERY_CRITICAL_THRESH) return "LOW";
  if (voltage_mV >= BATTERY_CUTOFF_MV) return "CRITICAL";
  return "CUTOFF";
}

void updateBattery() {
  batteryVoltage_mV = readBatteryVoltage();
  batteryPercent = batteryPercentage(batteryVoltage_mV);
  batteryState = classifyBatteryState(batteryVoltage_mV, batteryPercent);
  if (batteryVoltage_mV < BATTERY_CUTOFF_MV) {
    lowVoltageCount++;
    if (lowVoltageCount >= BATTERY_CUTOFF_COUNT) {
      Serial.println("\n  BATTERY CUTOFF — ENTERING DEEP SLEEP");
      Serial.flush();
      delay(100);
      esp_deep_sleep_start();
    }
  } else {
    lowVoltageCount = 0;
  }
}

void initBattery() {
  Serial.println("\n── BATTERY ADC INIT ──");
  analogReadResolution(12);
  pinMode(BATTERY_PIN, INPUT);
  updateBattery();
  Serial.printf("  Voltage: %.0f mV  Percent: %.1f%%  State: %s\n",
                batteryVoltage_mV, batteryPercent, batteryState.c_str());
  Serial.println("  ✓ BATTERY ADC READY");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("┌──────────────────────────────────────────────────┐");
  Serial.println("│   VARUNA — Phase 3 Step 3: Sustained Rise       │");
  Serial.println("└──────────────────────────────────────────────────┘");

  I2C_0.begin(SDA_0, SCL_0, 100000);
  I2C_1.begin(SDA_1, SCL_1, 100000);
  Serial.println("\nI2C buses initialised");

  Serial.println("\n── MPU6050 INIT ──");
  if (!initMPU6050()) { Serial.println("HALT"); while (1) delay(1000); }
  recalibrate();

  initBMP280();
  initPressureBaseline();
  initRTC();
  initGPS();
  initBattery();
  initTransport();

  for (int i = 0; i < SUSTAINED_BUF_SIZE; i++) {
    sustainedBuffer[i] = 0.0;
    sustainedTimeBuffer[i] = 0;
  }
  sustainedBufIndex = 0;
  sustainedBufCount = 0;
  sustainedRise = false;
  Serial.println("\n── SUSTAINED RISE BUFFER INIT ──");
  Serial.printf("  Buffer size: %d readings\n", SUSTAINED_BUF_SIZE);
  Serial.printf("  Rise threshold: %.1f cm between consecutive readings\n", SUSTAINED_RISE_THRESH);
  Serial.printf("  Min rising pairs: %d of %d\n", SUSTAINED_RISE_MIN_PAIRS, SUSTAINED_BUF_SIZE - 1);
  Serial.println("  ✓ SUSTAINED BUFFER READY");

  if (activeTransport != TRANSPORT_NONE) {
    postToFirebase();
  }

  prevTime = millis();

  Serial.println("\n══════════════════════════════════════════════════");
  Serial.printf("  OLP: %.1f cm\n", olpLength);
  Serial.printf("  Zones: ALERT=%.0f  WARNING=%.0f  DANGER=%.0f cm\n",
                alertLevelCm, warningLevelCm, dangerLevelCm);
  Serial.printf("  Rates: SLOW<2  MODERATE<5  FAST≥5 cm/15min\n");
  Serial.printf("  Sustained: %d readings, %.1fcm thresh, %d min pairs\n",
                SUSTAINED_BUF_SIZE, SUSTAINED_RISE_THRESH, SUSTAINED_RISE_MIN_PAIRS);
  Serial.println("══════════════════════════════════════════════════\n");
}

unsigned long lastPrint = 0;
unsigned long lastBmpRead = 0;
unsigned long lastRtcRead = 0;
unsigned long lastGpsPrint = 0;
unsigned long lastFirebasePost = 0;
unsigned long lastTransportRetry = 0;
unsigned long lastBatteryRead = 0;
unsigned long lastSustainedUpdate = 0;

#define FIREBASE_POST_INTERVAL   30000
#define TRANSPORT_RETRY_INTERVAL 120000
#define BATTERY_READ_INTERVAL    30000
#define SUSTAINED_UPDATE_INTERVAL 10000

int rtcYear, rtcMonth, rtcDay, rtcHour, rtcMin, rtcSec;

void loop() {
  if (!mpuHealthy || !calibrated) { delay(1000); return; }

  processGPS();
  syncRTCfromGPS();

  if (!readMPU6050()) { delay(10); return; }
  updateComplementaryFilter();

  lateralAccel = sqrt(ax * ax + ay * ay);
  lateralAccel_ms2 = lateralAccel * 9.81;

  if (bmpAvailable && (millis() - lastBmpRead >= 2000)) {
    lastBmpRead = millis();
    bmpReadData(&currentTemperature, &currentPressure);
    updatePressureBaseline();
  }

  detectMode();

  currentZone = classifyZone(waterHeight);
  calculateRateOfChange(waterHeight, millis());
  currentRateCategory = classifyRate(ratePer15Min);

  if (millis() - lastSustainedUpdate >= SUSTAINED_UPDATE_INTERVAL) {
    lastSustainedUpdate = millis();
    updateSustainedBuffer(waterHeight, getBestTimestamp());
  }

  if (millis() - lastRtcRead >= 1000) {
    lastRtcRead = millis();
    readRTC(rtcYear, rtcMonth, rtcDay, rtcHour, rtcMin, rtcSec);
  }

  if (millis() - lastBatteryRead >= BATTERY_READ_INTERVAL) {
    lastBatteryRead = millis();
    updateBattery();
  }

  if (activeTransport != TRANSPORT_NONE && (millis() - lastFirebasePost >= FIREBASE_POST_INTERVAL)) {
    lastFirebasePost = millis();
    postToFirebase();
  }

  if (activeTransport == TRANSPORT_NONE && (millis() - lastTransportRetry >= TRANSPORT_RETRY_INTERVAL)) {
    lastTransportRetry = millis();
    initTransport();
  }

  if (activeTransport == TRANSPORT_WIFI && WiFi.status() != WL_CONNECTED) {
    WiFi.reconnect();
    delay(3000);
    if (WiFi.status() != WL_CONNECTED) {
      wifiConnected = false;
      transportFailCount++;
    }
  }

  if (millis() - lastPrint >= 500) {
    lastPrint = millis();

    char timeStr[32];
    formatDateTime(currentUnixTime, timeStr, sizeof(timeStr));

    Serial.printf("M%d(%s) Z%d(%s) R:%s %+.1f S:%s | ",
                  currentMode, modeNames[currentMode],
                  currentZone, zoneNames[currentZone],
                  rateNames[currentRateCategory], ratePer15Min,
                  sustainedRise ? "YES" : "no");

    Serial.printf("θ%6.2f° H=%6.1fcm | ", theta, waterHeight);

    if (bmpAvailable && currentPressure > 0)
      Serial.printf("P:%+.2fhPa | ", pressureDeviation);

    if (currentMode == MODE_SUBMERGED)
      Serial.printf("depth:%.1fcm | ", estimatedDepth);

    Serial.printf("%s | ", timeStr);

    if (activeTransport == TRANSPORT_GPRS)
      Serial.printf("GPRS | ");
    else if (activeTransport == TRANSPORT_WIFI)
      Serial.printf("WiFi | ");
    else
      Serial.print("-- | ");

    Serial.printf("BAT:%.0f%% | ", batteryPercent);
    Serial.printf("FB:%d/%d", postSuccessCount, postFailCount);
    Serial.println();
  }

  if (millis() - lastGpsPrint >= 30000) {
    lastGpsPrint = millis();

    Serial.printf("  GPS: fix:%s sats:%d hdop:%.1f synced:%s\n",
                  gpsFixValid ? "Y" : "N", gpsSatellites, gpsHdop, gpsTimeSynced ? "Y" : "N");

    Serial.println("  ┌── FLOOD STATUS ────────────────────────────────┐");
    Serial.printf( "  │  Mode:       %d (%s)\n", currentMode, modeNames[currentMode]);
    Serial.printf( "  │  Zone:       %d (%s)\n", currentZone, zoneNames[currentZone]);
    Serial.printf( "  │  Rate:       %+.2f cm/15min (%s)\n", ratePer15Min, rateNames[currentRateCategory]);
    Serial.printf( "  │  Sustained:  %s (buf:%d/%d)\n", sustainedRise ? "YES ▲▲▲" : "no", sustainedBufCount, SUSTAINED_BUF_SIZE);
    if (sustainedBufCount > 0) {
      Serial.print("  │  Buffer:     [");
      for (int i = 0; i < sustainedBufCount; i++) {
        int idx = (sustainedBufIndex - sustainedBufCount + i + SUSTAINED_BUF_SIZE) % SUSTAINED_BUF_SIZE;
        if (i > 0) Serial.print(", ");
        Serial.printf("%.1f", sustainedBuffer[idx]);
      }
      Serial.println("]");
    }
    Serial.printf( "  │  Height:     %.1f cm\n", waterHeight);
    Serial.printf( "  │  OLP:        %.1f cm\n", olpLength);
    Serial.printf( "  │  H/L:        %.3f\n", olpLength > 0 ? waterHeight / olpLength : 0);
    Serial.printf( "  │  Theta:      %.2f°\n", theta);
    Serial.printf( "  │  Tether:     %s\n", tetherTaut ? "TAUT" : "SLACK");
    Serial.printf( "  │  Lat Accel:  %.4fg (%.3f m/s²)\n", lateralAccel, lateralAccel_ms2);
    Serial.printf( "  │  P dev:      %+.2f hPa\n", pressureDeviation);
    Serial.printf( "  │  Depth:      %.1f cm\n", estimatedDepth);
    Serial.printf( "  │  Baseline:   %.2f hPa\n", baselinePressure);
    Serial.printf( "  │  Thresholds: A=%.0f W=%.0f D=%.0f cm\n",
                   alertLevelCm, warningLevelCm, dangerLevelCm);
    Serial.printf( "  │  Battery:    %.0fmV %.1f%% %s\n",
                   batteryVoltage_mV, batteryPercent, batteryState.c_str());
    Serial.println("  └────────────────────────────────────────────────┘");
  }

  delay(8);
}