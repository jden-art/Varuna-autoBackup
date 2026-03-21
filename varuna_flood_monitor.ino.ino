#include <Wire.h>
#include <HardwareSerial.h>

// ─── PIN DEFINITIONS ───
#define SDA_0  8
#define SCL_0  9
#define SDA_1  4
#define SCL_1  5
#define STATUS_LED 3
#define GPS_RX 6
#define GPS_TX 7

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
#define BMP280_REG_CALIB     0x88
#define BMP280_REG_CTRL_MEAS 0xF4
#define BMP280_REG_CONFIG_R  0xF5
#define BMP280_REG_PRESS_MSB 0xF7

// ─── DS1307 CONSTANTS ───
#define DS1307_ADDR         0x68
#define DS1307_REG_SEC      0x00
#define DS1307_REG_CONTROL  0x07

// ─── GPS CONSTANTS ───
#define NMEA_BUF_SIZE  120
#define GPS_SYNC_INTERVAL_MS  86400000UL  // 24 hours

// ─── I2C BUSES ───
TwoWire I2C_0 = TwoWire(0);
TwoWire I2C_1 = TwoWire(1);

// ─── GPS UART ───
HardwareSerial GPS_Serial(1);

// ─── MPU6050 DATA ───
float ax, ay, az, gx, gy, gz, mpuTemp;
bool  mpuHealthy = false;

// ─── CALIBRATION ───
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
float currentPressure = 0, currentTemperature = 0;
uint16_t bmpDigT1; int16_t bmpDigT2, bmpDigT3;
uint16_t bmpDigP1; int16_t bmpDigP2, bmpDigP3, bmpDigP4, bmpDigP5;
int16_t bmpDigP6, bmpDigP7, bmpDigP8, bmpDigP9;
int32_t bmpTFine;

// ─── RTC DATA ───
bool     ds1307Available = false;
bool     rtcTimeValid = false;
uint32_t currentUnixTime = 0;
uint32_t softRtcBaseUnix = 0;
unsigned long softRtcBaseMillis = 0;
bool     softRtcSet = false;

// ─── GPS DATA ───
float gpsLat = 0, gpsLon = 0;
float gpsAlt = 0, gpsSpeed = 0;
float gpsHdop = 99.9;
int   gpsSatellites = 0;
bool  gpsFixValid = false;
int   gpsHour = 0, gpsMin = 0, gpsSec = 0;
int   gpsDay = 0, gpsMonth = 0, gpsYear = 0;
bool  gpsTimeValid = false;

// GPS→RTC sync tracking
bool  gpsTimeSynced = false;
unsigned long lastGpsSyncMillis = 0;

// NMEA parser state
char  nmeaBuffer[NMEA_BUF_SIZE];
int   nmeaIndex = 0;

// sentence counters for diagnostics
unsigned long gpsGGACount = 0;
unsigned long gpsRMCCount = 0;
unsigned long gpsChecksumFail = 0;
unsigned long gpsTotalSentences = 0;
unsigned long gpsBytesReceived = 0;


// ══════════════════════════════════════════════
//  MPU6050 SECTION
// ══════════════════════════════════════════════

void mpuWriteReg(uint8_t reg, uint8_t value) {
  I2C_0.beginTransmission(MPU6050_ADDR); I2C_0.write(reg); I2C_0.write(value); I2C_0.endTransmission();
}
uint8_t mpuReadReg(uint8_t reg) {
  I2C_0.beginTransmission(MPU6050_ADDR); I2C_0.write(reg); I2C_0.endTransmission(false);
  I2C_0.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)1);
  return I2C_0.available() ? I2C_0.read() : 0xFF;
}
bool initMPU6050() {
  mpuWriteReg(REG_PWR_MGMT_1, 0x00); delay(100);
  mpuWriteReg(REG_SMPLRT_DIV, 0x07);
  mpuWriteReg(REG_CONFIG, 0x03);
  mpuWriteReg(REG_GYRO_CONFIG, 0x00);
  mpuWriteReg(REG_ACCEL_CONFIG, 0x00);
  uint8_t w = mpuReadReg(REG_WHO_AM_I);
  mpuHealthy = (w == 0x68 || w == 0x72);
  Serial.printf("  WHO_AM_I = 0x%02X %s\n", w, mpuHealthy ? "✓" : "✗");
  return mpuHealthy;
}
bool readMPU6050() {
  I2C_0.beginTransmission(MPU6050_ADDR); I2C_0.write(REG_ACCEL_XOUT_H);
  if (I2C_0.endTransmission(false) != 0) return false;
  if (I2C_0.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)14) < 14) return false;
  uint8_t b[14]; for (int i=0;i<14;i++) b[i]=I2C_0.read();
  ax=((int16_t)((b[0]<<8)|b[1]))/ACCEL_SCALE;
  ay=((int16_t)((b[2]<<8)|b[3]))/ACCEL_SCALE;
  az=((int16_t)((b[4]<<8)|b[5]))/ACCEL_SCALE;
  gx=((int16_t)((b[8]<<8)|b[9]))/GYRO_SCALE;
  gy=((int16_t)((b[10]<<8)|b[11]))/GYRO_SCALE;
  gz=((int16_t)((b[12]<<8)|b[13]))/GYRO_SCALE;
  mpuTemp=((int16_t)((b[6]<<8)|b[7]))/340.0+36.53;
  return true;
}
bool calibrateGyro() {
  Serial.println("\n  ── GYRO CALIBRATION ──");
  float sx=0,sy=0,sz=0; int g=0;
  for(int i=0;i<GYRO_CAL_SAMPLES;i++){if(readMPU6050()){sx+=gx;sy+=gy;sz+=gz;g++;}delay(GYRO_CAL_DELAY_MS);}
  if(g<GYRO_CAL_SAMPLES/2)return false;
  gyroOffsetX=sx/g;gyroOffsetY=sy/g;gyroOffsetZ=sz/g;
  Serial.printf("  Offsets: X%+.4f Y%+.4f Z%+.4f\n",gyroOffsetX,gyroOffsetY,gyroOffsetZ);
  return true;
}
bool calibrateAccel() {
  Serial.println("  ── ACCEL CALIBRATION ──");
  float sx=0,sy=0,sz=0; int g=0;
  for(int i=0;i<ACCEL_CAL_SAMPLES;i++){
    if(readMPU6050()){float t=sqrt(ax*ax+ay*ay+az*az);
    if(t>=G_MIN_VALID&&t<=G_MAX_VALID){sx+=ax;sy+=ay;sz+=az;g++;}}
    delay(ACCEL_CAL_DELAY_MS);}
  if(g<ACCEL_CAL_SAMPLES/2)return false;
  refAccX=sx/g;refAccY=sy/g;refAccZ=sz/g;
  refTiltX=atan2(refAccY,sqrt(refAccX*refAccX+refAccZ*refAccZ))*180.0/PI;
  refTiltY=atan2(-refAccX,sqrt(refAccY*refAccY+refAccZ*refAccZ))*180.0/PI;
  Serial.printf("  Ref tilt: X%+.3f° Y%+.3f°\n",refTiltX,refTiltY);
  return true;
}
bool recalibrate() {
  Serial.println("\n┌──────────────────────────────────────┐");
  Serial.println("│       MPU6050 CALIBRATION            │");
  Serial.println("└──────────────────────────────────────┘");
  pinMode(STATUS_LED,OUTPUT);
  for(int i=0;i<6;i++){digitalWrite(STATUS_LED,HIGH);delay(250);digitalWrite(STATUS_LED,LOW);delay(250);}
  if(calibrateGyro()&&calibrateAccel()){calibrated=true;filterSeeded=false;Serial.println("  ✓ CALIBRATION COMPLETE");return true;}
  calibrated=false;return false;
}
void updateComplementaryFilter() {
  unsigned long now=millis();float dt=(now-prevTime)/1000.0;
  if(prevTime==0||dt>DT_MAX)dt=0.01;prevTime=now;
  float acX=atan2(ay,sqrt(ax*ax+az*az))*180.0/PI-refTiltX;
  float acY=atan2(-ax,sqrt(ay*ay+az*az))*180.0/PI-refTiltY;
  float grX=gx-gyroOffsetX,grY=gy-gyroOffsetY;
  if(!filterSeeded){filtTiltX=acX;filtTiltY=acY;filterSeeded=true;}
  filtTiltX=ALPHA*(filtTiltX+grX*dt)+(1.0-ALPHA)*acX;
  filtTiltY=ALPHA*(filtTiltY+grY*dt)+(1.0-ALPHA)*acY;
  correctedTiltX=filtTiltX;correctedTiltY=filtTiltY;
  theta=sqrt(correctedTiltX*correctedTiltX+correctedTiltY*correctedTiltY);
}


// ══════════════════════════════════════════════
//  BMP280 SECTION
// ══════════════════════════════════════════════

void bmpWriteReg(uint8_t reg,uint8_t val){I2C_1.beginTransmission(BMP280_ADDR);I2C_1.write(reg);I2C_1.write(val);I2C_1.endTransmission();}
uint8_t bmpReadReg(uint8_t reg){I2C_1.beginTransmission(BMP280_ADDR);I2C_1.write(reg);I2C_1.endTransmission(false);I2C_1.requestFrom((uint8_t)BMP280_ADDR,(uint8_t)1);return I2C_1.available()?I2C_1.read():0xFF;}
void bmpReadBytes(uint8_t reg,uint8_t*buf,uint8_t len){I2C_1.beginTransmission(BMP280_ADDR);I2C_1.write(reg);I2C_1.endTransmission(false);I2C_1.requestFrom((uint8_t)BMP280_ADDR,len);for(uint8_t i=0;i<len&&I2C_1.available();i++)buf[i]=I2C_1.read();}
bool initBMP280(){
  Serial.println("\n── BMP280 INIT ──");
  uint8_t id=bmpReadReg(BMP280_REG_CHIP_ID);
  Serial.printf("  Chip ID=0x%02X %s\n",id,(id==0x58||id==0x56||id==0x57||id==0x60)?"✓":"✗");
  if(id!=0x58&&id!=0x56&&id!=0x57&&id!=0x60){bmpAvailable=false;return false;}
  uint8_t c[26];bmpReadBytes(BMP280_REG_CALIB,c,26);
  bmpDigT1=(uint16_t)(c[1]<<8)|c[0];bmpDigT2=(int16_t)((c[3]<<8)|c[2]);bmpDigT3=(int16_t)((c[5]<<8)|c[4]);
  bmpDigP1=(uint16_t)(c[7]<<8)|c[6];bmpDigP2=(int16_t)((c[9]<<8)|c[8]);bmpDigP3=(int16_t)((c[11]<<8)|c[10]);
  bmpDigP4=(int16_t)((c[13]<<8)|c[12]);bmpDigP5=(int16_t)((c[15]<<8)|c[14]);bmpDigP6=(int16_t)((c[17]<<8)|c[16]);
  bmpDigP7=(int16_t)((c[19]<<8)|c[18]);bmpDigP8=(int16_t)((c[21]<<8)|c[20]);bmpDigP9=(int16_t)((c[23]<<8)|c[22]);
  bmpWriteReg(BMP280_REG_CONFIG_R,0x10);bmpWriteReg(BMP280_REG_CTRL_MEAS,0x57);delay(100);
  bmpAvailable=true;Serial.println("  ✓ BMP280 READY");return true;
}
int32_t bmpCompensateTemp(int32_t adcT){
  int32_t v1=((((adcT>>3)-((int32_t)bmpDigT1<<1)))*((int32_t)bmpDigT2))>>11;
  int32_t v2=(((((adcT>>4)-((int32_t)bmpDigT1))*((adcT>>4)-((int32_t)bmpDigT1)))>>12)*((int32_t)bmpDigT3))>>14;
  bmpTFine=v1+v2;return(bmpTFine*5+128)>>8;
}
uint32_t bmpCompensatePress(int32_t adcP){
  int64_t v1=((int64_t)bmpTFine)-128000;int64_t v2=v1*v1*(int64_t)bmpDigP6;
  v2=v2+((v1*(int64_t)bmpDigP5)<<17);v2=v2+(((int64_t)bmpDigP4)<<35);
  v1=((v1*v1*(int64_t)bmpDigP3)>>8)+((v1*(int64_t)bmpDigP2)<<12);
  v1=(((((int64_t)1)<<47)+v1))*((int64_t)bmpDigP1)>>33;if(v1==0)return 0;
  int64_t p=1048576-adcP;p=(((p<<31)-v2)*3125)/v1;
  v1=(((int64_t)bmpDigP9)*(p>>13)*(p>>13))>>25;v2=(((int64_t)bmpDigP8)*p)>>19;
  p=((p+v1+v2)>>8)+(((int64_t)bmpDigP7)<<4);return(uint32_t)p;
}
bool bmpReadData(float*temperature,float*pressure){
  uint8_t b[6];bmpReadBytes(BMP280_REG_PRESS_MSB,b,6);
  int32_t adcP=((int32_t)b[0]<<12)|((int32_t)b[1]<<4)|((int32_t)b[2]>>4);
  int32_t adcT=((int32_t)b[3]<<12)|((int32_t)b[4]<<4)|((int32_t)b[5]>>4);
  if(adcT==0||adcT==0xFFFFF||adcP==0||adcP==0xFFFFF)return false;
  *temperature=bmpCompensateTemp(adcT)/100.0;
  *pressure=bmpCompensatePress(adcP)/256.0/100.0;return true;
}


// ══════════════════════════════════════════════
//  RTC SECTION (Software + DS1307 abstraction)
// ══════════════════════════════════════════════

uint8_t bcdToDec(uint8_t v){return((v>>4)*10)+(v&0x0F);}
uint8_t decToBcd(uint8_t v){return((v/10)<<4)|(v%10);}

uint32_t dateToUnix(int year,int month,int day,int hour,int minute,int second){
  uint32_t days=0;
  for(int y=1970;y<year;y++){bool lp=(y%4==0&&(y%100!=0||y%400==0));days+=lp?366:365;}
  static const int md[]={0,31,28,31,30,31,30,31,31,30,31,30,31};
  for(int m=1;m<month;m++){days+=md[m];if(m==2&&(year%4==0&&(year%100!=0||year%400==0)))days++;}
  days+=(day-1);
  return days*86400UL+(uint32_t)hour*3600UL+(uint32_t)minute*60UL+(uint32_t)second;
}

void unixToDate(uint32_t ut,int&y,int&mo,int&d,int&h,int&mi,int&s){
  s=ut%60;ut/=60;mi=ut%60;ut/=60;h=ut%24;ut/=24;
  uint32_t td=ut;y=1970;
  while(true){bool lp=(y%4==0&&(y%100!=0||y%400==0));uint16_t dy=lp?366:365;if(td<dy)break;td-=dy;y++;}
  static const int md[]={0,31,28,31,30,31,30,31,31,30,31,30,31};
  bool lp=(y%4==0&&(y%100!=0||y%400==0));mo=1;
  while(mo<=12){int dm=md[mo];if(mo==2&&lp)dm=29;if(td<(uint32_t)dm)break;td-=dm;mo++;}
  d=td+1;
}

void formatDateTime(uint32_t ut,char*buf,size_t len){
  int y,mo,d,h,mi,s;unixToDate(ut,y,mo,d,h,mi,s);
  snprintf(buf,len,"%04d-%02d-%02d %02d:%02d:%02d",y,mo,d,h,mi,s);
}

// DS1307 functions
bool ds1307Probe(){I2C_1.beginTransmission(DS1307_ADDR);return(I2C_1.endTransmission()==0);}
bool ds1307Read(int&y,int&mo,int&d,int&h,int&mi,int&s){
  I2C_1.beginTransmission(DS1307_ADDR);I2C_1.write(DS1307_REG_SEC);
  if(I2C_1.endTransmission()!=0)return false;
  if(I2C_1.requestFrom((uint8_t)DS1307_ADDR,(uint8_t)7)<7)return false;
  uint8_t r[7];for(int i=0;i<7;i++)r[i]=I2C_1.read();
  s=bcdToDec(r[0]&0x7F);mi=bcdToDec(r[1]&0x7F);
  if(r[2]&0x40){h=bcdToDec(r[2]&0x1F);if(r[2]&0x20)h+=12;if(h==12)h=0;if(h==24)h=12;}
  else h=bcdToDec(r[2]&0x3F);
  d=bcdToDec(r[4]&0x3F);mo=bcdToDec(r[5]&0x1F);y=2000+bcdToDec(r[6]);return true;
}
bool ds1307Write(int y,int mo,int d,int h,int mi,int s){
  I2C_1.beginTransmission(DS1307_ADDR);I2C_1.write(DS1307_REG_SEC);
  I2C_1.write(decToBcd(s)&0x7F);I2C_1.write(decToBcd(mi));I2C_1.write(decToBcd(h));
  I2C_1.write(decToBcd(1));I2C_1.write(decToBcd(d));I2C_1.write(decToBcd(mo));I2C_1.write(decToBcd(y-2000));
  return(I2C_1.endTransmission()==0);
}
bool ds1307Init(){
  I2C_1.beginTransmission(DS1307_ADDR);I2C_1.write(DS1307_REG_SEC);
  if(I2C_1.endTransmission()!=0)return false;
  if(I2C_1.requestFrom((uint8_t)DS1307_ADDR,(uint8_t)1)<1)return false;
  uint8_t sec=I2C_1.read();
  if(sec&0x80){I2C_1.beginTransmission(DS1307_ADDR);I2C_1.write(DS1307_REG_SEC);I2C_1.write(sec&0x7F);I2C_1.endTransmission();}
  I2C_1.beginTransmission(DS1307_ADDR);I2C_1.write(DS1307_REG_CONTROL);I2C_1.write(0x00);I2C_1.endTransmission();
  return true;
}

// Software RTC
void softRtcWrite(int y,int mo,int d,int h,int mi,int s){softRtcBaseUnix=dateToUnix(y,mo,d,h,mi,s);softRtcBaseMillis=millis();softRtcSet=true;}
uint32_t softRtcRead(){if(!softRtcSet)return 0;return softRtcBaseUnix+(millis()-softRtcBaseMillis)/1000;}

// Unified interface
bool initRTC(){
  Serial.println("\n── RTC INIT ──");
  if(ds1307Probe()){
    Serial.println("  DS1307 detected");
    if(ds1307Init()){
      ds1307Available=true;
      int y,mo,d,h,mi,s;
      if(ds1307Read(y,mo,d,h,mi,s)&&y>=2024){
        currentUnixTime=dateToUnix(y,mo,d,h,mi,s);rtcTimeValid=true;
        softRtcWrite(y,mo,d,h,mi,s);
        char buf[32];formatDateTime(currentUnixTime,buf,sizeof(buf));
        Serial.printf("  Time: %s ✓\n",buf);return true;
      }
    }
  }
  ds1307Available=false;
  softRtcWrite(2025,1,1,0,0,0);rtcTimeValid=false;
  Serial.println("  DS1307 not found → Software RTC");
  Serial.println("  ✓ Software RTC active (awaiting GPS sync)");
  return true;
}
void readRTC(int&y,int&mo,int&d,int&h,int&mi,int&s){
  if(ds1307Available&&ds1307Read(y,mo,d,h,mi,s)){currentUnixTime=dateToUnix(y,mo,d,h,mi,s);return;}
  currentUnixTime=softRtcRead();if(currentUnixTime>0)unixToDate(currentUnixTime,y,mo,d,h,mi,s);
  else{y=2025;mo=1;d=1;h=0;mi=0;s=0;}
}
void writeRTC(int y,int mo,int d,int h,int mi,int s){
  softRtcWrite(y,mo,d,h,mi,s);currentUnixTime=dateToUnix(y,mo,d,h,mi,s);rtcTimeValid=true;
  if(ds1307Available)ds1307Write(y,mo,d,h,mi,s);
}
uint32_t getBestTimestamp(){
  if(ds1307Available){int y,mo,d,h,mi,s;if(ds1307Read(y,mo,d,h,mi,s))return dateToUnix(y,mo,d,h,mi,s);}
  if(softRtcSet)return softRtcRead();
  return millis()/1000;
}


// ══════════════════════════════════════════════
//  GPS SECTION
// ══════════════════════════════════════════════

// ── NMEA coordinate conversion ──
// Raw NMEA format: DDMM.MMMM
// Convert to decimal degrees

float nmeaToDecimal(float raw, char direction) {
  int degrees = (int)(raw / 100);
  float minutes = raw - (degrees * 100.0);
  float decimal = degrees + minutes / 60.0;
  if (direction == 'S' || direction == 'W') decimal = -decimal;
  return decimal;
}

// ── Extract field from comma-separated NMEA sentence ──
// fieldIndex 0 = sentence type, 1 = first data field, etc.
// Returns pointer into the buffer (not a copy)

bool nmeaGetField(const char *sentence, int fieldIndex, char *out, int outLen) {
  int currentField = 0;
  int i = 0;
  int len = strlen(sentence);

  // find start of requested field
  while (i < len && currentField < fieldIndex) {
    if (sentence[i] == ',') currentField++;
    i++;
  }

  if (currentField != fieldIndex) {
    out[0] = '\0';
    return false;
  }

  // copy until next comma, asterisk, or end
  int j = 0;
  while (i < len && sentence[i] != ',' && sentence[i] != '*' && j < outLen - 1) {
    out[j++] = sentence[i++];
  }
  out[j] = '\0';

  return (j > 0);  // true if field is non-empty
}

// ── Validate NMEA checksum ──
// XOR all bytes between '$' and '*', compare to hex after '*'

bool validateNMEAChecksum(const char *sentence) {
  // find $ and *
  const char *start = strchr(sentence, '$');
  const char *star  = strchr(sentence, '*');

  if (!start || !star || star <= start + 1) return false;

  // compute XOR
  uint8_t computed = 0;
  for (const char *p = start + 1; p < star; p++) {
    computed ^= (uint8_t)*p;
  }

  // parse stated checksum (2 hex chars after *)
  char hexStr[3] = { star[1], star[2], '\0' };
  uint8_t stated = (uint8_t)strtoul(hexStr, NULL, 16);

  return (computed == stated);
}

// ── Parse GGA sentence ──
// $GPGGA,HHMMSS.SS,DDMM.MMMM,N,DDDMM.MMMM,E,Q,SAT,HDOP,ALT,M,...

void parseGGA(const char *sentence) {
  char field[20];

  // field 1: time (HHMMSS.SS)
  if (nmeaGetField(sentence, 1, field, sizeof(field))) {
    if (strlen(field) >= 6) {
      gpsHour = (field[0]-'0')*10 + (field[1]-'0');
      gpsMin  = (field[2]-'0')*10 + (field[3]-'0');
      gpsSec  = (field[4]-'0')*10 + (field[5]-'0');
    }
  }

  // field 6: fix quality (0=none, 1=GPS, 2=DGPS)
  if (nmeaGetField(sentence, 6, field, sizeof(field))) {
    int quality = atoi(field);
    gpsFixValid = (quality >= 1);
  } else {
    gpsFixValid = false;
  }

  if (!gpsFixValid) return;  // don't parse position if no fix

  // field 2,3: latitude
  char dir[4];
  if (nmeaGetField(sentence, 2, field, sizeof(field)) &&
      nmeaGetField(sentence, 3, dir, sizeof(dir))) {
    gpsLat = nmeaToDecimal(atof(field), dir[0]);
  }

  // field 4,5: longitude
  if (nmeaGetField(sentence, 4, field, sizeof(field)) &&
      nmeaGetField(sentence, 5, dir, sizeof(dir))) {
    gpsLon = nmeaToDecimal(atof(field), dir[0]);
  }

  // field 7: satellites
  if (nmeaGetField(sentence, 7, field, sizeof(field))) {
    gpsSatellites = atoi(field);
  }

  // field 8: HDOP
  if (nmeaGetField(sentence, 8, field, sizeof(field))) {
    gpsHdop = atof(field);
  }

  // field 9: altitude (meters)
  if (nmeaGetField(sentence, 9, field, sizeof(field))) {
    gpsAlt = atof(field);
  }

  gpsGGACount++;
}

// ── Parse RMC sentence ──
// $GPRMC,HHMMSS.SS,A,DDMM.MMMM,N,DDDMM.MMMM,E,SPD,CRS,DDMMYY,...

void parseRMC(const char *sentence) {
  char field[20];

  // field 1: time
  if (nmeaGetField(sentence, 1, field, sizeof(field))) {
    if (strlen(field) >= 6) {
      gpsHour = (field[0]-'0')*10 + (field[1]-'0');
      gpsMin  = (field[2]-'0')*10 + (field[3]-'0');
      gpsSec  = (field[4]-'0')*10 + (field[5]-'0');
    }
  }

  // field 2: status (A=active/valid, V=void)
  if (nmeaGetField(sentence, 2, field, sizeof(field))) {
    gpsFixValid = (field[0] == 'A');
  }

  if (!gpsFixValid) return;

  // field 3,4: latitude
  char dir[4];
  if (nmeaGetField(sentence, 3, field, sizeof(field)) &&
      nmeaGetField(sentence, 4, dir, sizeof(dir))) {
    gpsLat = nmeaToDecimal(atof(field), dir[0]);
  }

  // field 5,6: longitude
  if (nmeaGetField(sentence, 5, field, sizeof(field)) &&
      nmeaGetField(sentence, 6, dir, sizeof(dir))) {
    gpsLon = nmeaToDecimal(atof(field), dir[0]);
  }

  // field 7: speed (knots)
  if (nmeaGetField(sentence, 7, field, sizeof(field))) {
    gpsSpeed = atof(field) * 1.852;  // knots → km/h
  }

  // field 9: date (DDMMYY)
  if (nmeaGetField(sentence, 9, field, sizeof(field))) {
    if (strlen(field) >= 6) {
      gpsDay   = (field[0]-'0')*10 + (field[1]-'0');
      gpsMonth = (field[2]-'0')*10 + (field[3]-'0');
      gpsYear  = 2000 + (field[4]-'0')*10 + (field[5]-'0');
      gpsTimeValid = (gpsYear >= 2024);
    }
  }

  gpsRMCCount++;
}

// ── Parse one complete NMEA sentence ──

void parseNMEA(const char *sentence) {
  gpsTotalSentences++;

  // identify sentence type (check for both GP and GN prefixes)
  if (strncmp(sentence + 1, "GPGGA", 5) == 0 || strncmp(sentence + 1, "GNGGA", 5) == 0) {
    parseGGA(sentence);
  }
  else if (strncmp(sentence + 1, "GPRMC", 5) == 0 || strncmp(sentence + 1, "GNRMC", 5) == 0) {
    parseRMC(sentence);
  }
  // other sentence types silently ignored
}

// ── Non-blocking GPS processing ──
// Call every loop iteration. Accumulates bytes, processes
// complete sentences as they arrive.

void processGPS() {
  while (GPS_Serial.available()) {
    char c = GPS_Serial.read();
    gpsBytesReceived++;

    if (c == '\n' || c == '\r') {
      if (nmeaIndex > 0) {
        nmeaBuffer[nmeaIndex] = '\0';

        // only process sentences starting with $
        if (nmeaBuffer[0] == '$') {
          if (validateNMEAChecksum(nmeaBuffer)) {
            parseNMEA(nmeaBuffer);
          } else {
            gpsChecksumFail++;
          }
        }

        nmeaIndex = 0;
      }
    } else {
      if (nmeaIndex < NMEA_BUF_SIZE - 1) {
        nmeaBuffer[nmeaIndex++] = c;
      } else {
        nmeaIndex = 0;  // overflow — discard
      }
    }
  }
}

// ── GPS → RTC sync ──
// Conditions: valid fix, valid date, year >= 2024,
// HDOP <= 5.0, sats >= 4, not synced in last 24 hours

void syncRTCfromGPS() {
  if (!gpsFixValid) return;
  if (!gpsTimeValid) return;
  if (gpsYear < 2024) return;
  if (gpsHdop > 5.0) return;
  if (gpsSatellites < 4) return;

  // check 24-hour cooldown
  if (gpsTimeSynced && (millis() - lastGpsSyncMillis < GPS_SYNC_INTERVAL_MS)) return;

  Serial.println("\n  ── GPS → RTC SYNC ──");
  Serial.printf("  GPS time: %04d-%02d-%02d %02d:%02d:%02d UTC\n",
                gpsYear, gpsMonth, gpsDay, gpsHour, gpsMin, gpsSec);
  Serial.printf("  Sats: %d  HDOP: %.1f\n", gpsSatellites, gpsHdop);

  // write to RTC (both software and DS1307 if present)
  writeRTC(gpsYear, gpsMonth, gpsDay, gpsHour, gpsMin, gpsSec);

  gpsTimeSynced = true;
  lastGpsSyncMillis = millis();

  char buf[32];
  formatDateTime(currentUnixTime, buf, sizeof(buf));
  Serial.printf("  RTC set to: %s (unix: %u)\n", buf, currentUnixTime);
  Serial.println("  ✓ GPS → RTC sync complete");
}

// ── Initialize GPS ──

void initGPS() {
  Serial.println("\n── GPS INIT ──");
  GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  nmeaIndex = 0;
  Serial.println("  UART1 at 9600 baud (RX=GPIO6, TX=GPIO7)");
  Serial.println("  Waiting for NMEA data...");
  Serial.println("  ✓ GPS UART active");
}


// ══════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("┌──────────────────────────────────────────────────┐");
  Serial.println("│   VARUNA — Step 1.7: GPS NMEA Parser            │");
  Serial.println("└──────────────────────────────────────────────────┘");

  I2C_0.begin(SDA_0, SCL_0, 100000);
  I2C_1.begin(SDA_1, SCL_1, 100000);
  Serial.println("\nI2C buses initialised");

  Serial.println("\n── MPU6050 INIT ──");
  if (!initMPU6050()) { Serial.println("HALT"); while(1) delay(1000); }
  recalibrate();

  initBMP280();
  initRTC();
  initGPS();

  prevTime = millis();

  Serial.println("\n══════════════════════════════════════════════════════");
  Serial.println("  LIVE OUTPUT at 2 Hz");
  Serial.println("  GPS status updates every 5 seconds");
  Serial.println("  Place GPS module near a window for sky view");
  Serial.println("  First fix may take 30s–10min (cold start)");
  Serial.println("══════════════════════════════════════════════════════\n");
}


// ══════════════════════════════════════════════
//  LOOP
// ══════════════════════════════════════════════

unsigned long lastPrint = 0;
unsigned long lastBmpRead = 0;
unsigned long lastRtcRead = 0;
unsigned long lastGpsPrint = 0;
int rtcYear, rtcMonth, rtcDay, rtcHour, rtcMin, rtcSec;

void loop() {
  if (!mpuHealthy || !calibrated) { delay(1000); return; }

  // ── always: process incoming GPS bytes (non-blocking) ──
  processGPS();

  // ── always: try GPS→RTC sync ──
  syncRTCfromGPS();

  // ── MPU read + filter ──
  if (!readMPU6050()) { delay(10); return; }
  updateComplementaryFilter();

  // ── BMP280 every 2s ──
  if (bmpAvailable && (millis() - lastBmpRead >= 2000)) {
    lastBmpRead = millis();
    bmpReadData(&currentTemperature, &currentPressure);
  }

  // ── RTC every 1s ──
  if (millis() - lastRtcRead >= 1000) {
    lastRtcRead = millis();
    readRTC(rtcYear, rtcMonth, rtcDay, rtcHour, rtcMin, rtcSec);
  }

  // ── main output at 2 Hz ──
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();

    char timeStr[32];
    formatDateTime(currentUnixTime, timeStr, sizeof(timeStr));

    Serial.printf("θ%6.2f° | ", theta);

    if (bmpAvailable && currentPressure > 0)
      Serial.printf("%.1fhPa %.1f°C | ", currentPressure, currentTemperature);

    Serial.printf("%s [%s]", timeStr, ds1307Available ? "DS1307" : "SoftRTC");
    if (rtcTimeValid) Serial.print(" ✓");
    else Serial.print(" ⚠");

    // GPS compact status on same line
    if (gpsFixValid)
      Serial.printf(" | GPS:%.5f,%.5f sat:%d hdop:%.1f", gpsLat, gpsLon, gpsSatellites, gpsHdop);
    else if (gpsBytesReceived > 0)
      Serial.printf(" | GPS:SEARCHING sat:%d", gpsSatellites);
    else
      Serial.print(" | GPS:NO_DATA");

    Serial.println();
  }

  // ── detailed GPS status every 5 seconds ──
  if (millis() - lastGpsPrint >= 5000) {
    lastGpsPrint = millis();

    Serial.println("  ┌── GPS STATUS ──────────────────────────────────────────────┐");
    Serial.printf( "  │  Fix: %s   Sats: %d   HDOP: %.1f\n",
                   gpsFixValid ? "YES ✓" : "NO  ⚠", gpsSatellites, gpsHdop);

    if (gpsFixValid) {
      Serial.printf("  │  Lat: %.6f   Lon: %.6f   Alt: %.1fm\n", gpsLat, gpsLon, gpsAlt);
      Serial.printf("  │  Speed: %.1f km/h\n", gpsSpeed);
    }

    if (gpsTimeValid) {
      Serial.printf("  │  GPS time: %04d-%02d-%02d %02d:%02d:%02d UTC\n",
                    gpsYear, gpsMonth, gpsDay, gpsHour, gpsMin, gpsSec);
    }

    Serial.printf( "  │  Bytes: %lu  Sentences: %lu  GGA: %lu  RMC: %lu  ChkFail: %lu\n",
                   gpsBytesReceived, gpsTotalSentences, gpsGGACount, gpsRMCCount, gpsChecksumFail);
    Serial.printf( "  │  RTC synced from GPS: %s\n", gpsTimeSynced ? "YES ✓" : "NOT YET");
    Serial.println("  └────────────────────────────────────────────────────────────┘");
  }

  delay(8);
}