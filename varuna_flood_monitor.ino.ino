#include <Wire.h>
#include <HardwareSerial.h>

// ─── PIN DEFINITIONS ───
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
#define GPS_SYNC_INTERVAL_MS  86400000UL

// ─── SIM800L CONSTANTS ───
#define SIM_RESPONSE_BUF_SIZE  256
#define AT_SUCCESS   0
#define AT_TIMEOUT   1
#define AT_ERROR     2

// ─── I2C BUSES ───
TwoWire I2C_0 = TwoWire(0);
TwoWire I2C_1 = TwoWire(1);

// ─── UART ───
HardwareSerial GPS_Serial(1);
HardwareSerial SIM_Serial(2);

// ─── MPU6050 DATA ───
float ax, ay, az, gx, gy, gz, mpuTemp;
bool  mpuHealthy = false;

// ─── CALIBRATION ───
float gyroOffsetX=0,gyroOffsetY=0,gyroOffsetZ=0;
float refAccX=0,refAccY=0,refAccZ=0;
float refTiltX=0,refTiltY=0;
bool  calibrated = false;
#define GYRO_CAL_SAMPLES 1000
#define GYRO_CAL_DELAY_MS 2
#define ACCEL_CAL_SAMPLES 500
#define ACCEL_CAL_DELAY_MS 3
#define G_MIN_VALID 0.9
#define G_MAX_VALID 1.1

// ─── FILTER STATE ───
float filtTiltX=0,filtTiltY=0;
float correctedTiltX=0,correctedTiltY=0;
float theta=0;
unsigned long prevTime=0;
bool filterSeeded=false;

// ─── BMP280 DATA ───
bool bmpAvailable=false;
float currentPressure=0,currentTemperature=0;
uint16_t bmpDigT1;int16_t bmpDigT2,bmpDigT3;
uint16_t bmpDigP1;int16_t bmpDigP2,bmpDigP3,bmpDigP4,bmpDigP5;
int16_t bmpDigP6,bmpDigP7,bmpDigP8,bmpDigP9;
int32_t bmpTFine;

// ─── RTC DATA ───
bool ds1307Available=false;
bool rtcTimeValid=false;
uint32_t currentUnixTime=0;
uint32_t softRtcBaseUnix=0;
unsigned long softRtcBaseMillis=0;
bool softRtcSet=false;

// ─── GPS DATA ───
float gpsLat=0,gpsLon=0,gpsAlt=0,gpsSpeed=0,gpsHdop=99.9;
int gpsSatellites=0;
bool gpsFixValid=false;
int gpsHour=0,gpsMin=0,gpsSec=0;
int gpsDay=0,gpsMonth=0,gpsYear=0;
bool gpsTimeValid=false;
bool gpsTimeSynced=false;
unsigned long lastGpsSyncMillis=0;
char nmeaBuffer[NMEA_BUF_SIZE];
int nmeaIndex=0;
unsigned long gpsGGACount=0,gpsRMCCount=0,gpsChecksumFail=0;
unsigned long gpsTotalSentences=0,gpsBytesReceived=0;

// ─── SIM800L DATA ───
char simResponseBuffer[SIM_RESPONSE_BUF_SIZE];
bool simAvailable = false;
bool simRegistered = false;
int  simSignalRSSI = 0;
bool simCardReady = false;
int  simConsecutiveFails = 0;


// ══════════════════════════════════════════════
//  MPU6050 SECTION (compact — tested in Steps 1.2–1.4)
// ══════════════════════════════════════════════

void mpuWriteReg(uint8_t r,uint8_t v){I2C_0.beginTransmission(MPU6050_ADDR);I2C_0.write(r);I2C_0.write(v);I2C_0.endTransmission();}
uint8_t mpuReadReg(uint8_t r){I2C_0.beginTransmission(MPU6050_ADDR);I2C_0.write(r);I2C_0.endTransmission(false);I2C_0.requestFrom((uint8_t)MPU6050_ADDR,(uint8_t)1);return I2C_0.available()?I2C_0.read():0xFF;}

bool initMPU6050(){
  mpuWriteReg(REG_PWR_MGMT_1,0x00);delay(100);
  mpuWriteReg(REG_SMPLRT_DIV,0x07);mpuWriteReg(REG_CONFIG,0x03);
  mpuWriteReg(REG_GYRO_CONFIG,0x00);mpuWriteReg(REG_ACCEL_CONFIG,0x00);
  uint8_t w=mpuReadReg(REG_WHO_AM_I);mpuHealthy=(w==0x68||w==0x72);
  Serial.printf("  WHO_AM_I=0x%02X %s\n",w,mpuHealthy?"✓":"✗");return mpuHealthy;
}
bool readMPU6050(){
  I2C_0.beginTransmission(MPU6050_ADDR);I2C_0.write(REG_ACCEL_XOUT_H);
  if(I2C_0.endTransmission(false)!=0)return false;
  if(I2C_0.requestFrom((uint8_t)MPU6050_ADDR,(uint8_t)14)<14)return false;
  uint8_t b[14];for(int i=0;i<14;i++)b[i]=I2C_0.read();
  ax=((int16_t)((b[0]<<8)|b[1]))/ACCEL_SCALE;ay=((int16_t)((b[2]<<8)|b[3]))/ACCEL_SCALE;
  az=((int16_t)((b[4]<<8)|b[5]))/ACCEL_SCALE;
  gx=((int16_t)((b[8]<<8)|b[9]))/GYRO_SCALE;gy=((int16_t)((b[10]<<8)|b[11]))/GYRO_SCALE;
  gz=((int16_t)((b[12]<<8)|b[13]))/GYRO_SCALE;
  mpuTemp=((int16_t)((b[6]<<8)|b[7]))/340.0+36.53;return true;
}
bool calibrateGyro(){
  Serial.println("\n  ── GYRO CAL ──");float sx=0,sy=0,sz=0;int g=0;
  for(int i=0;i<GYRO_CAL_SAMPLES;i++){if(readMPU6050()){sx+=gx;sy+=gy;sz+=gz;g++;}delay(GYRO_CAL_DELAY_MS);}
  if(g<GYRO_CAL_SAMPLES/2)return false;
  gyroOffsetX=sx/g;gyroOffsetY=sy/g;gyroOffsetZ=sz/g;
  Serial.printf("  Offsets: X%+.4f Y%+.4f Z%+.4f\n",gyroOffsetX,gyroOffsetY,gyroOffsetZ);return true;
}
bool calibrateAccel(){
  Serial.println("  ── ACCEL CAL ──");float sx=0,sy=0,sz=0;int g=0;
  for(int i=0;i<ACCEL_CAL_SAMPLES;i++){if(readMPU6050()){float t=sqrt(ax*ax+ay*ay+az*az);
  if(t>=G_MIN_VALID&&t<=G_MAX_VALID){sx+=ax;sy+=ay;sz+=az;g++;}}delay(ACCEL_CAL_DELAY_MS);}
  if(g<ACCEL_CAL_SAMPLES/2)return false;
  refAccX=sx/g;refAccY=sy/g;refAccZ=sz/g;
  refTiltX=atan2(refAccY,sqrt(refAccX*refAccX+refAccZ*refAccZ))*180.0/PI;
  refTiltY=atan2(-refAccX,sqrt(refAccY*refAccY+refAccZ*refAccZ))*180.0/PI;
  Serial.printf("  Ref tilt: X%+.3f° Y%+.3f°\n",refTiltX,refTiltY);return true;
}
bool recalibrate(){
  Serial.println("\n┌──────────────────────────────────────┐");
  Serial.println("│       MPU6050 CALIBRATION            │");
  Serial.println("└──────────────────────────────────────┘");
  pinMode(STATUS_LED,OUTPUT);
  for(int i=0;i<6;i++){digitalWrite(STATUS_LED,HIGH);delay(250);digitalWrite(STATUS_LED,LOW);delay(250);}
  if(calibrateGyro()&&calibrateAccel()){calibrated=true;filterSeeded=false;Serial.println("  ✓ COMPLETE");return true;}
  calibrated=false;return false;
}
void updateComplementaryFilter(){
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
//  BMP280 SECTION (compact — tested in Step 1.5)
// ══════════════════════════════════════════════

void bmpWriteReg(uint8_t r,uint8_t v){I2C_1.beginTransmission(BMP280_ADDR);I2C_1.write(r);I2C_1.write(v);I2C_1.endTransmission();}
uint8_t bmpReadReg(uint8_t r){I2C_1.beginTransmission(BMP280_ADDR);I2C_1.write(r);I2C_1.endTransmission(false);I2C_1.requestFrom((uint8_t)BMP280_ADDR,(uint8_t)1);return I2C_1.available()?I2C_1.read():0xFF;}
void bmpReadBytes(uint8_t r,uint8_t*buf,uint8_t len){I2C_1.beginTransmission(BMP280_ADDR);I2C_1.write(r);I2C_1.endTransmission(false);I2C_1.requestFrom((uint8_t)BMP280_ADDR,len);for(uint8_t i=0;i<len&&I2C_1.available();i++)buf[i]=I2C_1.read();}

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
  *temperature=bmpCompensateTemp(adcT)/100.0;*pressure=bmpCompensatePress(adcP)/256.0/100.0;return true;
}


// ══════════════════════════════════════════════
//  RTC SECTION (compact — tested in Step 1.6)
// ══════════════════════════════════════════════

uint8_t bcdToDec(uint8_t v){return((v>>4)*10)+(v&0x0F);}
uint8_t decToBcd(uint8_t v){return((v/10)<<4)|(v%10);}

uint32_t dateToUnix(int yr,int mo,int dy,int hr,int mi,int sc){
  uint32_t days=0;for(int y=1970;y<yr;y++){bool l=(y%4==0&&(y%100!=0||y%400==0));days+=l?366:365;}
  static const int md[]={0,31,28,31,30,31,30,31,31,30,31,30,31};
  for(int m=1;m<mo;m++){days+=md[m];if(m==2&&(yr%4==0&&(yr%100!=0||yr%400==0)))days++;}
  days+=(dy-1);return days*86400UL+(uint32_t)hr*3600UL+(uint32_t)mi*60UL+(uint32_t)sc;
}
void unixToDate(uint32_t ut,int&y,int&mo,int&d,int&h,int&mi,int&s){
  s=ut%60;ut/=60;mi=ut%60;ut/=60;h=ut%24;ut/=24;uint32_t td=ut;y=1970;
  while(true){bool l=(y%4==0&&(y%100!=0||y%400==0));uint16_t dy=l?366:365;if(td<dy)break;td-=dy;y++;}
  static const int md[]={0,31,28,31,30,31,30,31,31,30,31,30,31};
  bool l=(y%4==0&&(y%100!=0||y%400==0));mo=1;
  while(mo<=12){int dm=md[mo];if(mo==2&&l)dm=29;if(td<(uint32_t)dm)break;td-=dm;mo++;}d=td+1;
}
void formatDateTime(uint32_t ut,char*buf,size_t len){
  int y,mo,d,h,mi,s;unixToDate(ut,y,mo,d,h,mi,s);snprintf(buf,len,"%04d-%02d-%02d %02d:%02d:%02d",y,mo,d,h,mi,s);
}

// DS1307 stubs (for when hardware is connected)
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
  if(ds1307Probe()){Serial.println("  DS1307 detected");
    if(ds1307Init()){ds1307Available=true;int y,mo,d,h,mi,s;
    if(ds1307Read(y,mo,d,h,mi,s)&&y>=2024){currentUnixTime=dateToUnix(y,mo,d,h,mi,s);rtcTimeValid=true;
    softRtcWrite(y,mo,d,h,mi,s);Serial.println("  ✓ DS1307 active");return true;}}}
  ds1307Available=false;softRtcWrite(2025,1,1,0,0,0);rtcTimeValid=false;
  Serial.println("  DS1307 not found → Software RTC");
  Serial.println("  ✓ Software RTC active");return true;
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
  if(softRtcSet)return softRtcRead();return millis()/1000;
}


// ══════════════════════════════════════════════
//  GPS SECTION (compact — tested in Step 1.7)
// ══════════════════════════════════════════════

float nmeaToDecimal(float raw,char dir){int deg=(int)(raw/100);float min=raw-(deg*100.0);float dec=deg+min/60.0;if(dir=='S'||dir=='W')dec=-dec;return dec;}

bool nmeaGetField(const char*s,int fi,char*out,int ol){
  int cf=0,i=0,len=strlen(s);
  while(i<len&&cf<fi){if(s[i]==',')cf++;i++;}
  if(cf!=fi){out[0]='\0';return false;}
  int j=0;while(i<len&&s[i]!=','&&s[i]!='*'&&j<ol-1){out[j++]=s[i++];}out[j]='\0';return(j>0);
}
bool validateNMEAChecksum(const char*s){
  const char*st=strchr(s,'$');const char*star=strchr(s,'*');
  if(!st||!star||star<=st+1)return false;
  uint8_t comp=0;for(const char*p=st+1;p<star;p++)comp^=(uint8_t)*p;
  char hs[3]={star[1],star[2],'\0'};return(comp==(uint8_t)strtoul(hs,NULL,16));
}
void parseGGA(const char*s){
  char f[20];char d[4];
  if(nmeaGetField(s,1,f,sizeof(f))&&strlen(f)>=6){gpsHour=(f[0]-'0')*10+(f[1]-'0');gpsMin=(f[2]-'0')*10+(f[3]-'0');gpsSec=(f[4]-'0')*10+(f[5]-'0');}
  if(nmeaGetField(s,6,f,sizeof(f))){gpsFixValid=(atoi(f)>=1);}else gpsFixValid=false;
  if(!gpsFixValid)return;
  if(nmeaGetField(s,2,f,sizeof(f))&&nmeaGetField(s,3,d,sizeof(d)))gpsLat=nmeaToDecimal(atof(f),d[0]);
  if(nmeaGetField(s,4,f,sizeof(f))&&nmeaGetField(s,5,d,sizeof(d)))gpsLon=nmeaToDecimal(atof(f),d[0]);
  if(nmeaGetField(s,7,f,sizeof(f)))gpsSatellites=atoi(f);
  if(nmeaGetField(s,8,f,sizeof(f)))gpsHdop=atof(f);
  if(nmeaGetField(s,9,f,sizeof(f)))gpsAlt=atof(f);
  gpsGGACount++;
}
void parseRMC(const char*s){
  char f[20];char d[4];
  if(nmeaGetField(s,1,f,sizeof(f))&&strlen(f)>=6){gpsHour=(f[0]-'0')*10+(f[1]-'0');gpsMin=(f[2]-'0')*10+(f[3]-'0');gpsSec=(f[4]-'0')*10+(f[5]-'0');}
  if(nmeaGetField(s,2,f,sizeof(f))){gpsFixValid=(f[0]=='A');}
  if(!gpsFixValid)return;
  if(nmeaGetField(s,3,f,sizeof(f))&&nmeaGetField(s,4,d,sizeof(d)))gpsLat=nmeaToDecimal(atof(f),d[0]);
  if(nmeaGetField(s,5,f,sizeof(f))&&nmeaGetField(s,6,d,sizeof(d)))gpsLon=nmeaToDecimal(atof(f),d[0]);
  if(nmeaGetField(s,7,f,sizeof(f)))gpsSpeed=atof(f)*1.852;
  if(nmeaGetField(s,9,f,sizeof(f))&&strlen(f)>=6){gpsDay=(f[0]-'0')*10+(f[1]-'0');gpsMonth=(f[2]-'0')*10+(f[3]-'0');gpsYear=2000+(f[4]-'0')*10+(f[5]-'0');gpsTimeValid=(gpsYear>=2024);}
  gpsRMCCount++;
}
void parseNMEA(const char*s){
  gpsTotalSentences++;
  if(strncmp(s+1,"GPGGA",5)==0||strncmp(s+1,"GNGGA",5)==0)parseGGA(s);
  else if(strncmp(s+1,"GPRMC",5)==0||strncmp(s+1,"GNRMC",5)==0)parseRMC(s);
}
void processGPS(){
  while(GPS_Serial.available()){char c=GPS_Serial.read();gpsBytesReceived++;
  if(c=='\n'||c=='\r'){if(nmeaIndex>0){nmeaBuffer[nmeaIndex]='\0';
  if(nmeaBuffer[0]=='$'){if(validateNMEAChecksum(nmeaBuffer))parseNMEA(nmeaBuffer);else gpsChecksumFail++;}nmeaIndex=0;}}
  else{if(nmeaIndex<NMEA_BUF_SIZE-1)nmeaBuffer[nmeaIndex++]=c;else nmeaIndex=0;}}
}
void syncRTCfromGPS(){
  if(!gpsFixValid||!gpsTimeValid||gpsYear<2024||gpsHdop>5.0||gpsSatellites<4)return;
  if(gpsTimeSynced&&(millis()-lastGpsSyncMillis<GPS_SYNC_INTERVAL_MS))return;
  Serial.printf("\n  GPS→RTC SYNC: %04d-%02d-%02d %02d:%02d:%02d UTC sats:%d hdop:%.1f\n",gpsYear,gpsMonth,gpsDay,gpsHour,gpsMin,gpsSec,gpsSatellites,gpsHdop);
  writeRTC(gpsYear,gpsMonth,gpsDay,gpsHour,gpsMin,gpsSec);
  gpsTimeSynced=true;lastGpsSyncMillis=millis();Serial.println("  ✓ RTC synced from GPS");
}
void initGPS(){
  Serial.println("\n── GPS INIT ──");
  GPS_Serial.begin(9600,SERIAL_8N1,GPS_RX,GPS_TX);nmeaIndex=0;
  Serial.println("  UART1 at 9600 baud (RX=GPIO6 TX=GPIO7)");
  Serial.println("  ✓ GPS UART active");
}


// ══════════════════════════════════════════════
//  SIM800L SECTION
// ══════════════════════════════════════════════

// ──────────────────────────────────────────────
//  AT COMMAND ENGINE
//
//  Sends a command, waits for expected response
//  or ERROR or timeout. Returns status code.
//
//  The response is stored in simResponseBuffer
//  for the caller to inspect if needed.
// ──────────────────────────────────────────────

int send_at_command(const char *command, const char *expected, unsigned long timeout_ms) {
  // clear buffer
  memset(simResponseBuffer, 0, SIM_RESPONSE_BUF_SIZE);
  int bufIndex = 0;

  // flush any pending input
  while (SIM_Serial.available()) SIM_Serial.read();

  // send command
  SIM_Serial.print(command);
  SIM_Serial.print("\r\n");

  // log what we sent
  Serial.printf("    TX: %s\n", command);

  // wait for response
  unsigned long start = millis();

  while (millis() - start < timeout_ms) {
    while (SIM_Serial.available()) {
      char c = SIM_Serial.read();

      // store in buffer (leave room for null terminator)
      if (bufIndex < SIM_RESPONSE_BUF_SIZE - 1) {
        simResponseBuffer[bufIndex++] = c;
        simResponseBuffer[bufIndex] = '\0';
      }

      // check for expected response
      if (expected && strstr(simResponseBuffer, expected)) {
        // print response (clean up newlines for readability)
        Serial.printf("    RX: ");
        for (int i = 0; i < bufIndex; i++) {
          char ch = simResponseBuffer[i];
          if (ch == '\r') continue;
          if (ch == '\n') { Serial.print(" | "); continue; }
          Serial.print(ch);
        }
        Serial.println("  ✓");
        return AT_SUCCESS;
      }

      // check for error
      if (strstr(simResponseBuffer, "ERROR")) {
        Serial.printf("    RX: ");
        for (int i = 0; i < bufIndex; i++) {
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

  // timeout
  Serial.printf("    RX: ");
  if (bufIndex > 0) {
    for (int i = 0; i < bufIndex; i++) {
      char ch = simResponseBuffer[i];
      if (ch == '\r') continue;
      if (ch == '\n') { Serial.print(" | "); continue; }
      Serial.print(ch);
    }
  } else {
    Serial.print("(no response)");
  }
  Serial.println("  ✗ TIMEOUT");
  return AT_TIMEOUT;
}

// ──────────────────────────────────────────────
//  HARDWARE RESET
//  Pull RST low for 200ms, release, wait for boot
// ──────────────────────────────────────────────

void simHardwareReset() {
  Serial.println("  Pulling RST LOW for 200ms...");
  pinMode(SIM_RST, OUTPUT);
  digitalWrite(SIM_RST, LOW);
  delay(200);
  digitalWrite(SIM_RST, HIGH);
  Serial.println("  RST released — waiting 3s for boot...");
  delay(3000);
}

// ──────────────────────────────────────────────
//  PARSE SIGNAL STRENGTH from +CSQ response
//  Response format: +CSQ: <rssi>,<ber>
//  RSSI: 0-31 (usable: 10-31), 99 = not known
// ──────────────────────────────────────────────

int parseSignalRSSI(const char *response) {
  const char *p = strstr(response, "+CSQ:");
  if (!p) return 0;
  p += 5;  // skip "+CSQ:"
  while (*p == ' ') p++;  // skip spaces
  return atoi(p);
}

// ──────────────────────────────────────────────
//  PARSE REGISTRATION STATUS from +CREG response
//  Response format: +CREG: <n>,<stat>
//  stat: 0=not registered, 1=home, 2=searching,
//        3=denied, 5=roaming
// ──────────────────────────────────────────────

int parseRegStatus(const char *response) {
  const char *p = strstr(response, "+CREG:");
  if (!p) return -1;
  p += 6;  // skip "+CREG:"
  while (*p == ' ') p++;
  // skip first number and comma: <n>,<stat>
  while (*p && *p != ',') p++;
  if (*p == ',') p++;
  return atoi(p);
}

// ──────────────────────────────────────────────
//  FULL INITIALIZATION SEQUENCE
// ──────────────────────────────────────────────

bool initSIM800L() {
  Serial.println("\n── SIM800L INIT ──");

  // step 1: hardware reset
  simHardwareReset();

  // step 2: basic AT — try up to 5 times
  Serial.println("\n  Step 1/6: Basic AT communication");
  bool atOk = false;
  for (int attempt = 1; attempt <= 5; attempt++) {
    Serial.printf("  Attempt %d/5:\n", attempt);
    if (send_at_command("AT", "OK", 2000) == AT_SUCCESS) {
      atOk = true;
      break;
    }
    delay(1000);
  }
  if (!atOk) {
    Serial.println("  ✗ SIM800L not responding after 5 attempts");
    Serial.println("    Check: power supply (needs 4.0V, 2A peak)");
    Serial.println("    Check: TX/RX wiring (TX→GPIO15, RX→GPIO16)");
    Serial.println("    Check: RST wiring (GPIO17)");
    simAvailable = false;
    return false;
  }
  simAvailable = true;

  // step 3: disable echo
  Serial.println("\n  Step 2/6: Disable echo");
  send_at_command("ATE0", "OK", 1000);

  // step 4: SMS text mode (for diagnostic channel)
  Serial.println("\n  Step 3/6: SMS text mode");
  if (send_at_command("AT+CMGF=1", "OK", 1000) != AT_SUCCESS) {
    Serial.println("  ⚠ SMS mode failed — non-critical, continuing");
  }

  // step 5: check SIM card
  Serial.println("\n  Step 4/6: SIM card check");
  if (send_at_command("AT+CPIN?", "READY", 5000) == AT_SUCCESS) {
    simCardReady = true;
    Serial.println("  SIM card: READY");
  } else {
    simCardReady = false;
    Serial.println("  ✗ SIM card not ready");
    Serial.println("    Check: SIM card inserted correctly");
    Serial.println("    Check: SIM card not PIN-locked");
    return false;
  }

  // step 6: network registration (try up to 5 times with longer waits)
  Serial.println("\n  Step 5/6: Network registration");
  simRegistered = false;
  for (int attempt = 1; attempt <= 5; attempt++) {
    Serial.printf("  Check %d/5:\n", attempt);
    int result = send_at_command("AT+CREG?", "+CREG:", 5000);
    if (result == AT_SUCCESS) {
      int regStatus = parseRegStatus(simResponseBuffer);
      Serial.printf("    Registration status: %d", regStatus);
      if (regStatus == 1) {
        Serial.println(" (home network) ✓");
        simRegistered = true;
        break;
      } else if (regStatus == 5) {
        Serial.println(" (roaming) ✓");
        simRegistered = true;
        break;
      } else if (regStatus == 2) {
        Serial.println(" (searching...)");
      } else if (regStatus == 0) {
        Serial.println(" (not registered, not searching)");
      } else if (regStatus == 3) {
        Serial.println(" (registration denied!)");
        break;  // no point retrying
      } else {
        Serial.printf(" (unknown: %d)\n", regStatus);
      }
    }
    if (attempt < 5) {
      Serial.println("    Waiting 3s before retry...");
      delay(3000);
    }
  }

  if (!simRegistered) {
    Serial.println("  ✗ Not registered on network");
    Serial.println("    May still register later — continuing");
  }

  // step 7: signal strength
  Serial.println("\n  Step 6/6: Signal strength");
  if (send_at_command("AT+CSQ", "+CSQ:", 2000) == AT_SUCCESS) {
    simSignalRSSI = parseSignalRSSI(simResponseBuffer);
    Serial.printf("    RSSI: %d", simSignalRSSI);
    if (simSignalRSSI == 99)      Serial.println(" (not known)");
    else if (simSignalRSSI >= 20) Serial.println(" (excellent)");
    else if (simSignalRSSI >= 15) Serial.println(" (good)");
    else if (simSignalRSSI >= 10) Serial.println(" (fair)");
    else if (simSignalRSSI >= 5)  Serial.println(" (weak)");
    else                          Serial.println(" (very weak — may have issues)");

    // approximate dBm: dBm = -113 + (2 × RSSI)
    if (simSignalRSSI < 99) {
      int dbm = -113 + (2 * simSignalRSSI);
      Serial.printf("    Approx: %d dBm\n", dbm);
    }
  }

  // configure SMS direct delivery (for diagnostic inbound)
  Serial.println("\n  Configuring SMS delivery...");
  send_at_command("AT+CSCS=\"GSM\"", "OK", 1000);
  send_at_command("AT+CNMI=2,2,0,0,0", "OK", 1000);

  // clear any stored messages
  send_at_command("AT+CMGDA=\"DEL ALL\"", "OK", 5000);

  // summary
  Serial.println("\n  ┌── SIM800L STATUS ────────────────────────┐");
  Serial.printf( "  │  Module:     %s\n", simAvailable ? "DETECTED ✓" : "NOT FOUND ✗");
  Serial.printf( "  │  SIM card:   %s\n", simCardReady ? "READY ✓" : "NOT READY ✗");
  Serial.printf( "  │  Registered: %s\n", simRegistered ? "YES ✓" : "NO ⚠");
  Serial.printf( "  │  Signal:     %d/31 RSSI\n", simSignalRSSI);
  Serial.println("  └───────────────────────────────────────────┘");

  if (simAvailable && simCardReady) {
    Serial.println("  ✓ SIM800L READY");
    return true;
  }

  Serial.println("  ⚠ SIM800L partially initialised");
  return false;
}

// ──────────────────────────────────────────────
//  PERIODIC CHECKS (called from loop)
// ──────────────────────────────────────────────

void checkSimSignal() {
  if (!simAvailable) return;
  if (send_at_command("AT+CSQ", "+CSQ:", 2000) == AT_SUCCESS) {
    simSignalRSSI = parseSignalRSSI(simResponseBuffer);
  }
}

void checkSimRegistration() {
  if (!simAvailable) return;
  if (send_at_command("AT+CREG?", "+CREG:", 3000) == AT_SUCCESS) {
    int reg = parseRegStatus(simResponseBuffer);
    simRegistered = (reg == 1 || reg == 5);
  }
}


// ══════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("┌──────────────────────────────────────────────────┐");
  Serial.println("│   VARUNA — Step 1.8: SIM800L Communication      │");
  Serial.println("└──────────────────────────────────────────────────┘");

  I2C_0.begin(SDA_0, SCL_0, 100000);
  I2C_1.begin(SDA_1, SCL_1, 100000);
  Serial.println("\nI2C buses initialised");

  // ── MPU6050 ──
  Serial.println("\n── MPU6050 INIT ──");
  if (!initMPU6050()) { Serial.println("HALT"); while(1) delay(1000); }
  recalibrate();

  // ── BMP280 ──
  initBMP280();

  // ── RTC ──
  initRTC();

  // ── GPS ──
  initGPS();

  // ── SIM800L ──
  SIM_Serial.begin(9600, SERIAL_8N1, SIM_RX, SIM_TX);
  Serial.println("\n  SIM UART2 at 9600 baud (RX=GPIO15 TX=GPIO16 RST=GPIO17)");
  initSIM800L();

  prevTime = millis();

  Serial.println("\n══════════════════════════════════════════════════════");
  Serial.println("  LIVE OUTPUT — all sensors at 2 Hz");
  Serial.println("  SIM status checks every 60s (signal) / 30s (reg)");
  Serial.println("══════════════════════════════════════════════════════\n");
}


// ══════════════════════════════════════════════
//  LOOP
// ══════════════════════════════════════════════

unsigned long lastPrint = 0;
unsigned long lastBmpRead = 0;
unsigned long lastRtcRead = 0;
unsigned long lastGpsPrint = 0;
unsigned long lastSimSignal = 0;
unsigned long lastSimReg = 0;

int rtcYear, rtcMonth, rtcDay, rtcHour, rtcMin, rtcSec;

void loop() {
  if (!mpuHealthy || !calibrated) { delay(1000); return; }

  // ── GPS (non-blocking) ──
  processGPS();
  syncRTCfromGPS();

  // ── MPU + filter ──
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

  // ── SIM signal check every 60s ──
  if (simAvailable && (millis() - lastSimSignal >= 60000)) {
    lastSimSignal = millis();
    checkSimSignal();
  }

  // ── SIM registration check every 30s ──
  if (simAvailable && (millis() - lastSimReg >= 30000)) {
    lastSimReg = millis();
    checkSimRegistration();
  }

  // ── main output at 2 Hz ──
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();

    char timeStr[32];
    formatDateTime(currentUnixTime, timeStr, sizeof(timeStr));

    Serial.printf("θ%6.2f° | ", theta);

    if (bmpAvailable && currentPressure > 0)
      Serial.printf("%.1fhPa %.1f°C | ", currentPressure, currentTemperature);

    Serial.printf("%s [%s]%s", timeStr,
                  ds1307Available ? "DS1307" : "SoftRTC",
                  rtcTimeValid ? "✓" : "⚠");

    // GPS compact
    if (gpsFixValid)
      Serial.printf(" | GPS:%.4f,%.4f s:%d", gpsLat, gpsLon, gpsSatellites);
    else if (gpsBytesReceived > 0)
      Serial.printf(" | GPS:SRCH s:%d", gpsSatellites);
    else
      Serial.print(" | GPS:--");

    // SIM compact
    if (simAvailable)
      Serial.printf(" | SIM:%s rssi:%d", simRegistered ? "REG" : "NOREG", simSignalRSSI);
    else
      Serial.print(" | SIM:OFF");

    Serial.println();
  }

  // ── detailed GPS status every 10s ──
  if (millis() - lastGpsPrint >= 10000) {
    lastGpsPrint = millis();
    Serial.println("  ┌── GPS ──────────────────────────────────────────┐");
    Serial.printf( "  │  Fix:%s Sats:%d HDOP:%.1f",gpsFixValid?"YES":"NO ",gpsSatellites,gpsHdop);
    if(gpsFixValid)Serial.printf(" Lat:%.6f Lon:%.6f Alt:%.0fm",gpsLat,gpsLon,gpsAlt);
    Serial.printf("\n  │  Bytes:%lu Sent:%lu GGA:%lu RMC:%lu Fail:%lu Synced:%s\n",
                  gpsBytesReceived,gpsTotalSentences,gpsGGACount,gpsRMCCount,gpsChecksumFail,
                  gpsTimeSynced?"YES":"NO");
    Serial.println("  └────────────────────────────────────────────────┘");
  }

  delay(8);
}