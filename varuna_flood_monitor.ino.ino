/*
 * ═══════════════════════════════════════════════════════════════════════════
 * VARUNA FLOOD MONITOR - ESP32-S3 Based Flood Detection System
 * ═══════════════════════════════════════════════════════════════════════════
 * Phase 1.1: I2C Bus Setup and Scanner
 * 
 * Hardware Connections:
 *   I2C Bus 0 (MPU6050):
 *     - SDA: GPIO 8
 *     - SCL: GPIO 9
 *   
 *   I2C Bus 1 (DS1307 RTC + BMP280):
 *     - SDA: GPIO 4
 *     - SCL: GPIO 5
 * ═══════════════════════════════════════════════════════════════════════════
 */

#include <Wire.h>
#include <HardwareSerial.h>
#include <EEPROM.h>
#include <SPIFFS.h>
#include <esp_task_wdt.h>
#include <esp_sleep.h>

// ═══════════════════════════════════════════════════════════════════════════
// PIN DEFINITIONS
// ═══════════════════════════════════════════════════════════════════════════

// I2C Bus 0 - MPU6050 (Accelerometer/Gyroscope)
#define SDA_0  8
#define SCL_0  9

// I2C Bus 1 - RTC (DS1307) + Barometer (BMP280)
#define SDA_1  4
#define SCL_1  5

// GPS Module (Serial)
#define GPS_RX 6
#define GPS_TX 7

// SIM800L GSM Module (Serial)
#define SIM_RX  15
#define SIM_TX  16
#define SIM_RST 17

// Analog & Digital I/O
#define BATTERY_PIN  2   // ADC for battery voltage monitoring
#define STATUS_LED   3   // System status indicator
#define ALGO_BUTTON  12  // Algorithm selection button
#define ALGO_LED     13  // Algorithm status LED
#define C3_FEED_PIN  14  // Watchdog feed to external ESP32-C3

// ═══════════════════════════════════════════════════════════════════════════
// CONSTANTS
// ═══════════════════════════════════════════════════════════════════════════

#define ALPHA           0.98    // Complementary filter coefficient
#define WDT_TIMEOUT_SEC 120     // Watchdog timeout in seconds
#define I2C_CLOCK_SPEED 100000  // 100kHz I2C clock

// Known I2C Addresses
#define MPU6050_ADDR    0x68    // MPU6050 (can also be 0x69 if AD0 is HIGH)
#define DS1307_ADDR     0x68    // DS1307 RTC
#define BMP280_ADDR     0x76    // BMP280 (can also be 0x77)

// ═══════════════════════════════════════════════════════════════════════════
// GLOBAL OBJECTS
// ═══════════════════════════════════════════════════════════════════════════

// Two independent I2C buses
TwoWire I2C_0 = TwoWire(0);  // Bus 0: MPU6050
TwoWire I2C_1 = TwoWire(1);  // Bus 1: RTC + BMP280

// ═══════════════════════════════════════════════════════════════════════════
// FUNCTION DECLARATIONS
// ═══════════════════════════════════════════════════════════════════════════

void initI2CBuses();
void scanBus(TwoWire &bus, const char* name);
void scanAllBuses();
void printDeviceName(uint8_t addr, uint8_t busNum);
void blinkLED(int times, int delayMs);

// ═══════════════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Wait for serial port to connect (needed for native USB)
  delay(2000);
  
  // Print boot header
  Serial.println();
  Serial.println("═══════════════════════════════════════════════════════════");
  Serial.println("           VARUNA FLOOD MONITOR - SYSTEM BOOT              ");
  Serial.println("═══════════════════════════════════════════════════════════");
  Serial.println();
  
  // Initialize status LED
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);
  
  // Visual boot indicator
  blinkLED(3, 100);
  
  // Initialize I2C buses
  initI2CBuses();
  
  // Scan all I2C buses for connected devices
  scanAllBuses();
  
  // Boot complete indicator
  Serial.println();
  Serial.println("═══════════════════════════════════════════════════════════");
  Serial.println("                    BOOT COMPLETE                          ");
  Serial.println("═══════════════════════════════════════════════════════════");
  Serial.println();
  
  // Solid LED indicates successful boot
  digitalWrite(STATUS_LED, HIGH);
}

// ═══════════════════════════════════════════════════════════════════════════
// MAIN LOOP
// ═══════════════════════════════════════════════════════════════════════════

void loop() {
  // For now, just re-scan buses every 10 seconds for testing
  static unsigned long lastScan = 0;
  
  if (millis() - lastScan > 10000) {
    lastScan = millis();
    Serial.println("\n[PERIODIC SCAN]");
    scanAllBuses();
  }
  
  // Heartbeat blink
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink > 1000) {
    lastBlink = millis();
    digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
  }
  
  delay(100);
}

// ═══════════════════════════════════════════════════════════════════════════
// I2C INITIALIZATION
// ═══════════════════════════════════════════════════════════════════════════

void initI2CBuses() {
  Serial.println("[I2C] Initializing I2C buses...");
  
  // Initialize Bus 0 (MPU6050)
  Serial.printf("[I2C] Bus 0: SDA=%d, SCL=%d, Speed=%dHz\n", 
                SDA_0, SCL_0, I2C_CLOCK_SPEED);
  
  if (I2C_0.begin(SDA_0, SCL_0, I2C_CLOCK_SPEED)) {
    Serial.println("[I2C] Bus 0: Initialized successfully");
  } else {
    Serial.println("[I2C] Bus 0: INITIALIZATION FAILED!");
  }
  
  // Initialize Bus 1 (RTC + BMP280)
  Serial.printf("[I2C] Bus 1: SDA=%d, SCL=%d, Speed=%dHz\n", 
                SDA_1, SCL_1, I2C_CLOCK_SPEED);
  
  if (I2C_1.begin(SDA_1, SCL_1, I2C_CLOCK_SPEED)) {
    Serial.println("[I2C] Bus 1: Initialized successfully");
  } else {
    Serial.println("[I2C] Bus 1: INITIALIZATION FAILED!");
  }
  
  Serial.println("[I2C] Bus initialization complete");
  Serial.println();
}

// ═══════════════════════════════════════════════════════════════════════════
// I2C BUS SCANNER
// ═══════════════════════════════════════════════════════════════════════════

void scanBus(TwoWire &bus, const char* name) {
  Serial.printf("Scanning %s...\n", name);
  
  int deviceCount = 0;
  
  for (uint8_t addr = 1; addr < 127; addr++) {
    bus.beginTransmission(addr);
    uint8_t error = bus.endTransmission();
    
    if (error == 0) {
      Serial.printf("  ✓ Found device at 0x%02X", addr);
      
      // Print known device name
      if (strcmp(name, "Bus 0") == 0) {
        printDeviceName(addr, 0);
      } else {
        printDeviceName(addr, 1);
      }
      
      Serial.println();
      deviceCount++;
    } else if (error == 4) {
      Serial.printf("  ✗ Unknown error at 0x%02X\n", addr);
    }
  }
  
  if (deviceCount == 0) {
    Serial.println("  No devices found!");
  } else {
    Serial.printf("  Total: %d device(s) found\n", deviceCount);
  }
}

void scanAllBuses() {
  Serial.println("┌─────────────────────────────────────────┐");
  Serial.println("│           I2C BUS SCAN RESULTS          │");
  Serial.println("└─────────────────────────────────────────┘");
  
  scanBus(I2C_0, "Bus 0");
  Serial.println();
  scanBus(I2C_1, "Bus 1");
}

void printDeviceName(uint8_t addr, uint8_t busNum) {
  // Identify common devices based on address and bus number
  switch (addr) {
    case 0x68:
      if (busNum == 0) {
        Serial.print(" -> MPU6050 (Accel/Gyro)");
      } else {
        Serial.print(" -> DS1307 (RTC)");
      }
      break;
    
    case 0x69:
      Serial.print(" -> MPU6050 (AD0=HIGH)");
      break;
    
    case 0x76:
      Serial.print(" -> BMP280 (Barometer)");
      break;
    
    case 0x77:
      Serial.print(" -> BMP280/BMP180 (Alt Addr)");
      break;
    
    case 0x50:
      Serial.print(" -> AT24C32 (EEPROM on RTC)");
      break;
    
    case 0x57:
      Serial.print(" -> AT24C32 (Alt EEPROM)");
      break;
    
    case 0x3C:
    case 0x3D:
      Serial.print(" -> OLED Display");
      break;
    
    default:
      Serial.print(" -> Unknown Device");
      break;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// UTILITY FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

void blinkLED(int times, int delayMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(STATUS_LED, HIGH);
    delay(delayMs);
    digitalWrite(STATUS_LED, LOW);
    delay(delayMs);
  }
}