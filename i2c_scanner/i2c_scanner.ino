/*
 * I2C Scanner for ESP32-S3
 * Scans I2C bus to find connected devices
 * Helps diagnose wiring issues
 */

#include <Wire.h>

// I2C Pin Definitions (match your wiring)
const int SDA_PIN = 8;   // Your SDA pin
const int SCL_PIN = 9;   // Your SCL pin

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n\n========================================");
  Serial.println("I2C Scanner for ESP32-S3");
  Serial.println("========================================");
  Serial.print("SDA Pin: GPIO ");
  Serial.println(SDA_PIN);
  Serial.print("SCL Pin: GPIO ");
  Serial.println(SCL_PIN);
  Serial.println("\nScanning I2C bus...");
  Serial.println("========================================\n");
  
  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000); // 100kHz I2C speed
  
  delay(500);
}

void loop() {
  byte error, address;
  int nDevices = 0;
  
  Serial.println("Scanning...");
  
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      Serial.print(" (");
      Serial.print(address);
      Serial.println(")");
      
      // Check if it's the expected AS5600 address
      if (address == 0x36) {
        Serial.println("  ✓ This is the AS5600 address!");
      }
      
      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }
  
  if (nDevices == 0) {
    Serial.println("\n❌ No I2C devices found!");
    Serial.println("\nTroubleshooting:");
    Serial.println("1. Check wiring:");
    Serial.println("   - VCC -> 3.3V (NOT 5V!)");
    Serial.println("   - GND -> GND");
    Serial.println("   - SDA -> GPIO 8");
    Serial.println("   - SCL -> GPIO 9");
    Serial.println("2. Verify sensor is powered (LED on sensor?)");
    Serial.println("3. Check if sensor needs pull-up resistors");
    Serial.println("4. Try swapping SDA and SCL");
    Serial.println("5. Verify you have AS5600 (I2C), not AS500 (SPI)");
  } else {
    Serial.print("\n✓ Found ");
    Serial.print(nDevices);
    Serial.println(" device(s)");
  }
  
  Serial.println("\n========================================\n");
  delay(5000); // Scan every 5 seconds
}


