/*
 * AS5600 Magnetic Encoder Test (I2C Version)
 * Tests I2C communication with AS5600 sensor
 * Displays angle readings and magnet strength
 * 
 * AS5600 Connections (I2C mode - default):
 * - VCC -> 3.3V
 * - GND -> GND
 * - SDA -> GPIO 21 (ESP32-S3 default I2C SDA)
 * - SCL -> GPIO 22 (ESP32-S3 default I2C SCL)
 * 
 * Note: ESP32-S3 default I2C pins may vary by board
 * Adjust Wire.begin() pins if needed
 */

#include <Wire.h>

// AS5600 I2C Address
#define AS5600_I2C_ADDR 0x36

// AS5600 Register Addresses
#define AS5600_REG_STATUS 0x0B
#define AS5600_REG_RAW_ANGLE 0x0C  // 12-bit raw angle (0-4095)
#define AS5600_REG_ANGLE 0x0E      // Filtered angle
#define AS5600_REG_MAGNITUDE 0x1B  // Magnet strength

// I2C Pin Definitions
const int SDA_PIN = 8;   // Your SDA pin
const int SCL_PIN = 9;   // Your SCL pin

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n\n========================================");
  Serial.println("AS5600 Magnetic Encoder Test (I2C)");
  Serial.println("========================================");
  
  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000); // 100kHz I2C speed
  
  Serial.println("I2C initialized");
  Serial.print("SDA Pin: "); Serial.println(SDA_PIN);
  Serial.print("SCL Pin: "); Serial.println(SCL_PIN);
  Serial.print("I2C Address: 0x"); Serial.println(AS5600_I2C_ADDR, HEX);
  Serial.println("\nReading AS5600 sensor...");
  Serial.println("Rotate the magnet to see angle changes");
  Serial.println("========================================\n");
  
  // Test initial read
  delay(500);
  testConnection();
}

void loop() {
  // Read and display sensor data
  uint16_t rawAngle = readRawAngle();
  uint16_t angle = readAngle();
  uint16_t magnitude = readMagnitude();
  uint8_t status = readStatus();
  
  // Calculate angle in degrees (0-360)
  float angleDegrees = (rawAngle / 4096.0) * 360.0;
  
  // Display readings
  Serial.print("Raw Angle: ");
  Serial.print(rawAngle);
  Serial.print(" (0-4095) | ");
  
  Serial.print("Angle: ");
  Serial.print(angle);
  Serial.print(" (0-4095) | ");
  
  Serial.print("Degrees: ");
  Serial.print(angleDegrees, 1);
  Serial.print("° | ");
  
  Serial.print("Magnitude: ");
  Serial.print(magnitude);
  Serial.print(" | ");
  
  // Status bits
  Serial.print("Status: 0x");
  Serial.print(status, HEX);
  
  // Check magnet detection
  if (magnitude < 100) {
    Serial.print(" [WARNING: Magnet too weak or too far!]");
  } else if (magnitude > 3000) {
    Serial.print(" [WARNING: Magnet too strong or too close!]");
  } else {
    Serial.print(" [OK]");
  }
  
  Serial.println();
  
  delay(100); // Update every 100ms
}

// Read a register from AS5600 via I2C
uint8_t readRegister(uint8_t regAddr) {
  Wire.beginTransmission(AS5600_I2C_ADDR);
  Wire.write(regAddr);
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    Serial.print("I2C Error: ");
    Serial.println(error);
    return 0;
  }
  
  Wire.requestFrom(AS5600_I2C_ADDR, 1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0;
}

// Read a 16-bit register (high byte first)
uint16_t readRegister16(uint8_t regAddr) {
  uint8_t highByte = readRegister(regAddr);
  uint8_t lowByte = readRegister(regAddr + 1);
  return (highByte << 8) | lowByte;
}

// Read raw angle (0-4095)
uint16_t readRawAngle() {
  return readRegister16(AS5600_REG_RAW_ANGLE);
}

// Read filtered angle (0-4095)
uint16_t readAngle() {
  return readRegister16(AS5600_REG_ANGLE);
}

// Read magnet magnitude (strength)
uint16_t readMagnitude() {
  return readRegister16(AS5600_REG_MAGNITUDE);
}

// Read status register
uint8_t readStatus() {
  return readRegister(AS5600_REG_STATUS);
}

// Test connection and display initial values
void testConnection() {
  Serial.println("Testing AS5600 connection...");
  
  // Try to read status register
  uint8_t status = readStatus();
  Serial.print("Status register: 0x");
  Serial.println(status, HEX);
  
  if (status == 0xFF || status == 0x00) {
    Serial.println("\n⚠ WARNING: Could not read from sensor!");
    Serial.println("Check wiring:");
    Serial.println("  - VCC -> 3.3V");
    Serial.println("  - GND -> GND");
    Serial.println("  - SDA -> GPIO 21 (or your SDA pin)");
    Serial.println("  - SCL -> GPIO 22 (or your SCL pin)");
    Serial.println("\nIf using different I2C pins, update SDA_PIN and SCL_PIN");
    return;
  }
  
  uint16_t rawAngle = readRawAngle();
  Serial.print("Raw angle: ");
  Serial.println(rawAngle);
  
  uint16_t magnitude = readMagnitude();
  Serial.print("Magnet magnitude: ");
  Serial.println(magnitude);
  
  if (magnitude == 0 || magnitude > 5000) {
    Serial.println("\n⚠ WARNING: Check magnet positioning!");
    Serial.println("  - Magnet should be 1-3mm above sensor");
    Serial.println("  - Ensure magnet is centered over sensor");
  } else {
    Serial.println("\n✓ Sensor detected! Wiring appears correct.");
    Serial.println("✓ Magnet detected! Position looks good.");
  }
  
  Serial.println();
}

