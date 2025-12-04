/*
 * AS5600 Magnetic Encoder Test
 * Tests SPI communication with AS5600 sensor
 * Displays angle readings and raw register values
 * 
 * AS5600 Connections (SPI mode):
 * - VCC -> 3.3V
 * - GND -> GND
 * - MISO -> GPIO 19 (or any available GPIO)
 * - MOSI -> GPIO 23 (or any available GPIO)
 * - SCK  -> GPIO 18 (or any available GPIO)
 * - CS   -> GPIO 5  (or any available GPIO)
 * 
 * Note: Adjust pin numbers below to match your wiring
 */

#include <SPI.h>

// SPI Pin Definitions (adjust for your wiring)
const int CS_PIN = 5;    // Chip Select pin
const int MISO_PIN = 19; // Master In, Slave Out
const int MOSI_PIN = 23; // Master Out, Slave In
const int SCK_PIN = 18;  // Serial Clock

// AS5600 Register Addresses
#define AS5600_ADDR 0x36  // I2C address (not used in SPI, but kept for reference)
#define AS5600_REG_STATUS 0x0B
#define AS5600_REG_RAW_ANGLE 0x0C  // 12-bit raw angle (0-4095)
#define AS5600_REG_ANGLE 0x0E      // Filtered angle
#define AS5600_REG_MAGNITUDE 0x1B  // Magnet strength

// SPI Settings
SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE1); // 1MHz, MSB first, Mode 1

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n\n========================================");
  Serial.println("AS5600 Magnetic Encoder Test");
  Serial.println("========================================");
  
  // Configure SPI pins
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH); // CS high = inactive
  
  pinMode(MISO_PIN, INPUT);
  pinMode(MOSI_PIN, OUTPUT);
  pinMode(SCK_PIN, OUTPUT);
  
  // Initialize SPI
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
  
  Serial.println("SPI initialized");
  Serial.print("CS Pin: "); Serial.println(CS_PIN);
  Serial.print("MISO Pin: "); Serial.println(MISO_PIN);
  Serial.print("MOSI Pin: "); Serial.println(MOSI_PIN);
  Serial.print("SCK Pin: "); Serial.println(SCK_PIN);
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

// Read a register from AS5600 via SPI
uint8_t readRegister(uint8_t regAddr) {
  SPI.beginTransaction(spiSettings);
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(1);
  
  // Send read command: bit 7 = 1 for read, bits 6-0 = register address
  uint8_t cmd = (regAddr & 0x3F) | 0x80; // Set bit 7 for read
  SPI.transfer(cmd);
  delayMicroseconds(1);
  
  // Read the data (dummy byte, then actual data)
  uint8_t data = SPI.transfer(0x00);
  
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
  
  return data;
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
  
  uint8_t status = readStatus();
  Serial.print("Status register: 0x");
  Serial.println(status, HEX);
  
  uint16_t rawAngle = readRawAngle();
  Serial.print("Raw angle: ");
  Serial.println(rawAngle);
  
  uint16_t magnitude = readMagnitude();
  Serial.print("Magnet magnitude: ");
  Serial.println(magnitude);
  
  if (magnitude == 0 || magnitude > 5000) {
    Serial.println("\n⚠ WARNING: Check wiring!");
    Serial.println("  - Verify VCC (3.3V) and GND connections");
    Serial.println("  - Check SPI pins (MISO, MOSI, SCK, CS)");
    Serial.println("  - Ensure magnet is positioned correctly");
  } else {
    Serial.println("\n✓ Sensor detected! Wiring appears correct.");
  }
  
  Serial.println();
}



