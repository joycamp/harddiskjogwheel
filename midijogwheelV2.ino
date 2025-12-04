/*
 * MIDI Jog Wheel Controller for ESP32-S3
 * Reads AS5600 magnetic encoder rotation and button state
 * Sends MIDI CC messages over BLE MIDI (Bluetooth Low Energy)
 * 
 * Hardware Connections:
 * - AS5600 VCC -> 3.3V
 * - AS5600 GND -> GND
 * - AS5600 SDA -> GPIO 8
 * - AS5600 SCL -> GPIO 9
 * - Button -> GPIO pin (configurable, default: GPIO 4)
 * 
 * Libraries Required:
 * - ESP32-BLE-MIDI by max22- (install via Arduino Library Manager)
 * - NimBLE-Arduino by h2zero (install via Arduino Library Manager)
 * 
 * Board Settings:
 * - Tools > USB Mode → Hardware CDC and JTAG (for Serial debugging)
 * - Tools > USB CDC On Boot → Enabled
 * - Tools > Partition Scheme → Default 4MB with spiffs (or Huge APP)
 * 
 * MIDI Configuration:
 * - CW rotation sends: CC 60 = 65 (relative mode +1)
 * - CCW rotation sends: CC 60 = 63 (relative mode -1)
 * - Button press sends: CC 61 = 127
 * - Button release sends: CC 61 = 0
 * 
 * Connection:
 * 1. Upload this sketch to ESP32-S3
 * 2. Open Audio MIDI Setup (Applications > Utilities)
 * 3. Go to Window > Show MIDI Studio
 * 4. Click "Configure Bluetooth" icon
 * 5. Find "ESP32-S3 MIDI Jog Wheel" and click Connect
 */

#include <Arduino.h>
#include <BLEMidi.h>
#include <Wire.h>

// AS5600 I2C Configuration
#define AS5600_I2C_ADDR 0x36
#define AS5600_REG_RAW_ANGLE 0x0C  // 12-bit raw angle (0-4095)
#define AS5600_REG_MAGNITUDE 0x1B  // Magnet strength

// I2C Pin Definitions
const int SDA_PIN = 8;   // AS5600 SDA pin
const int SCL_PIN = 9;   // AS5600 SCL pin

// Button pin
const int BUTTON_PIN = 4;  // GPIO pin for button

// MIDI Configuration
// BLE MIDI library may be 0-indexed: 0 = MIDI channel 1, 1 = MIDI channel 2, etc.
// If Audio MIDI Monitor shows channel 2, change this to 0 to send on channel 1
const int MIDI_CHANNEL = 0;  // MIDI channel (0-15, 0 = channel 1)
const int JOG_CC = 60;       // CC number for jog wheel
const int BUTTON_CC = 61;    // CC number for button
// Relative mode encoding for Logic Pro:
// Values 1-63 = increment (positive), Values 65-127 = decrement (negative)
// Value 64 = no change (never send this)
// We'll send proportional values based on rotation speed
const int MIN_RELATIVE_VALUE = 1;   // Minimum relative increment
const int MAX_RELATIVE_VALUE = 10;   // Maximum relative increment (lower = less sensitive to fast spinning)
const int RELATIVE_SCALE = 20;      // Scale factor: angle change / this = MIDI value (higher = less sensitive)

// AS5600 angle tracking
uint16_t lastAngle = 0;
const int ANGLE_THRESHOLD = 10;  // Minimum angle change to trigger MIDI (0-4095 scale)

// Button state variables
int lastButtonState = HIGH;  // Button is HIGH when not pressed (pull-up)

// Button debounce variables
unsigned long lastButtonDebounceTime = 0;
const unsigned long BUTTON_DEBOUNCE_DELAY = 50;

// Helper function to send MIDI Control Change message
void sendMIDICC(uint8_t channel, uint8_t control, uint8_t value) {
  // Only send if BLE MIDI is connected
  if (BLEMidiServer.isConnected()) {
    // BLE MIDI library uses 1-16 for channels (not 0-15)
    BLEMidiServer.controlChange(channel, control, value);
  }
}

// Read a register from AS5600 via I2C
uint8_t readRegister(uint8_t regAddr) {
  Wire.beginTransmission(AS5600_I2C_ADDR);
  Wire.write(regAddr);
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    return 0xFF; // Error indicator
  }
  
  Wire.requestFrom(AS5600_I2C_ADDR, 1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0xFF;
}

// Read a 16-bit register (high byte first)
uint16_t readRegister16(uint8_t regAddr) {
  uint8_t highByte = readRegister(regAddr);
  uint8_t lowByte = readRegister(regAddr + 1);
  if (highByte == 0xFF || lowByte == 0xFF) {
    return 0xFFFF; // Error indicator
  }
  return (highByte << 8) | lowByte;
}

// Read raw angle from AS5600 (0-4095)
uint16_t readRawAngle() {
  return readRegister16(AS5600_REG_RAW_ANGLE);
}

// Read magnet magnitude (strength) from AS5600
uint16_t readMagnitude() {
  return readRegister16(AS5600_REG_MAGNITUDE);
}

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n========================================");
  Serial.println("MIDI Jog Wheel Controller");
  Serial.println("ESP32-S3 BLE MIDI + AS5600 Version");
  Serial.println("========================================");
  
  // Initialize I2C for AS5600
  Serial.println("Initializing I2C for AS5600...");
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000); // 100kHz I2C speed
  Serial.print("SDA Pin: "); Serial.println(SDA_PIN);
  Serial.print("SCL Pin: "); Serial.println(SCL_PIN);
  Serial.print("I2C Address: 0x"); Serial.println(AS5600_I2C_ADDR, HEX);
  
  // Test AS5600 connection
  delay(100);
  uint16_t testAngle = readRawAngle();
  uint16_t testMagnitude = readMagnitude();
  
  if (testAngle == 0xFFFF) {
    Serial.println("⚠ WARNING: Could not read from AS5600!");
    Serial.println("Check wiring: VCC->3.3V, GND->GND, SDA->GPIO8, SCL->GPIO9");
  } else {
    Serial.print("✓ AS5600 detected! Initial angle: ");
    Serial.println(testAngle);
    Serial.print("  Magnet magnitude: ");
    Serial.println(testMagnitude);
    if (testMagnitude < 100) {
      Serial.println("  ⚠ WARNING: Magnet may be too weak or too far!");
    }
    lastAngle = testAngle;
  }
  
  // Configure button pin
  Serial.println("Configuring button pin...");
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  lastButtonState = digitalRead(BUTTON_PIN);
  
  // Initialize BLE MIDI
  Serial.println("Initializing BLE MIDI...");
  BLEMidiServer.begin("ESP32-S3-TE!");
  Serial.println("BLE MIDI initialized");
  Serial.println("Device name: ESP32-S3 MIDI Jog Wheel");
  Serial.println("\nTo connect:");
  Serial.println("1. Open Audio MIDI Setup");
  Serial.println("2. Window > Show MIDI Studio");
  Serial.println("3. Click 'Configure Bluetooth'");
  Serial.println("4. Find 'ESP32-S3 MIDI Jog Wheel' and Connect");
  Serial.println("========================================\n");
  
  Serial.println("Setup complete! Waiting for BLE connection...");
  Serial.println("Rotate wheel or press button to test (after connecting)\n");
}

void loop() {
  // Only process encoder/button if BLE MIDI is connected
  if (BLEMidiServer.isConnected()) {
    // Read current angle from AS5600
    uint16_t currentAngle = readRawAngle();
    
    // Check if read was successful
    if (currentAngle != 0xFFFF) {
      // Calculate angle change, handling wrap-around at 0/4095 boundary
      int angleChange = currentAngle - lastAngle;
      
      // Handle wrap-around: if change is > 2048, assume wrap-around in CCW direction
      if (angleChange > 2048) {
        angleChange = angleChange - 4096;
      }
      // Handle wrap-around: if change is < -2048, assume wrap-around in CW direction
      else if (angleChange < -2048) {
        angleChange = angleChange + 4096;
      }
      
      // Only send MIDI if change exceeds threshold (to avoid noise)
      if (abs(angleChange) >= ANGLE_THRESHOLD) {
        // Calculate proportional relative value based on rotation amount
        // Logic Pro relative mode: 1-63 = increment, 65-127 = decrement
        int relativeValue;
        
        if (angleChange > 0) {
          // Clockwise rotation: send increment value (1-63)
          relativeValue = constrain(angleChange / RELATIVE_SCALE, MIN_RELATIVE_VALUE, MAX_RELATIVE_VALUE);
          sendMIDICC(MIDI_CHANNEL, JOG_CC, relativeValue);
          Serial.print("▶ CW - Angle: ");
          Serial.print(currentAngle);
          Serial.print(" | Change: ");
          Serial.print(angleChange);
          Serial.print(" | CC");
          Serial.print(JOG_CC);
          Serial.print(" = ");
          Serial.println(relativeValue);
        } else {
          // Counter-clockwise rotation: send decrement value (65-127)
          // Convert negative change to positive, then add 64 to get 65-127 range
          int absChange = abs(angleChange);
          relativeValue = constrain(64 + (absChange / RELATIVE_SCALE), 65, 127);
          sendMIDICC(MIDI_CHANNEL, JOG_CC, relativeValue);
          Serial.print("◀ CCW - Angle: ");
          Serial.print(currentAngle);
          Serial.print(" | Change: ");
          Serial.print(angleChange);
          Serial.print(" | CC");
          Serial.print(JOG_CC);
          Serial.print(" = ");
          Serial.println(relativeValue);
        }
        lastAngle = currentAngle;
      }
    }
    
    // Check for button press/release with debouncing
    int currentButtonState = digitalRead(BUTTON_PIN);
    
    if (currentButtonState != lastButtonState) {
      unsigned long currentTime = millis();
      
      // Debounce: wait a bit and check again
      if ((currentTime - lastButtonDebounceTime) > BUTTON_DEBOUNCE_DELAY) {
        currentButtonState = digitalRead(BUTTON_PIN);
        
        if (currentButtonState != lastButtonState) {
          lastButtonState = currentButtonState;
          lastButtonDebounceTime = currentTime;
          
          if (currentButtonState == LOW) {
            // Button pressed
            sendMIDICC(MIDI_CHANNEL, BUTTON_CC, 127);
            Serial.println("🔘 Button PRESSED");
          } else {
            // Button released
            sendMIDICC(MIDI_CHANNEL, BUTTON_CC, 0);
            Serial.println("🔘 Button RELEASED");
          }
        }
      }
    }
  } else {
    // Print connection status every 5 seconds if not connected
    static unsigned long lastStatusMessage = 0;
    if (millis() - lastStatusMessage > 5000) {
      Serial.println("Waiting for BLE MIDI connection...");
      Serial.println("Connect via Audio MIDI Setup > Configure Bluetooth");
      lastStatusMessage = millis();
    }
  }
  
  delay(10); // Small delay - AS5600 doesn't need super fast polling
}

