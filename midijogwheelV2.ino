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
// BLE MIDI library uses 1-16 for channels (1 = MIDI channel 1, 2 = MIDI channel 2, etc.)
// If monitor shows wrong channel, adjust this value (1-16)
const int MIDI_CHANNEL = 1;  // MIDI channel (1-16, where 1 = MIDI channel 1)
const int JOG_CC = 60;       // CC number for jog wheel
const int BUTTON_CC = 61;    // CC number for button
// Relative mode encoding for Logic Pro:
// Values 1-63 = increment (positive), Values 65-127 = decrement (negative)
// Value 64 = no change (never send this)
// We send constant values: 1 for CW, 65 for CCW (not proportional to speed)
const int CW_VALUE = 1;       // Clockwise rotation value
const int CCW_VALUE = 65;    // Counter-clockwise rotation value

// AS5600 angle tracking
uint16_t lastAngle = 0;
uint16_t smoothedAngle = 0;  // Smoothed angle value for direction detection
bool angleInitialized = false;  // Track if we've initialized smoothing
const int ANGLE_THRESHOLD = 10;  // Minimum smoothed angle change to trigger MIDI (0-4095 scale)

// Movement accumulator to ignore small initial touches
int movementAccumulator = 0;  // Accumulates movement in current direction
int lastDirection = 0;  // Last movement direction: 1 = positive, -1 = negative, 0 = none
const int MOVEMENT_THRESHOLD = 30;  // Minimum cumulative movement before sending MIDI (prevents accidental touches)

// Direction lock to prevent rapid direction changes
int lockedDirection = 0;  // Currently locked direction: 1 = positive, -1 = negative, 0 = none
int oppositeDirectionAccumulator = 0;  // Accumulates movement in opposite direction
int consecutiveOppositeSamples = 0;  // Count consecutive samples in opposite direction
unsigned long directionLockTime = 0;  // When direction was locked
const int DIRECTION_CHANGE_THRESHOLD = 300;  // Must accumulate this much in opposite direction before switching
const int MIN_CONSECUTIVE_OPPOSITE = 10;  // Require this many consecutive opposite samples before even accumulating
const unsigned long MIN_LOCK_TIME_MS = 500;  // Direction must be locked for this long before allowing change
unsigned long lastOppositeMovementTime = 0;  // Track when last opposite movement occurred
const unsigned long OPPOSITE_DECAY_TIME_MS = 500;  // If no opposite movement for this long, reset accumulator

// Smoothing configuration - used only to detect direction reliably
const float SMOOTHING_FACTOR = 0.2;  // 0.0-1.0, lower = more smoothing (0.2 = 20% new, 80% old)
const unsigned long MAX_SAMPLE_TIME_MS = 100;  // Maximum time between samples to reset (prevents large jumps)
const int MAX_ANGLE_CHANGE_PER_SAMPLE = 500;  // Maximum angle change per sample (safety limit)

// Button state variables
int lastButtonState = HIGH;  // Button is HIGH when not pressed (pull-up)

// Button debounce variables
unsigned long lastButtonDebounceTime = 0;
const unsigned long BUTTON_DEBOUNCE_DELAY = 50;

// Helper function to send MIDI Control Change message
void sendMIDICC(uint8_t channel, uint8_t control, uint8_t value) {
  // Only send if BLE MIDI is connected
  if (BLEMidiServer.isConnected()) {
    // BLE MIDI library uses 1-16 for channels
    // channel parameter is already 1-16, pass it directly
    BLEMidiServer.controlChange(channel, control, value);
  }
}

// Read a register from AS5600 via I2C
uint8_t readRegister(uint8_t regAddr) {
  // Add small delay to prevent bus issues
  delayMicroseconds(100);
  
  Wire.beginTransmission(AS5600_I2C_ADDR);
  Wire.write(regAddr);
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    // Log error details occasionally
    static unsigned long lastI2CError = 0;
    if (millis() - lastI2CError > 5000) {
      Serial.print("I2C Error: ");
      Serial.print(error);
      Serial.print(" (0=OK, 1=data too long, 2=NACK addr, 3=NACK data, 4=other)");
      Serial.print(" | Address: 0x");
      Serial.print(AS5600_I2C_ADDR, HEX);
      Serial.print(" | Register: 0x");
      Serial.print(regAddr, HEX);
      Serial.print(" | Try: Check wiring, pull-up resistors, power");
      Serial.println();
      lastI2CError = millis();
    }
    return 0xFF; // Error indicator
  }
  
  // Small delay before requesting data
  delayMicroseconds(100);
  
  uint8_t bytesRequested = Wire.requestFrom(AS5600_I2C_ADDR, 1, true); // true = stop after request
  if (bytesRequested == 1 && Wire.available()) {
    return Wire.read();
  }
  
  // No data available
  static unsigned long lastI2CError = 0;
  if (millis() - lastI2CError > 5000) {
    Serial.print("I2C Error: No data available | Requested: ");
    Serial.print(bytesRequested);
    Serial.print(" | Available: ");
    Serial.println(Wire.available());
    lastI2CError = millis();
  }
  return 0xFF;
}

// Read a 16-bit register (high byte first) - AS5600 specific
uint16_t readRegister16(uint8_t regAddr) {
  // AS5600 16-bit registers are read as two consecutive bytes
  // Read both bytes in a single I2C transaction for better reliability
  delayMicroseconds(100);
  
  Wire.beginTransmission(AS5600_I2C_ADDR);
  Wire.write(regAddr);
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    return 0xFFFF; // Error indicator
  }
  
  delayMicroseconds(100);
  
  // Request 2 bytes (high byte, then low byte)
  uint8_t bytesRequested = Wire.requestFrom(AS5600_I2C_ADDR, 2, true);
  if (bytesRequested == 2 && Wire.available() >= 2) {
    uint8_t highByte = Wire.read();
    uint8_t lowByte = Wire.read();
    return (highByte << 8) | lowByte;
  }
  
  return 0xFFFF; // Error indicator
}

// Reset I2C bus - useful if bus gets locked up
void resetI2CBus() {
  Wire.end();
  delay(10);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(50000);
  Wire.setTimeout(1000);
}

// Read raw angle from AS5600 (0-4095)
uint16_t readRawAngle() {
  uint16_t result = readRegister16(AS5600_REG_RAW_ANGLE);
  
  // If we get repeated errors, try resetting the bus
  static int errorCount = 0;
  if (result == 0xFFFF) {
    errorCount++;
    if (errorCount > 10) {
      Serial.println("Resetting I2C bus due to repeated errors...");
      resetI2CBus();
      errorCount = 0;
      delay(50); // Give bus time to stabilize
    }
  } else {
    errorCount = 0; // Reset counter on successful read
  }
  
  return result;
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
  Serial.println("Version: 2.1 - Constant Values (1/65)");
  Serial.println("========================================");
  
  // Initialize I2C for AS5600
  Serial.println("Initializing I2C for AS5600...");
  Wire.begin(SDA_PIN, SCL_PIN);
  // Try slower I2C speed - AS5600 can be sensitive to timing
  Wire.setClock(50000); // 50kHz I2C speed (slower, more reliable)
  // Set I2C timeout to prevent hanging
  Wire.setTimeout(1000); // 1 second timeout
  Serial.print("SDA Pin: "); Serial.println(SDA_PIN);
  Serial.print("SCL Pin: "); Serial.println(SCL_PIN);
  Serial.print("I2C Address: 0x"); Serial.println(AS5600_I2C_ADDR, HEX);
  Serial.print("I2C Clock: 50kHz");
  
  // Scan I2C bus to see what devices are present
  Serial.println("\nScanning I2C bus...");
  byte devicesFound = 0;
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("  I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      if (address == AS5600_I2C_ADDR) {
        Serial.println(" <- AS5600 (expected)");
      } else {
        Serial.println();
      }
      devicesFound++;
    } else if (error == 4) {
      Serial.print("  Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  
  if (devicesFound == 0) {
    Serial.println("  ⚠ No I2C devices found!");
    Serial.println("  Check wiring: SDA->GPIO8, SCL->GPIO9, VCC->3.3V, GND->GND");
  } else {
    Serial.print("  Found ");
    Serial.print(devicesFound);
    Serial.println(" device(s)");
  }
  
  // Test AS5600 connection
  delay(100);
  Serial.println("\nTesting AS5600 communication...");
  uint16_t testAngle = readRawAngle();
  uint16_t testMagnitude = readMagnitude();
  
  if (testAngle == 0xFFFF) {
    Serial.println("⚠ WARNING: Could not read from AS5600!");
    Serial.println("Check wiring: VCC->3.3V, GND->GND, SDA->GPIO8, SCL->GPIO9");
    Serial.println("Verify magnet is positioned 1-3mm above sensor");
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
  // Check connection status (only log when it changes)
  static bool lastConnectionState = false;
  bool isConnected = BLEMidiServer.isConnected();
  
  if (isConnected != lastConnectionState) {
    Serial.print("BLE MIDI: ");
    Serial.println(isConnected ? "CONNECTED" : "DISCONNECTED");
    lastConnectionState = isConnected;
  }
  
  // Process encoder/button regardless of connection (for testing)
  // MIDI will only be sent if connected
  {
    // Connection is logged above when status changes
    
    // Track time for rate limiting
    static unsigned long lastSampleTime = 0;
    unsigned long currentTime = millis();
    unsigned long timeDelta = currentTime - lastSampleTime;
    
    // Read current angle from AS5600
    uint16_t currentAngle = readRawAngle();
    
    // Check if read was successful
    if (currentAngle == 0xFFFF) {
      // I2C read failed - log occasionally
      static unsigned long lastErrorTime = 0;
      if (millis() - lastErrorTime > 5000) {
        Serial.println("⚠ ERROR: Failed to read from AS5600 (I2C error)");
        lastErrorTime = millis();
      }
    } else {
      // Apply exponential smoothing to reduce noise and detect direction reliably
      if (!angleInitialized) {
        // First reading - initialize smoothed value
        smoothedAngle = currentAngle;
        lastAngle = currentAngle;
        angleInitialized = true;
        lastSampleTime = currentTime;
        // Angle initialized silently
      } else {
        // Reset if too much time has passed (prevents large jumps from delays)
        if (timeDelta > MAX_SAMPLE_TIME_MS) {
          smoothedAngle = currentAngle;
          lastAngle = currentAngle;
          lastSampleTime = currentTime;
        } else {
          // Apply exponential smoothing: smoothed = (1-alpha) * old + alpha * new
          // Using floating point for accuracy, then convert to uint16_t
          float newSmoothed = (1.0 - SMOOTHING_FACTOR) * smoothedAngle + SMOOTHING_FACTOR * currentAngle;
          smoothedAngle = (uint16_t)round(newSmoothed);
          
          // Calculate angle change from smoothed values, handling wrap-around at 0/4095 boundary
          int angleChange = (int)smoothedAngle - (int)lastAngle;
          
          // Handle wrap-around: if change is > 2048, assume wrap-around in CCW direction
          if (angleChange > 2048) {
            angleChange = angleChange - 4096;
          }
          // Handle wrap-around: if change is < -2048, assume wrap-around in CW direction
          else if (angleChange < -2048) {
            angleChange = angleChange + 4096;
          }
          
          // Safety limit: prevent huge values from calculation errors
          if (angleChange > MAX_ANGLE_CHANGE_PER_SAMPLE) {
            angleChange = MAX_ANGLE_CHANGE_PER_SAMPLE;
          } else if (angleChange < -MAX_ANGLE_CHANGE_PER_SAMPLE) {
            angleChange = -MAX_ANGLE_CHANGE_PER_SAMPLE;
          }
          
          // Only process if smoothed change exceeds threshold (to avoid noise)
          if (abs(angleChange) >= ANGLE_THRESHOLD) {
            int currentDirection = (angleChange > 0) ? 1 : -1;
            
            // Direction lock logic: prevent rapid direction changes
            if (lockedDirection == 0) {
              // No direction locked yet - establish one
              lockedDirection = currentDirection;
              directionLockTime = currentTime;
              movementAccumulator += abs(angleChange);
              oppositeDirectionAccumulator = 0;
            } else if (currentDirection == lockedDirection) {
              // Same direction as locked - accumulate normally
              movementAccumulator += abs(angleChange);
              oppositeDirectionAccumulator = 0;  // Reset opposite direction accumulator
              consecutiveOppositeSamples = 0;  // Reset consecutive opposite counter
              lastOppositeMovementTime = 0;  // Reset opposite movement timer
            } else {
              // Opposite direction detected
              // Only allow direction change if direction has been locked for minimum time
              if ((currentTime - directionLockTime) < MIN_LOCK_TIME_MS) {
                // Too soon to change direction - completely ignore opposite movement
                return;  // Exit early, don't process this sample at all
              }
              
              consecutiveOppositeSamples++;
              lastOppositeMovementTime = currentTime;
              
              // Only start accumulating opposite movement after multiple consecutive samples
              // This filters out single-sample noise/vibration
              if (consecutiveOppositeSamples >= MIN_CONSECUTIVE_OPPOSITE) {
                // We have enough consecutive opposite samples - start accumulating
                oppositeDirectionAccumulator += abs(angleChange);
                
                // Only switch direction if we've accumulated enough in opposite direction
                if (oppositeDirectionAccumulator >= DIRECTION_CHANGE_THRESHOLD) {
                  // Direction change confirmed - switch lock
                  lockedDirection = currentDirection;
                  directionLockTime = currentTime;  // Reset lock time
                  movementAccumulator = oppositeDirectionAccumulator - DIRECTION_CHANGE_THRESHOLD;
                  oppositeDirectionAccumulator = 0;
                  consecutiveOppositeSamples = 0;
                  lastOppositeMovementTime = 0;
                  // Continue processing with new direction
                } else {
                  // Not enough movement to change direction - completely ignore this movement
                  // CRITICAL: Don't update angle tracking - this prevents opposite movements from affecting future calculations
                  lastDirection = currentDirection;  // Track direction for decay logic
                  // Skip MIDI sending - exit this iteration early
                  return;  // Exit early, don't process this sample at all
                }
              } else {
                // Not enough consecutive opposite samples yet - completely ignore
                // Don't update angle tracking, don't accumulate, just ignore
                return;  // Exit early, don't process this sample at all
              }
            }
            
            lastDirection = currentDirection;
            
            // Decay opposite direction accumulator if no opposite movement for a while
            if (oppositeDirectionAccumulator > 0 && 
                (currentTime - lastOppositeMovementTime) > OPPOSITE_DECAY_TIME_MS) {
              oppositeDirectionAccumulator = max(0, oppositeDirectionAccumulator - 10);
              consecutiveOppositeSamples = 0;
              if (oppositeDirectionAccumulator == 0) {
                lastOppositeMovementTime = 0;
              }
            }
            
            // Only send MIDI once we've accumulated enough movement (definite spin)
            // AND we're processing in the locked direction (not accumulating opposite)
            if (movementAccumulator >= MOVEMENT_THRESHOLD) {
              if (lockedDirection > 0) {
                // Positive angle change (CW): send decrement (backward) = 65
                sendMIDICC(MIDI_CHANNEL, JOG_CC, CCW_VALUE);
                Serial.print("▶ CW | Change: ");
                Serial.print(angleChange);
                Serial.print(" | Accum: ");
                Serial.print(movementAccumulator);
                Serial.print(" | CC60 = ");
                Serial.println(CCW_VALUE);
              } else {
                // Negative angle change (CCW): send increment (forward) = 1
                sendMIDICC(MIDI_CHANNEL, JOG_CC, CW_VALUE);
                Serial.print("◀ CCW | Change: ");
                Serial.print(angleChange);
                Serial.print(" | Accum: ");
                Serial.print(movementAccumulator);
                Serial.print(" | CC60 = ");
                Serial.println(CW_VALUE);
              }
              
              // Reset accumulator after sending (but keep some to maintain responsiveness)
              movementAccumulator = movementAccumulator - MOVEMENT_THRESHOLD;
              
              // Update last angle from smoothed value (not raw)
              lastAngle = smoothedAngle;
              lastSampleTime = currentTime;
            }
          } else {
            // If movement is too small, gradually decay the accumulators
            // This prevents accidental touches from accumulating over time
            if (movementAccumulator > 0) {
              movementAccumulator = max(0, movementAccumulator - 5);
            }
            if (oppositeDirectionAccumulator > 0) {
              oppositeDirectionAccumulator = max(0, oppositeDirectionAccumulator - 5);
            }
            // Reset direction lock if both accumulators reach zero
            if (movementAccumulator == 0 && oppositeDirectionAccumulator == 0) {
              lockedDirection = 0;
              lastDirection = 0;
            }
          }
        }
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
  }
  
  // Connection status is logged above when it changes
  
  delay(10); // Small delay - AS5600 doesn't need super fast polling
}

