/*
 * BLE MIDI Test Version - Sends CC60 = 1 every second
 * Uses Bluetooth Low Energy MIDI (works on macOS/iOS)
 * 
 * Required Libraries:
 * - ESP32-BLE-MIDI by max22- (already installed)
 * - NimBLE-Arduino by h2zero (install from Library Manager)
 *   https://github.com/h2zero/NimBLE-Arduino
 */

#include <Arduino.h>
#include <BLEMidi.h>

// MIDI Configuration
const int MIDI_CHANNEL = 1;  // MIDI channel (1-16, 1 = channel 1)
const int JOG_CC = 60;       // CC number for jog wheel
const int TEST_VALUE = 1;    // MIDI value to send

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  delay(2000);  // Longer delay to ensure Serial is ready
  
  Serial.println("========================================");
  Serial.println("BLE MIDI Test - Sending CC60 = 1 every second");
  Serial.println("========================================");
  
  // Initialize BLE MIDI
  Serial.println("Initializing BLE MIDI...");
  Serial.println("Calling BLEMidiServer.begin()...");
  
  BLEMidiServer.begin("ESP32-S3 MIDI");
  
  // Wait for BLE to initialize and start advertising
  Serial.println("Waiting for BLE to start advertising...");
  delay(2000);
  
  Serial.println("BLE MIDI initialized!");
  Serial.println("Device name: ESP32-S3 MIDI");
  Serial.println("BLE should now be advertising...");
  Serial.println("Look for this device in Bluetooth settings");
  Serial.println("Then connect it in Audio MIDI Setup");
  Serial.println("========================================");
  
  // Print initial connection status
  Serial.print("Initial connection status: ");
  Serial.println(BLEMidiServer.isConnected() ? "CONNECTED" : "NOT CONNECTED");
}

void loop() {
  // Check connection status
  bool connected = BLEMidiServer.isConnected();
  
  // Print status every 5 seconds
  static unsigned long lastStatusMessage = 0;
  if (millis() - lastStatusMessage > 5000) {
    Serial.println("\n--- Status Update ---");
    Serial.print("Connection status: ");
    Serial.println(connected ? "CONNECTED" : "NOT CONNECTED");
    Serial.print("Uptime: ");
    Serial.print(millis() / 1000);
    Serial.println(" seconds");
    if (!connected) {
      Serial.println(">>> BLE is advertising - look for 'ESP32-S3 MIDI' in Bluetooth settings <<<");
    }
    Serial.println("--- End Status ---\n");
    lastStatusMessage = millis();
  }
  
  // Only send MIDI if connected
  if (connected) {
    // Send CC60 = 1 every second
    BLEMidiServer.controlChange(MIDI_CHANNEL, JOG_CC, TEST_VALUE);
    
    Serial.print("[CONNECTED] Sent: CC");
    Serial.print(JOG_CC);
    Serial.print(" = ");
    Serial.println(TEST_VALUE);
  } else {
    // Print waiting message every 10 seconds when not connected
    static unsigned long lastWaitMessage = 0;
    if (millis() - lastWaitMessage > 10000) {
      Serial.println("[NOT CONNECTED] Waiting for Bluetooth connection...");
      lastWaitMessage = millis();
    }
  }
  
  delay(1000); // Wait 1 second
}

