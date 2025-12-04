/*
 * MIDI Test Version - Sends CC60 = 1 every second
 * Use this to test USB MIDI transmission
 */

#include <Adafruit_TinyUSB.h>

// Create USB MIDI instance - must be global
Adafruit_USBD_MIDI usb_midi;

// MIDI Configuration
const int MIDI_CHANNEL = 0;  // MIDI channel (0-15, 0 = channel 1)
const int JOG_CC = 60;       // CC number for jog wheel
const int TEST_VALUE = 1;     // MIDI value to send

// Helper function to send MIDI Control Change message
void sendMIDICC(uint8_t channel, uint8_t control, uint8_t value) {
  // USB MIDI packet format: [CIN, Status, Data1, Data2]
  // CIN (Code Index Number) for 3-byte messages: 0x0B = Control Change
  uint8_t packet[4];
  packet[0] = 0x0B; // USB MIDI header (Cable 0, CIN 0x0B = Control Change)
  packet[1] = 0xB0 | (channel & 0x0F); // Status byte: Control Change + channel (0-15)
  packet[2] = control & 0x7F; // Control number (0-127)
  packet[3] = value & 0x7F;   // Value (0-127)
  usb_midi.write(packet, 4);
}

void setup() {
  // Initialize TinyUSB device stack FIRST
  TinyUSBDevice.begin();
  
  // Then initialize USB MIDI
  // This should register the MIDI interface with USB
  usb_midi.begin();
  
  // Critical: Wait for USB to enumerate
  // The device needs time to register as MIDI device
  delay(5000);
  
  // Note: Serial won't work in USB-OTG mode
  // Check System Information - Product ID should change if MIDI is registered
  // It should appear in Audio MIDI Setup as a MIDI device
}

void loop() {
  // Send CC60 = 1 every second
  sendMIDICC(MIDI_CHANNEL, JOG_CC, TEST_VALUE);
  
  Serial.print("Sent: CC");
  Serial.print(JOG_CC);
  Serial.print(" = ");
  Serial.println(TEST_VALUE);
  
  delay(1000); // Wait 1 second
}

