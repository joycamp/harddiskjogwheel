# BLE MIDI Setup for ESP32-S3

This version uses Bluetooth Low Energy (BLE) MIDI instead of USB MIDI, which works well on macOS/iOS.

## Library Installation

1. **Install ESP32-BLE-MIDI Library:**
   - Go to **Tools > Manage Libraries**
   - Search for "ESP32-BLE-MIDI"
   - Install "ESP32-BLE-MIDI" by lathoub (or max22-)

## Board Settings

- **Tools > USB Mode** → Can be "Hardware CDC and JTAG" (Serial will work!)
- **Tools > USB CDC On Boot** → "Enabled"

## Setup Steps

1. **Upload the BLE MIDI code** (`minimal_test_ble.ino`)

2. **Pair with macOS:**
   - Open **System Settings > Bluetooth**
   - Look for "ESP32-S3 MIDI"
   - Click **Connect**

3. **Add to Audio MIDI Setup:**
   - Open **Audio MIDI Setup** (Applications > Utilities)
   - Go to **Window > Show MIDI Studio**
   - The device should appear automatically after pairing
   - If not, click **"Rescan MIDI"**

4. **Test:**
   - Open Serial Monitor (115200 baud) to see status
   - The device sends CC60 = 1 every second when connected
   - Check Audio MIDI Setup to verify messages are being received

## Advantages of BLE MIDI

- ✅ Works wirelessly
- ✅ Native macOS/iOS support
- ✅ Serial debugging works (unlike USB-OTG mode)
- ✅ More reliable than USB MIDI on some boards
- ✅ Can connect to multiple devices

## Notes

- The device name is "ESP32-S3 MIDI" - look for this in Bluetooth settings
- You may need to pair it first in Bluetooth settings before it appears in MIDI Studio
- BLE MIDI has slightly higher latency than USB MIDI, but usually not noticeable




