# Hard Disk Jog Wheel

Turn a hard disk drive (or any rotary mechanism) into a wireless MIDI jog wheel for video or audio editing using an ESP32-S3 and AS5600 magnetic encoder.

This project creates a Bluetooth Low Energy (BLE) MIDI device that sends relative jog wheel commands compatible with Logic Pro, Final Cut Pro (via Command Post), and other DAWs.

## Features

- **Wireless BLE MIDI** - No cables needed, works with macOS Audio MIDI Setup
- **AS5600 Magnetic Encoder** - Contactless, high-resolution rotation sensing
- **Relative MIDI Mode** - Proper jog wheel behavior with proportional speed control
- **Button Support** - Additional MIDI CC for button press/release
- **Configurable Sensitivity** - Adjustable rotation thresholds and scaling

## Hardware Requirements

- **ESP32-S3** development board (e.g., ESP32-S3-DevKitC-1)
- **AS5600** magnetic position sensor (AMS AS5600)
- **Magnet** - Diametrically magnetized magnet (typically 6-8mm diameter, 2-3mm thick)
- **Button** - Optional tactile button for additional control
- **Wiring** - Jumper wires, breadboard (or custom PCB)

## Hardware Connections

### AS5600 Magnetic Encoder
- **VCC** → 3.3V (ESP32-S3)
- **GND** → GND (ESP32-S3)
- **SDA** → GPIO 8
- **SCL** → GPIO 9

**Note:** The AS5600 operates at 3.3V logic levels, which matches the ESP32-S3 perfectly. No level shifters needed.

### Button (Optional)
- **One terminal** → GPIO 4
- **Other terminal** → GND

The code uses internal pull-up resistors, so no external pull-up needed.

### Magnet Positioning
- Place magnet **1-3mm** above the AS5600 sensor
- Ensure magnet is **centered** over the sensor
- Magnet should be **diametrically magnetized** (north-south poles on opposite sides)

## Software Setup

### 1. Install Arduino IDE

Download and install [Arduino IDE](https://www.arduino.cc/en/software) (version 1.8.19 or later).

### 2. Install ESP32-S3 Board Support

1. Open Arduino IDE
2. Go to **File > Preferences**
3. In "Additional Board Manager URLs", add:
   ```
   https://espressif.github.io/arduino-esp32/package_esp32_index.json
   ```
4. Go to **Tools > Board > Boards Manager**
5. Search for "ESP32" and install **"esp32 by Espressif Systems"** (version 3.3.3 or later)
6. Select your board: **Tools > Board > ESP32 Arduino > ESP32S3 Dev Module**

### 3. Install Required Libraries

Install these libraries via **Tools > Manage Libraries**:

#### Required Libraries:

1. **ESP32-BLE-MIDI** by max22-
   - Search for "ESP32-BLE-MIDI" in Library Manager
   - Install version compatible with ESP32-S3
   - GitHub: https://github.com/max22-/ESP32-BLE-MIDI

2. **NimBLE-Arduino** by h2zero
   - Search for "NimBLE-Arduino" in Library Manager
   - Install latest version
   - Required dependency for ESP32-BLE-MIDI
   - GitHub: https://github.com/h2zero/NimBLE-Arduino

#### Built-in Libraries (No Installation Needed):
- **Wire** - I2C communication (included with ESP32 core)
- **Arduino.h** - Core Arduino functions (included with ESP32 core)

### 4. Configure Board Settings

In Arduino IDE, configure these settings:

- **Tools > Board** → **ESP32S3 Dev Module**
- **Tools > USB Mode** → **Hardware CDC and JTAG** (for Serial debugging)
- **Tools > USB CDC On Boot** → **Enabled**
- **Tools > Partition Scheme** → **Default 4MB with spiffs** (or **Huge APP**)
- **Tools > Port** → Select your ESP32-S3 COM port

### 5. Upload the Sketch

1. Open `midijogwheelV2.ino` in Arduino IDE
2. Verify all settings are correct
3. Click **Upload**

**If upload fails:**
- Hold **BOOT** button on ESP32-S3
- Press and release **RESET** button
- Release **BOOT** button
- Immediately click Upload in Arduino IDE

### 6. Connect via Bluetooth

1. Open **Audio MIDI Setup** (Applications > Utilities on macOS)
2. Go to **Window > Show MIDI Studio**
3. Click the **"Configure Bluetooth"** icon (or go to **Window > Configure Bluetooth**)
4. Find **"ESP32-S3-TE!"** in the list
5. Click **Connect**

The device should now appear in your MIDI Studio and be available to Logic Pro, Final Cut Pro, and other applications.

## Configuration

### MIDI Settings

Edit these constants in `midijogwheelV2.ino`:

```cpp
const int MIDI_CHANNEL = 0;  // MIDI channel (0-15, 0 = channel 1)
const int JOG_CC = 60;       // CC number for jog wheel
const int BUTTON_CC = 61;    // CC number for button
```

### Sensitivity Settings

Adjust rotation sensitivity:

```cpp
const int ANGLE_THRESHOLD = 10;        // Minimum angle change to trigger MIDI (0-4095 scale)
const int MIN_RELATIVE_VALUE = 1;      // Minimum relative increment
const int MAX_RELATIVE_VALUE = 10;     // Maximum relative increment (lower = less sensitive)
const int RELATIVE_SCALE = 20;         // Scale factor (higher = less sensitive)
```

**Tips:**
- Lower `MAX_RELATIVE_VALUE` (e.g., 5) = less sensitive to fast spinning
- Higher `RELATIVE_SCALE` (e.g., 30) = overall less sensitive
- Lower `ANGLE_THRESHOLD` (e.g., 5) = more responsive to small movements

### Pin Configuration

If you need different GPIO pins:

```cpp
const int SDA_PIN = 8;      // AS5600 SDA pin
const int SCL_PIN = 9;      // AS5600 SCL pin
const int BUTTON_PIN = 4;   // Button pin
```

**Note:** ESP32-S3 supports I2C on any GPIO pins. Choose pins that don't conflict with your board's USB/JTAG pins.

### Device Name

Change the BLE device name:

```cpp
BLEMidiServer.begin("ESP32-S3-TE!");  // Change to your preferred name
```

**Note:** Keep the name short (max 29 characters) and avoid special characters that might cause issues.

## MIDI Protocol

### Jog Wheel (CC 60)

Uses **relative mode encoding**:
- **Clockwise rotation**: CC 60 = 1-63 (increment values, proportional to rotation speed)
- **Counter-clockwise rotation**: CC 60 = 65-127 (decrement values, proportional to rotation speed)
- **No movement**: No MIDI messages sent

Values scale with rotation speed:
- Slow rotation → small values (1-2 or 65-66)
- Fast rotation → larger values (up to MAX_RELATIVE_VALUE or 64+MAX_RELATIVE_VALUE)

### Button (CC 61)

- **Press**: CC 61 = 127
- **Release**: CC 61 = 0

## Usage with DAWs

### Logic Pro

1. Open Logic Pro
2. Go to **Logic Pro > Settings > MIDI**
3. Ensure your ESP32-S3 device is enabled
4. Use **Learn** function:
   - Right-click on transport controls or jog wheel
   - Select **"Learn MIDI Assignment"**
   - Rotate the jog wheel
   - Logic Pro will learn the MIDI CC message

### Final Cut Pro (via Command Post)

1. Install [Command Post](https://commandpost.fcp.cafe/)
2. Open Command Post preferences
3. Go to **MIDI** settings
4. Click **Learn** for the action you want to map
5. Rotate the jog wheel
6. Command Post will learn the MIDI assignment

**Note:** Ensure the device name in Command Post matches exactly what your ESP32-S3 advertises (default: "ESP32-S3-TE!").

### Other DAWs

Any DAW that supports MIDI CC learning should work. The device sends standard MIDI Control Change messages over BLE MIDI.

## Testing

### Serial Monitor

Open Serial Monitor (115200 baud) to see:
- AS5600 initialization status
- Angle readings and changes
- MIDI messages being sent
- Connection status

### I2C Scanner

If the AS5600 isn't responding, use the included `i2c_scanner` sketch to verify I2C communication.

### AS5600 Test Sketch

Use `as5600_test_i2c/as5600_test_i2c.ino` to verify:
- AS5600 wiring is correct
- Magnet is positioned properly
- Sensor readings are stable

## Troubleshooting

### Device Not Appearing in Bluetooth Settings

- Ensure BLE MIDI is initialized (check Serial Monitor)
- Try disconnecting and reconnecting in Audio MIDI Setup
- Restart Audio MIDI Setup
- Check that no other device is using the same name

### AS5600 Not Responding (I2C Error)

- Check wiring: VCC→3.3V, GND→GND, SDA→GPIO8, SCL→GPIO9
- Verify magnet is positioned 1-3mm above sensor
- Use I2C scanner to check if device appears at address 0x36
- Check Serial Monitor for specific error codes

### MIDI Messages Not Working in DAW

- Verify device is connected in Audio MIDI Setup
- Check MIDI channel matches what DAW expects (default: channel 1)
- Ensure DAW has MIDI input enabled
- Try learning the MIDI assignment again
- Check Serial Monitor to confirm messages are being sent

### Too Sensitive / Not Sensitive Enough

- Adjust `MAX_RELATIVE_VALUE` (lower = less sensitive to fast spinning)
- Adjust `RELATIVE_SCALE` (higher = overall less sensitive)
- Adjust `ANGLE_THRESHOLD` (lower = more responsive)

### Upload Fails

- Use manual bootloader mode (hold BOOT, press RESET, release BOOT, upload)
- Try different USB cable (some are power-only)
- Try different USB port
- Close Serial Monitor before uploading
- Ensure no other program is using the serial port

## Project Structure

```
midijogwheelV2/
├── midijogwheelV2.ino          # Main sketch
├── README.md                    # This file
├── as5600_test_i2c/            # AS5600 test sketch
│   └── as5600_test_i2c.ino
├── i2c_scanner/                # I2C bus scanner utility
│   └── i2c_scanner.ino
├── serial_monitor.py           # Python serial monitor script
└── models/                     # 3D models (if applicable)
```

## Libraries Used

### ESP32-BLE-MIDI
- **Author**: max22-
- **Purpose**: BLE MIDI server implementation for ESP32
- **Installation**: Arduino Library Manager → Search "ESP32-BLE-MIDI"
- **GitHub**: https://github.com/max22-/ESP32-BLE-MIDI
- **License**: Check repository for license information

### NimBLE-Arduino
- **Author**: h2zero
- **Purpose**: NimBLE BLE stack for Arduino/ESP32 (dependency of ESP32-BLE-MIDI)
- **Installation**: Arduino Library Manager → Search "NimBLE-Arduino"
- **GitHub**: https://github.com/h2zero/NimBLE-Arduino
- **License**: Apache 2.0

### Wire (Built-in)
- **Purpose**: I2C communication library
- **Included**: With ESP32 Arduino core
- **Documentation**: https://www.arduino.cc/reference/en/language/functions/communication/wire/

## License

MIT License - See LICENSE file for details.

## Credits

- ESP32-S3 by Espressif Systems
- AS5600 by AMS (Austria Micro Systems)
- ESP32-BLE-MIDI library by max22-
- NimBLE-Arduino by h2zero

## Contributing

Contributions welcome! Please open an issue or pull request on GitHub.

## Support

For issues, questions, or contributions, please use the GitHub Issues page.
