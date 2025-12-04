#!/usr/bin/env python3
"""
Simple Serial Monitor for ESP32-S3
Reads Serial output from the board
"""

import sys
import time

# Import serial at module level
try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("Error: pyserial not installed. Install with: pip3 install pyserial")
    sys.exit(1)

# Try to find the port
PORT = '/dev/cu.usbmodem23201'  # Update this if your port is different
BAUD = 115200

def main():
    print(f"Connecting to {PORT} at {BAUD} baud...")
    print("Press Ctrl+C to exit\n")
    
    ser = None
    try:
        print(f"Opening port {PORT}...")
        ser = serial.Serial(PORT, BAUD, timeout=1)
        print("Connected! Reading Serial output...\n")
        print("="*60)
        print("Press RESET button on ESP32-S3 to see boot messages")
        print("="*60 + "\n")
        
        # Wait a moment for connection to stabilize
        time.sleep(1)
        
        # Clear any stale data
        try:
            ser.reset_input_buffer()
        except:
            pass
        
        while True:
            try:
                if ser.in_waiting > 0:
                    try:
                        line = ser.readline().decode('utf-8', errors='ignore')
                        if line.strip():  # Only print non-empty lines
                            print(line.rstrip())
                            sys.stdout.flush()
                    except UnicodeDecodeError:
                        pass
            except OSError as e:
                if e.errno == 6:  # Device not configured
                    print("\n⚠ Device disconnected or not configured")
                    print("Try pressing RESET button on ESP32-S3")
                    time.sleep(1)
                    continue
                else:
                    raise
            time.sleep(0.01)
    
    except serial.SerialException as e:
        print(f"Error: {e}")
        print(f"\nTrying to find available ports...")
        try:
            ports = serial.tools.list_ports.comports()
            print("\nAvailable ports:")
            for p in ports:
                print(f"  {p.device} - {p.description}")
        except:
            pass
        if ser:
            ser.close()
        sys.exit(1)
    
    except KeyboardInterrupt:
        print("\n\nDisconnecting...")
        if ser:
            ser.close()
        sys.exit(0)

if __name__ == "__main__":
    main()

