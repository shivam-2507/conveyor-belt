#!/usr/bin/env python3
# pip install -r requirements.txt
"""
Serial Data Logger for Conveyor Belt Measurements
Receives data from ESP32 and logs to CSV file
"""

import serial
import json
import csv
import time
from datetime import datetime
import sys

# Configuration
SERIAL_PORT = '/dev/cu.usbserial-110'  # Change this to your ESP32 port
BAUD_RATE = 115200
CSV_FILENAME = 'conveyor_measurements.csv'


def find_esp32_port():
    """Helper function to find ESP32 port (macOS)"""
    import serial.tools.list_ports
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if 'usbserial' in port.device or 'USB' in port.device:
            print(f"Found potential ESP32 at: {port.device}")
            return port.device
    return None


def init_csv(filename):
    """Initialize CSV file with headers"""
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Date', 'Time', 'Timestamp_ms',
                        'Time_Diff_ms', 'Length_cm', 'RPM'])
    print(f"✅ CSV file initialized: {filename}")


def log_to_csv(filename, data):
    """Append measurement data to CSV"""
    now = datetime.now()
    with open(filename, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([
            now.strftime('%Y-%m-%d'),
            now.strftime('%H:%M:%S'),
            data['timestamp'],
            data['time_diff_ms'],
            data['length_cm'],
            data['rpm']
        ])
    print(
        f"📝 Logged: {data['length_cm']:.2f} cm, Time: {data['time_diff_ms']} ms")


def main():
    # Try to find ESP32 port automatically
    port = find_esp32_port()
    if not port:
        port = SERIAL_PORT
        print(f"Using default port: {port}")

    # Initialize CSV file
    init_csv(CSV_FILENAME)

    try:
        # Open serial connection
        print(f"🔌 Connecting to {port} at {BAUD_RATE} baud...")
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
        time.sleep(2)  # Wait for connection to stabilize
        print("✅ Connected to ESP32")
        print("📊 Logging data to CSV... (Press Ctrl+C to stop)\n")

        data_started = False

        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()

                # Look for data markers
                if line == "DATA_START":
                    data_started = True
                elif line == "DATA_END":
                    data_started = False
                elif data_started and line.startswith('{'):
                    # Parse JSON data
                    try:
                        data = json.loads(line)
                        log_to_csv(CSV_FILENAME, data)
                    except json.JSONDecodeError as e:
                        print(f"❌ JSON parse error: {e}")
                else:
                    # Print other serial output (for debugging)
                    if line and not line.startswith('Distance'):
                        print(f"ESP32: {line}")

    except serial.SerialException as e:
        print(f"❌ Serial error: {e}")
        print(f"\nTip: Check your port with: ls /dev/cu.*")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n\n👋 Stopping data logger...")
        ser.close()
        print(f"✅ Data saved to {CSV_FILENAME}")
        sys.exit(0)


if __name__ == "__main__":
    main()
