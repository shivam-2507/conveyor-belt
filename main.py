#!/usr/bin/env python3
# pip install -r requirements.txt
"""
Serial Data Logger for Conveyor Belt Measurements
Receives data from ESP32 and logs to CSV file with enhanced error handling
"""

import serial
import serial.tools.list_ports
import json
import csv
import time
from datetime import datetime
import sys
import argparse
from pathlib import Path

# Configuration
SERIAL_PORT = '/dev/cu.usbserial-110'  # Default port (macOS)
BAUD_RATE = 115200
CSV_FILENAME = 'conveyor_measurements.csv'
VERBOSE_DEBUG = False  # Set to True to see all serial output


class ConveyorLogger:
    """Enhanced logger for ESP32 conveyor belt measurements"""

    def __init__(self, port, baud_rate, csv_filename, verbose=False):
        self.port = port
        self.baud_rate = baud_rate
        self.csv_filename = csv_filename
        self.verbose = verbose
        self.serial_conn = None

        # Statistics tracking
        self.stats = {
            'measurements': 0,
            'json_errors': 0,
            'timer1_events': 0,
            'timer2_events': 0,
            'spike_detections': 0,
            'total_length': 0.0,
            'min_length': float('inf'),
            'max_length': 0.0
        }

        # State machine
        self.data_capture_active = False
        self.json_buffer = ""

    def find_esp32_port(self):
        """Find ESP32 port automatically (cross-platform)"""
        ports = serial.tools.list_ports.comports()

        for port in ports:
            # Check for common ESP32 identifiers
            if any(keyword in port.device.lower() for keyword in ['usbserial', 'usb', 'slab', 'ch340', 'cp210']):
                print(f"🔍 Found potential ESP32 at: {port.device}")
                if port.description:
                    print(f"   Description: {port.description}")
                return port.device

        print("⚠️  No ESP32 detected automatically")
        return None

    def init_csv(self):
        """Initialize CSV file with headers"""
        file_path = Path(self.csv_filename)

        # Check if file exists and prompt user
        if file_path.exists():
            response = input(
                f"⚠️  {self.csv_filename} already exists. Overwrite? (y/n): ")
            if response.lower() != 'y':
                # Append timestamp to filename
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                self.csv_filename = f"conveyor_measurements_{timestamp}.csv"
                print(f"📝 Using new filename: {self.csv_filename}")

        with open(self.csv_filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Date', 'Time', 'Python_Timestamp', 'ESP32_Timestamp_ms',
                           'Time_Diff_ms', 'Length_mm', 'RPM', 'Belt_Speed_mm_s'])
        print(f"✅ CSV file initialized: {self.csv_filename}")
    
    def log_to_csv(self, data):
        """Append measurement data to CSV with validation"""
        try:
            now = datetime.now()

            # Calculate belt speed for additional analysis
            time_seconds = data['time_diff_ms'] / 1000.0
            belt_speed = data['length_mm'] / time_seconds if time_seconds > 0 else 0

            with open(self.csv_filename, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    now.strftime('%Y-%m-%d'),
                    now.strftime('%H:%M:%S.%f')[:-3],  # Millisecond precision
                    now.timestamp(),
                    data['timestamp'],
                    data['time_diff_ms'],
                    f"{data['length_mm']:.6f}",  # Match ESP32 precision
                    f"{data['rpm']:.2f}",
                    f"{belt_speed:.2f}"
                ])

            # Update statistics
            self.stats['measurements'] += 1
            self.stats['total_length'] += data['length_mm']
            self.stats['min_length'] = min(self.stats['min_length'], data['length_mm'])
            self.stats['max_length'] = max(self.stats['max_length'], data['length_mm'])

            print(f"📝 [{self.stats['measurements']}] Logged: {data['length_mm']:.4f} mm | "
                  f"Time: {data['time_diff_ms']} ms | Speed: {belt_speed:.2f} mm/s")

        except Exception as e:
            print(f"❌ Error writing to CSV: {e}")

    def parse_json_data(self, line):
        """Parse JSON data from ESP32 with robust error handling"""
        try:
            data = json.loads(line)

            # Validate required fields
            required_fields = ['timestamp', 'time_diff_ms', 'length_mm', 'rpm']
            if all(field in data for field in required_fields):
                # Sanity checks
                if data['length_mm'] < 0:
                    print(f"⚠️  Warning: Negative length detected: {data['length_mm']} mm")
                if data['time_diff_ms'] < 0:
                    print(f"⚠️  Warning: Negative time diff: {data['time_diff_ms']} ms")

                self.log_to_csv(data)
                return True
            else:
                missing = [f for f in required_fields if f not in data]
                print(f"⚠️  Incomplete JSON data, missing: {missing}")
                self.stats['json_errors'] += 1
                return False

        except json.JSONDecodeError as e:
            print(f"❌ JSON parse error at position {e.pos}: {e.msg}")
            if self.verbose:
                print(f"   Raw line: {line}")
            self.stats['json_errors'] += 1
            return False

    def handle_esp32_message(self, line):
        """Process ESP32 debug/status messages"""
        line_lower = line.lower()

        # Track events for statistics
        if "timer1 started" in line_lower:
            self.stats['timer1_events'] += 1
            print(f"🟢 {line}")
        elif "timer2 started" in line_lower or "warning" in line_lower:
            self.stats['timer2_events'] += 1
            print(f"⚠️  {line}")
        elif "spike detected" in line_lower:
            self.stats['spike_detections'] += 1
            print(f"🔄 {line}")
        elif "system initialized" in line_lower or "motor running" in line_lower:
            print(f"✅ {line}")
        elif line.startswith("==="):
            # Measurement summary separator
            print(line)
        elif "object detection time" in line_lower or "calculated belt length" in line_lower:
            # Measurement details
            print(f"📊 {line}")
        elif self.verbose and line and not line.startswith("Distance (mm):"):
            # Show all other output in verbose mode, except distance spam
            print(f"ESP32: {line}")

    def print_statistics(self):
        """Display session statistics"""
        print("\n" + "="*60)
        print("📊 SESSION STATISTICS")
        print("="*60)
        print(f"Total measurements logged: {self.stats['measurements']}")
        print(
            f"Timer1 events (objects detected): {self.stats['timer1_events']}")
        print(f"Timer2 events (object exits): {self.stats['timer2_events']}")
        print(f"Spike detections: {self.stats['spike_detections']}")
        print(f"JSON parse errors: {self.stats['json_errors']}")

        if self.stats['measurements'] > 0:
            avg_length = self.stats['total_length'] / self.stats['measurements']
            print(f"\nLength statistics:")
            print(f"  Average: {avg_length:.4f} mm")
            print(f"  Min: {self.stats['min_length']:.4f} mm")
            print(f"  Max: {self.stats['max_length']:.4f} mm")
            print(f"  Range: {self.stats['max_length'] - self.stats['min_length']:.4f} mm")

        print(f"\nData saved to: {self.csv_filename}")
        print("="*60 + "\n")

    def run(self):
        """Main logging loop with enhanced error handling"""
        try:
            # Try to find ESP32 port automatically
            port = self.find_esp32_port()
            if not port:
                port = self.port
                print(f"Using configured port: {port}")
            else:
                self.port = port

            # Initialize CSV file
            self.init_csv()

            # Open serial connection
            print(f"🔌 Connecting to {self.port} at {self.baud_rate} baud...")
            self.serial_conn = serial.Serial(
                self.port, self.baud_rate, timeout=1)
            time.sleep(2)  # Wait for ESP32 to stabilize

            # Flush initial buffer
            self.serial_conn.reset_input_buffer()

            print("✅ Connected to ESP32")
            print("📊 Logging measurements to CSV...")
            print(f"💡 Verbose mode: {'ON' if self.verbose else 'OFF'}")
            print("Press Ctrl+C to stop\n")

            while True:
                if self.serial_conn.in_waiting > 0:
                    try:
                        line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()

                        if not line:
                            continue

                        # State machine for JSON data capture
                        if line == "DATA_START":
                            self.data_capture_active = True
                            self.json_buffer = ""

                        elif line == "DATA_END":
                            if self.data_capture_active and self.json_buffer:
                                # Process captured JSON
                                self.parse_json_data(self.json_buffer)
                            self.data_capture_active = False
                            self.json_buffer = ""

                        elif self.data_capture_active:
                            # Accumulate JSON data (handles multi-line if needed)
                            self.json_buffer += line

                        else:
                            # Handle ESP32 debug/status messages
                            self.handle_esp32_message(line)

                    except UnicodeDecodeError as e:
                        if self.verbose:
                            print(f"⚠️  Unicode decode error: {e}")
                        continue

                time.sleep(0.01)  # Small delay to prevent CPU overload

        except serial.SerialException as e:
            print(f"\n❌ Serial error: {e}")
            print("\nTroubleshooting tips:")
            print("1. Check if ESP32 is connected")
            print("2. Verify the correct port:")
            print("   macOS: ls /dev/cu.*")
            print("   Windows: Check Device Manager")
            print("   Linux: ls /dev/ttyUSB* /dev/ttyACM*")
            print("3. Close other programs using the serial port")

            # List available ports
            print("\n📋 Available serial ports:")
            ports = serial.tools.list_ports.comports()
            if ports:
                for port in ports:
                    print(f"   - {port.device}: {port.description}")
            else:
                print("   No serial ports found")

            return 1

        except KeyboardInterrupt:
            print("\n\n👋 Stopping data logger...")
            return 0

        except Exception as e:
            print(f"\n❌ Unexpected error: {e}")
            import traceback
            traceback.print_exc()
            return 1

        finally:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
                print("🔌 Serial connection closed")

            self.print_statistics()


def main():
    """Entry point with argument parsing"""
    parser = argparse.ArgumentParser(
        description='ESP32 Conveyor Belt Data Logger',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python main.py                           # Auto-detect port
  python main.py --port COM3               # Specify Windows port
  python main.py --port /dev/cu.usbserial-0 # Specify macOS port
  python main.py --verbose                 # Show all debug output
  python main.py --output measurements.csv # Custom output file
        """
    )

    parser.add_argument('--port', '-p',
                        default=SERIAL_PORT,
                        help='Serial port (default: auto-detect)')

    parser.add_argument('--baud', '-b',
                        type=int,
                        default=BAUD_RATE,
                        help=f'Baud rate (default: {BAUD_RATE})')

    parser.add_argument('--output', '-o',
                        default=CSV_FILENAME,
                        help=f'Output CSV filename (default: {CSV_FILENAME})')

    parser.add_argument('--verbose', '-v',
                        action='store_true',
                        help='Show all serial output including distance readings')

    args = parser.parse_args()

    # Create logger instance
    logger = ConveyorLogger(
        port=args.port,
        baud_rate=args.baud,
        csv_filename=args.output,
        verbose=args.verbose
    )

    # Run the logger
    exit_code = logger.run()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
