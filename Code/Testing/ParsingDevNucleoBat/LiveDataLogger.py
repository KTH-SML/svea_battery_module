import serial
from serial import Serial, SerialException
import csv
import os
import sys
import time
from datetime import datetime

# Configuration
SERIAL_PORT = "/dev/cu.usbmodem1103"  # Replace with your microcontroller's serial port
# SERIAL_PORT = "/dev/ttyACM2" on svea
BAUD_RATE = 115200
BATTERY_CSV_FILE = "battery_readings.csv"
CHARGER_CSV_FILE = "charger_readings.csv"

# Ensure the CSV files exist and have headers
if not os.path.exists(BATTERY_CSV_FILE):
    with open(BATTERY_CSV_FILE, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(
            [
                "Timestamp",
                "BusVoltage",
                "ShuntVoltage",
                "Current",
                "Power",
                "SoC_Sigmoidal",
                "SoC_AsymmetricSigmoidal",
            ]
        )

if not os.path.exists(CHARGER_CSV_FILE):
    with open(CHARGER_CSV_FILE, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(
            [
                "Timestamp",
                "BusVoltage",
                "ShuntVoltage",
                "Current",
                "Power",
            ]
        )

# Initialize variables to store readings
battery_reading = None  # Will store (timestamp, data_values)
charger_reading = None


# Function to open serial port
def open_serial_port():
    while True:
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
            return ser
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            print("Retrying in 5 seconds...")
            time.sleep(5)


# Function to parse a line and return sensor name and data values
def parse_line(line):
    try:
        # Ignore lines that are error messages or do not start with expected sensor names
        if not (line.startswith("Battery:") or line.startswith("Charger:")):
            print(f"Ignoring irrelevant line: {line}")
            return None, None

        # Parse the line (colon-separated values)
        values = line.strip().split(":")
        if len(values) >= 2:
            sensor_name = values[0]
            data_values = values[1:]
            return sensor_name, data_values
        else:
            print(f"Unexpected data format: {line}")
            return None, None
    except Exception as e:
        print(f"Error parsing line: {line} - {e}")
        return None, None


# Function to write data to CSV file
def write_to_csv(csv_file, timestamp, data_values):
    try:
        parsed_data = [float(v) for v in data_values]
        row = [timestamp] + parsed_data
        with open(csv_file, mode="a", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(row)
            file.flush()
            os.fsync(file.fileno())
        print(f"Saved to {csv_file}: {row}")
    except ValueError as e:
        print(f"Error parsing data: {data_values} - {e}")
    except Exception as e:
        print(f"Error writing to file: {e}")


# Main loop to read from serial and process
def main():
    ser = open_serial_port()
    global battery_reading, charger_reading
    try:
        print("Listening for data...")
        while True:
            try:
                if ser.in_waiting > 0:
                    line = ser.readline().decode("utf-8", errors="replace").strip()
                    if line:
                        sensor_name, data_values = parse_line(line)
                        if sensor_name and data_values:
                            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[
                                :-3
                            ]
                            if sensor_name == "Battery":
                                battery_reading = (timestamp, data_values)
                            elif sensor_name == "Charger":
                                charger_reading = (timestamp, data_values)
                            # Check if both readings are available
                            if battery_reading and charger_reading:
                                # Use the earlier timestamp for both readings
                                common_timestamp = min(
                                    battery_reading[0], charger_reading[0]
                                )
                                # Write to battery CSV
                                write_to_csv(
                                    BATTERY_CSV_FILE,
                                    common_timestamp,
                                    battery_reading[1],
                                )
                                # Write to charger CSV
                                write_to_csv(
                                    CHARGER_CSV_FILE,
                                    common_timestamp,
                                    charger_reading[1],
                                )
                                # Reset readings
                                battery_reading = None
                                charger_reading = None
                        else:
                            # Ignore irrelevant or malformed lines
                            pass
            except serial.SerialException as e:
                print(f"Serial exception: {e}")
                ser.close()
                print("Attempting to reopen serial port...")
                ser = open_serial_port()
            except Exception as e:
                print(f"Error reading from serial port: {e}")
    except KeyboardInterrupt:
        print("\nTerminating program.")
    finally:
        ser.close()
        print("Serial port closed.")


if __name__ == "__main__":
    main()
