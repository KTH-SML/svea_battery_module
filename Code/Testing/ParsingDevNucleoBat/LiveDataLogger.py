import serial
from serial import Serial, SerialException
import csv
import os
import sys
import time
from datetime import datetime

# Configuration
SERIAL_PORT = "/dev/cu.usbmodem1103"  # Replace with your microcontroller's serial port
BAUD_RATE = 115200
CSV_FILE = "readings.csv"

# Ensure the CSV file exists and has a header
if not os.path.exists(CSV_FILE):
    with open(CSV_FILE, mode="w", newline="") as file:
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


# Function to parse data and write to CSV
def parse_and_write(line):
    try:
        # Parse the line (colon-separated values)
        values = line.strip().split(":")
        if len(values) == 6:
            parsed_data = [float(v) for v in values]  # Convert all to float
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")  # Current time
            row = [timestamp] + parsed_data
            # Write to the CSV file
            with open(CSV_FILE, mode="a", newline="") as file:
                writer = csv.writer(file)
                writer.writerow(row)
                file.flush()
                os.fsync(file.fileno())
            print(f"Saved: {row}")
        else:
            print(f"Unexpected data format: {line}")
    except ValueError as e:
        print(f"Error parsing data: {line} - {e}")
    except Exception as e:
        print(f"Error writing to file: {e}")


# Main loop to read from serial and process
def main():
    ser = open_serial_port()
    try:
        print("Listening for data...")
        while True:
            try:
                if ser.in_waiting > 0:
                    line = ser.readline().decode("utf-8", errors="replace").strip()
                    if line:
                        parse_and_write(line)
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
