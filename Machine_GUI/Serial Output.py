import serial
import os
import platform
import time

# === SERIAL CONFIGURATION ===
SERIAL_PORT = 'COM3'  # Default port to check for on Windows (replace as needed)
USB_PATHS = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0']  # List of USB paths to try on Linux/macOS

BAUD_RATE = 9600  # Match your Teensy's Serial.begin()

def check_and_open_serial_port():
    """Tries to open a serial port based on the platform and available devices."""
    if platform.system() == 'Windows':
        return open_serial_port_windows(SERIAL_PORT)
    elif platform.system() in ['Linux', 'Darwin']:  # Darwin is macOS
        return open_serial_port_linux_mac(USB_PATHS)
    else:
        raise Exception("Unsupported OS")

def open_serial_port_windows(port):
    """Opens the serial port on Windows (COMx)."""
    try:
        print(f"Attempting to open {port} on Windows...")
        ser = serial.Serial(port, baudrate=BAUD_RATE, timeout=1)
        return ser
    except serial.SerialException as e:
        print(f"[Error] Could not open serial port {port}: {e}")
        return None

def open_serial_port_linux_mac(paths):
    """Opens the serial port on Linux/macOS (e.g., /dev/ttyUSB0)."""
    for path in paths:
        try:
            print(f"Attempting to open {path} on Linux/macOS...")
            if os.path.exists(path):
                ser = serial.Serial(path, baudrate=BAUD_RATE, timeout=1)
                return ser
            else:
                print(f"[Error] USB device {path} not found.")
        except serial.SerialException as e:
            print(f"[Error] Could not open USB device {path}: {e}")
    return None

def read_from_serial():
    """Reads data from the serial port and prints it."""
    ser = check_and_open_serial_port()  # Try opening either COM or USB port
    if ser:
        try:
            while True:
                if ser.in_waiting > 0:
                    data = ser.readline().decode('utf-8', errors='replace').strip()
                    if data:
                        print(data)  # Output to console or log file
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("Exiting program.")
        finally:
            ser.close()  # Always close the serial port when done
    else:
        print("[Error] Unable to open any serial port.")

if __name__ == "__main__":
    read_from_serial()
