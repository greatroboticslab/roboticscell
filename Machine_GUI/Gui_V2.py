import tkinter as tk
from tkinter.scrolledtext import ScrolledText
import serial
import threading
import time
import os
import datetime
import subprocess

# === SERIAL CONFIGURATION ===
SERIAL_PORT = 'COM3'  # Replace with your Teensy's port
BAUD_RATE = 9600      # Match your Teensy's Serial.begin()

# === FILE LOGGING ===
timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
LOG_FILE = os.path.expanduser(f"~/Desktop/Log_Files/serial_output_{timestamp}.log")

# === FUNCTION TO OPEN INSIGHT EXPLORER ===
def open_cognex():
    insight_path = r"C:\Program Files (x86)\Cognex\In-Sight\In-Sight Explorer 6.5.1\In-Sight Explorer.exe"
    try:
        subprocess.Popen([insight_path])
    except FileNotFoundError:
        print("Insight Explorer executable not found! Check the path.")

def open_arduino_IDE():
    insight_path = r"C:\Users\TaterMan\AppData\Local\Programs\Arduino IDE\Arduino IDE.exe"
    try:
        subprocess.Popen([insight_path])
    except FileNotFoundError:
        print("Arduino IDE executable not found! Check the path.")

# === GUI SETUP ===
class SerialApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Teensy Serial Monitor")
        self.root.geometry("1200x400")  # Wider window for side-by-side layout

        # Left frame for serial output text area
        left_frame = tk.Frame(root)
        left_frame.pack(side='left', fill='both', expand=True)

        self.text_area = ScrolledText(left_frame, wrap='word', font=('Courier', 10))
        self.text_area.pack(fill='both', expand=True, padx=5, pady=5)

        # Right frame for buttons
        right_frame = tk.Frame(root, width=180)
        right_frame.pack(side='right', fill='y', padx=5, pady=5)

        self.open_button = tk.Button(right_frame, text="Open Cognex Insight Explorer", command=open_cognex)
        self.open_button.pack(fill='x', pady=(0, 10))

        self.open_button = tk.Button(right_frame, text="Open Arduino IDE", command=open_arduino_IDE)
        self.open_button.pack(fill='x', pady=(0, 10))

        # You can add more buttons here if you want
        # Example:
        # self.another_button = tk.Button(right_frame, text="Another Button", command=self.another_func)
        # self.another_button.pack(fill='x', pady=(0,10))

        # Open log file
        self.log_file = open(LOG_FILE, "a")

        # Serial read thread
        self.running = True
        self.thread = threading.Thread(target=self.read_serial)
        self.thread.daemon = True  # So thread exits when main app closes
        self.thread.start()

        # Handle window close event
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def read_serial(self):
        try:
            with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
                while self.running:
                    if ser.in_waiting:
                        line = ser.readline().decode('utf-8', errors='replace').strip()
                        if line:
                            self.text_area.insert('end', line + '\n')
                            self.text_area.see('end')
                            self.log_file.write(line + '\n')
                            self.log_file.flush()
                    time.sleep(0.1)
        except serial.SerialException as e:
            self.text_area.insert('end', f"[Error] {e}\n")

    def on_close(self):
        self.running = False
        self.thread.join(timeout=2)
        self.log_file.close()
        self.root.destroy()

# === MAIN ===
if __name__ == "__main__":
    root = tk.Tk()
    app = SerialApp(root)
    root.mainloop()
