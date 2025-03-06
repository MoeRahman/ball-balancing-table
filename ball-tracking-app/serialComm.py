

import serial
import time

ser = serial.Serial(port='COM3', baudrate=9600, timeout=0.1)
ser.close()
time.sleep(2)
ser.open()

while True:
    if ser.in_waiting > 0:  # Check if there is data available to read
        line = ser.readline().decode('utf-8').strip()  # Read the line and decode it
        print("Received line:", line)
        try:
            # Split the string by commas and convert to floats
            float_values = [float(val) for val in line.split(',')]
            print("Parsed values:", float_values)
        except ValueError:
            print("Error parsing float values")
    time.sleep(1)
