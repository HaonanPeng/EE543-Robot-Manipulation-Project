import serial
import time

# Define serial port and baud rate
serial_port = 'COM3'  # Update with your serial port
baud_rate = 9600

# Open serial port
ser = serial.Serial(serial_port, baud_rate)

#reset input/output buffer
ser.reset_input_buffer()
ser.reset_output_buffer()
time.sleep(1)

# wait for arduino to initialize
while True:
    if ser.read() == b'I':
        break
ser.write(b'S')
# time.sleep(1)
while True:
    #poll for the acknoledgement 
    if ser.read() == b'A':
        
        ser.write(b'D')
        time.sleep(0.01)

# Close serial port
ser.close()