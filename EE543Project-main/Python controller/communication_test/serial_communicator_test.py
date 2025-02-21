import serial
import time
import numpy as np

# define some variable
joint_num = 4
angle_max = 90
angle_min = -90
pulse_max = 440
pulse_min = 70

#clip the number without numpy
def clip_number(number, min_value, max_value):
    if number > max_value:
        return max_value
    elif number < min_value:
        return min_value
    else:
        return number

# convert the multiple joint poses in angle into pulse lengthes array
# map the angle from -90 to 90 degree to 600 till 2400 pulse length
def angle_to_pulse_length(angles):
    ret = []
    for angle in angles:
        # clip the pulse length
        angle = clip_number(angle, angle_min, angle_max)
        # convert the number into high and low bytes
        pulse_length = (int)((angle - angle_min) * (pulse_max - pulse_min)/(angle_max - angle_min) + pulse_min)
        ret.append(pulse_length)
    return ret

# convert the multiple joint poses in pulse lengthes into 8 bytes array
# format will be JP1_H,JP1_L...., unit: length count
def pulse_length_to_byte(pulse_lengthes):
    ret = []
    for pulse_length in pulse_lengthes:
        # clip the pulse length
        pulse_length = clip_number(pulse_length, pulse_min, pulse_max)
        # convert the number into high and low bytes
        pulse_length_byte = pulse_length.to_bytes(2, byteorder='big')
        ret.append(pulse_length_byte[0])
        ret.append(pulse_length_byte[1])
    return ret
    


def main():
    # Define serial port and baud rate
    serial_port = 'COM3'  # Update with your serial port
    baud_rate = 115200

    # Open serial port
    ser = serial.Serial(serial_port, baud_rate)

    # Function to send data
    def send_data(data):
        # Convert data to bytes
        # data_bytes = bytes(data)
        # Send data over serial
        ser.write(data)
        ser.flush()

    # Reset input/output buffer and wait for initialization
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(1)

    # Wait for Arduino to initialize
    while True:
        if ser.read() == b'I':
            break

    # Send signaling byte
    ser.write(b'S')
    time.sleep(0.1)
    joint_poses = [0,0,0,0,0]
    joint_vel = [0,0,0,0,1]

    # Main loop
    # counter = 0
    while True:
        start = time.time()
        # Generate 8 uint8_t numbers
        # joint_poses  = [x + y for x, y in zip(joint_poses, joint_vel)]
        joint_poses = [0,0,0,0,0]
        print(joint_poses)
        joint_pulse_lengthes = angle_to_pulse_length(joint_poses)
        # print(joint_pulse_lengthes)
        numbers = pulse_length_to_byte(joint_pulse_lengthes)
        # print(numbers)
        # time.sleep(1)

        # Poll for acknowledgement
        while ser.in_waiting == 0:
            continue
        # ser.reset_input_buffer()

        # # Send data if acknowledgement received
        if ser.read() == b'A':
            send_data(numbers)
            dur = time.time() - start
            # Wait for 1/50 seconds (50Hz)
            # time.sleep(0.02-dur)
            time.sleep(0.05-dur)#20Hz

    # Close serial port
    ser.close()

if __name__ == "__main__":
    main()
