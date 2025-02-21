import serial
import time
import numpy as np
from robot_controller import robot_controller

#create robot object
RC = robot_controller()

def main():
    # Define and open serial port
    ser = serial.Serial(RC.com_port, RC.com_baudrate)

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

    # Main loop
    while True:
        start = time.time()
        # Generate 8 uint8_t numbers
        joint_poses = RC.get_joint_pose_update()
        # print(joint_poses)
        joint_pulse_lengthes = RC.angle_to_pulse_length(joint_poses)
        # print(joint_pulse_lengthes)
        numbers = RC.pulse_length_to_byte(joint_pulse_lengthes)
        # print(numbers)


        # Poll for acknowledgement
        while ser.in_waiting == 0:
            continue
        # ser.reset_input_buffer()

        # # Send data if acknowledgement received
        if ser.read() == b'A':
            send_data(numbers)
            dur = time.time() - start
            time.sleep((1/RC.com_frequency)-dur)#20Hz

    # Close serial port
    ser.close()

if __name__ == "__main__":
    main()
