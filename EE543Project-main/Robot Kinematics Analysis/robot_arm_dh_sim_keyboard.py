# -*- coding: utf-8 -*-
# Inspired by ChatGPT: https://chat.openai.com/share/4a2dcf5a-2c03-4154-867f-124887dcfdc8

# This code provides visual simulation of robot arm joint and links based on the given DH parameters

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

from scipy.optimize import minimize
from sympy import symbols, cos, sin, pi, Matrix

import time
import keyboard
import sys, os


# The joint can be defined as rotational with 'r', or translational with 't', if there is an ebd-effector frame, you can also add one more row as dh parameter and joint type to be 'f' (fixed joint)
# By defination, the d or theta of the DH parameters will be variable and the other one will be constant
joint_type = ['r', 'r', 'r', 'r', 'f']
joint_pos_init = [0, 0, 0, 0]

# DH Parameters given in order of [a, alpha, d, theta] for each joint, angles should be given in degree
# If the joint is rotational, then the 4th entry will be variable and the number given here will be treat as offset of that joint. This is the same if the joint is translational so that d is variable
dh_parameters = [
    [0, 0, 62.8, 90],    # Joint 1
    [90, 0, -5.1, 90],   # Joint 2
    [-90, 105, 0, 0],   # Joint 3
    [90, 27, 4.9, 90],   # Joint 4
    [0, 0, 10, 0]        # end-effector joint, fixed
]

dh_parameters = [[0.000, 0.000, 84.504, 0],
 [90.000, 20.174, 106.928, 180],
 [45.000, 0.250, -124.481, 0.0],
 [90.000, 0.000, 0.000, 0.000],
 [0.000, 0.000, 167.800, 90.000]]

dh_parameters = [[0.000, 0.000, 84.504, 0.196],
 [90.000, 20.174, 106.928, -1.793],
 [45.000, 0.000, -124.481, 179.864],
 [90.000, 0.000, 0.000, 0.000],
 [0.000, 0.000, 167.800, 90.002]]

# 
joint_speed = [1.5, 1.5, 1.5, 1.5]

ee_speed = [1, 1, 1]

deg2rad = np.pi/180.0
rad2deg = 180.0/np.pi

plt_axis_limit = 200  # This value should be changed based on the maximum lenth of the robot arm for better visualization
plt_x_lim = [-plt_axis_limit, plt_axis_limit]
plt_y_lim = [-plt_axis_limit, plt_axis_limit]
plt_z_lim = [0, plt_axis_limit]


# Base Frame (Identity Matrix for simplicity), by defult (base_frame = np.eye(4)) it is coincide with the plot axis and origin
# If you want to change the base frame, you can change the transfromation matrix below
base_frame = np.eye(4)

def dh_to_transformation_matrix(alpha, a, d, theta):
    alpha_rad = np.radians(alpha)
    theta_rad = np.radians(theta)
    return np.array([
        [np.cos(theta_rad), -np.sin(theta_rad), 0, a],
        [np.sin(theta_rad)*np.cos(alpha_rad), np.cos(theta_rad)*np.cos(alpha_rad), -np.sin(alpha_rad), -d*np.sin(alpha_rad)],
        [np.sin(theta_rad)*np.sin(alpha_rad), np.cos(theta_rad)*np.sin(alpha_rad), np.cos(alpha_rad), d*np.cos(alpha_rad)],
        [0, 0, 0, 1]
    ])

# def update_frame(frame, dh_parameters, base_frame, joint_positions):
#     ax.clear()
#     # ax.set_xlim([-1, 1])
#     # ax.set_ylim([-1, 1])
#     # ax.set_zlim([-1, 3])
#     ax.set_xlabel('X')
#     ax.set_ylabel('Y')
#     ax.set_zlabel('Z')

#     # Starting point (base frame)
#     current_position = np.array(base_frame)
#     previous_position = np.array(base_frame)

#     for i, (alpha, a, d, theta) in enumerate(dh_parameters):
#         theta = joint_positions[i]  # Update theta with interpolated joint position
#         if joint_type[i] == 'r':
#           theta = theta + joint_positions[i]
#         elif joint_type[i] == 't':
#           d = d + joint_positions[i]
#         else:
#           print('[ERR]: Unknown joint type')
        
#         transformation_matrix = dh_to_transformation_matrix(alpha, a, d, theta)

#         # Update current position based on transformation matrix
#         current_position = previous_position @ transformation_matrix

#         # Draw link
#         ax.plot([previous_position[0, 3], current_position[0, 3]], [previous_position[1, 3], current_position[1, 3]], [previous_position[2, 3], current_position[2, 3]], 'k-')
        
#         # Draw coordinate frame axes for each link
#         ax.quiver(current_position[0, 3], current_position[1, 3], current_position[2, 3], current_position[0, 0], current_position[1, 0], current_position[2, 0], color='b', length=0.1*plt_axis_limit, width=0.005*plt_axis_limit)  # X-axis in blue
#         ax.quiver(current_position[0, 3], current_position[1, 3], current_position[2, 3], current_position[0, 2], current_position[1, 2], current_position[2, 2], color='r', length=0.1*plt_axis_limit, width=0.005*plt_axis_limit)  # Z-axis in red

#         previous_position = current_position
        
def plot_frame(frame, dh_parameters, base_frame, joint_positions):
    ax.clear()
    ax.view_init(elev=plt_elev, azim=plt_azim)
    ax.set_xlim(plt_x_lim)
    ax.set_ylim(plt_y_lim)
    ax.set_zlim(plt_z_lim)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Starting point (base frame)
    current_position = np.array(base_frame)
    previous_position = np.array(base_frame)
    
    # Draw base frame axes
    ax.quiver(current_position[0, 3], current_position[1, 3], current_position[2, 3], current_position[0, 0], current_position[1, 0], current_position[2, 0], color='y', length=0.1*plt_axis_limit)  # X-axis in blue, , length=0.1*plt_axis_limit, width=0.005*plt_axis_limit
    ax.quiver(current_position[0, 3], current_position[1, 3], current_position[2, 3], current_position[0, 2], current_position[1, 2], current_position[2, 2], color='r', length=0.1*plt_axis_limit)  # Z-axis in red

    for i, (alpha, a, d, theta) in enumerate(dh_parameters):

        if joint_type[i] == 'r':
          theta = theta + joint_positions[i]    
          jpos = str(round(joint_positions[i], 2))
        elif joint_type[i] == 't':
          d = d + joint_positions[i]
          jpos = str(round(joint_positions[i], 2))
        elif joint_type[i] == 'f':
          jpos = 'fixed'     # fixed last joint for the end-effector frame
        else:
          print('[ERR]: Unknown joint type')
        
        transformation_matrix = dh_to_transformation_matrix(alpha, a, d, theta)

        # Update current position based on transformation matrix
        current_position = previous_position @ transformation_matrix

        # Draw links
        ax.plot([previous_position[0, 3], current_position[0, 3]], [previous_position[1, 3], current_position[1, 3]], [previous_position[2, 3], current_position[2, 3]], 'k-', linewidth = 5)
        
        # Draw coordinate frame axes for each link
        ax.quiver(current_position[0, 3], current_position[1, 3], current_position[2, 3], current_position[0, 0], current_position[1, 0], current_position[2, 0], color='y', length=0.1*plt_axis_limit, label = 'Joint '+str(i+1)+' ' + jpos)  # X-axis in blue, , length=0.1*plt_axis_limit, width=0.005*plt_axis_limit
        ax.quiver(current_position[0, 3], current_position[1, 3], current_position[2, 3], current_position[0, 2], current_position[1, 2], current_position[2, 2], color='r', length=0.1*plt_axis_limit)  # Z-axis in red

        previous_position = current_position
    plt.title('End-effector Location (x, y, z): ' + str([round(current_position[0, 3],2), round(current_position[1, 3],2), round(current_position[2, 3],2)]))
    ax.legend()
    return current_position
  
  
# def forward_kinematics(joints, dh_params):
#     """
#     Compute the forward kinematics using the DH parameters and joint angles.
#     """
#     previous_position = np.array(base_frame)
#     for i, (alpha, a, d, theta) in enumerate(dh_parameters):
#         if joint_type[i] == 'r':
#           theta = theta + joint_pos_init[i]    
        
#         elif joint_type[i] == 't':
#           d = d + joint_pos_init[i]

#         elif joint_type[i] == 'f':
#           _ = 1 # do nothing
#         else:
#           print('[ERR]: Unknown joint type')
        
#         transformation_matrix = dh_to_transformation_matrix(alpha, a, d, theta)

#         # Update current position based on transformation matrix
#         current_position = previous_position @ transformation_matrix
#     return current_position
  
# symbolic forward kinematics, will be used to solve the numerical inverse kinematics 
def forward_kinematics_sym(joint_positions, dh_params):
    """
    Compute the forward kinematics using the DH parameters and joint angles.
    """
    T = Matrix.eye(4)
    for i, (alpha, a, d, theta) in enumerate(dh_parameters):
        if joint_type[i] == 'r':
          # print(len(joint_positions))
          # print(joint_positions)
          theta = theta + joint_positions[i]    
        
        elif joint_type[i] == 't':
          d = d + joint_positions[i]

        elif joint_type[i] == 'f':
          _ = 1 # do nothing
        else:
          print('[ERR]: Unknown joint type')
        # DH Transformation matrix
        T = T * Matrix([
            [cos(theta), -sin(theta), 0, a],
            [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d],
            [sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d],
            [0, 0, 0, 1]
            ])
    return T
  
def objective_function(joints, desired_pos, dh_params):
    """
    The objective function that calculates the error between the current end-effector
    position and the desired position.
    """
    # print(joints)
    fk = forward_kinematics_sym(joints, dh_params)
    # Position from forward kinematics
    current_pos = fk[:3, 3]
    # Calculate error
    error = np.linalg.norm(np.array(current_pos).astype(np.float64).flatten() - np.array(desired_pos))
    return error

def inverse_kinematics(dh_params, desired_pos, initial_guess):
    """
    Uses optimization to find the joint angles that achieve the desired end-effector position.
    """
    initial_guess = np.array(initial_guess) * deg2rad
    res = minimize(objective_function, initial_guess, args=(desired_pos, dh_params), method='BFGS')
    if res.success:
        return res.x * rad2deg
    else:
        print("Inverse kinematics did not converge")
        return initial_guess * rad2deg



# If you would like to add more joints, you can modify the following code

def print_manu():
    print('  ')
    print('-----------------------------------------')
    print('EE543 Arm DH Simulation Keyboard Controller:')
    
    print('Joint Control')
    print('[Exit]: 9')
    print('[Joint 1 +]: 1 | [Joint 1 -]: q')
    print('[Joint 2 +]: 2 | [Joint 2 -]: w')
    print('[Joint 3 +]: 3 | [Joint 3 -]: e')
    print('[Joint 4 +]: 4 | [Joint 4 -]: r')
    print('-----------------------------------------')
    print('End-effector Control')
    print('[End-effector X +]: l | [End-effector X -]: .')
    print('[End-effector Y +]: , | [End-effector Y -]: /')
    print('[End-effector Z +]: k | [End-effector Z -]: m')
    print('-----------------------------------------')
    print('View Control')
    print('[View Elev +]: s  | [View Elev -]: x')
    print('[View Azim +]: z  | [View Azim -]: c')

    print('-----------------------------------------')
    print('-----------------------------------------')
    print('Current command:\n')
    return None

def print_no_newline(string):
    sys.stdout.write("\r" + string)
    sys.stdout.flush()
    return None

previous_position = np.array(base_frame)
# init the end-effector position based on the initial joint positions
for i, (alpha, a, d, theta) in enumerate(dh_parameters):

    if joint_type[i] == 'r':
      theta = theta + joint_pos_init[i]    
      jpos = str(round(joint_pos_init[i], 2))
    elif joint_type[i] == 't':
      d = d + joint_pos_init[i]
      jpos = str(round(joint_pos_init[i], 2))
    elif joint_type[i] == 'f':
      jpos = 'fixed'     # fixed last joint for the end-effector frame
    else:
      print('[ERR]: Unknown joint type')
    
    transformation_matrix = dh_to_transformation_matrix(alpha, a, d, theta)

    # Update current position based on transformation matrix
    current_position = previous_position @ transformation_matrix
  
ee_pos_cur = current_position[:3,3]*1.0 # initialize the current end-effector position
ee_pos_d = current_position[:3,3]*1.0 # initialize the desired end-effector position 
# print('initial ee_pos_cur: 9', ee_pos_cur)

# Setup the plot
fig = plt.figure(figsize=(15,15))
fig.clf()
ax = fig.add_subplot(111, projection='3d')

working = 1
command = False
ik_command = False   
print_manu()

plt_elev = 45
plt_azim = 45

joint_positions = joint_pos_init

# ani = FuncAnimation(fig, update_frame, frames=1, fargs=(dh_parameters, base_frame, joint_positions), interval=100)
frame = 0
plt.show()
while working==1:

    #get the keyboard input
    input_key = keyboard.read_event().name

    if input_key == '9':
        # RC.communication_end()
        os.system('cls' if os.name == 'nt' else 'clear')
        sys.exit('Closing Keyboard controller')
        

    elif input_key == '1':
        print_no_newline(" Moving: Joint 1 +++         ")
        joint_positions[0] += joint_speed[0]
        command = True


    elif input_key == 'q':
        print_no_newline(" Moving: Joint 1 ---         ")
        joint_positions[0] -= joint_speed[0]
        command = True

    elif input_key == '2':
        print_no_newline(" Moving: Joint 2 +++         ")
        joint_positions[1] += joint_speed[1]
        command = True
              
    elif input_key == 'w':
        print_no_newline(" Moving: Joint 2 ---         ")
        joint_positions[1] -= joint_speed[1]
        command = True

    elif input_key == '3':
        print_no_newline(" Moving: Joint 3 +++         ")
        joint_positions[2] += joint_speed[2]
        command = True
        
    elif input_key == 'e':
        print_no_newline(" Moving: Joint 3 ---         ")
        joint_positions[2] -= joint_speed[2]
        command = True

    elif input_key == '4':
        print_no_newline(" Moving: Joint 4 +++         ")
        joint_positions[3] += joint_speed[3]
        command = True
        
    elif input_key == 'r':
        print_no_newline(" Moving: Joint 4 ---         ")
        joint_positions[3] -= joint_speed[3]
        command = True
    
    elif input_key == 'h':
        print_no_newline(" Homing....                  ")
        joint_positions = [0, 0, 0, 0]
        command = True
        
    #Keys for cartesian based control-----------------------------------------
    elif input_key == 'l':
        print_no_newline(" Moving: x +++              ")
        ee_pos_d = ee_pos_cur + [ee_speed[0], 0, 0]
        print('ee_pos_cur', ee_pos_cur)
        print('ee_pos_d', ee_pos_d)
        command = True
        ik_command = True
    elif input_key == '.':
        print_no_newline(" Moving: x ---              ")
        ee_pos_d = ee_pos_cur - [ee_speed[0], 0, 0]
        command = True
        ik_command = True
    elif input_key == ',':
        print_no_newline(" Moving: y +++              ")
        ee_pos_d = ee_pos_cur + [0, ee_speed[1], 0]
        command = True
        ik_command = True
    elif input_key == '/':
        print_no_newline(" Moving: y ---              ")
        ee_pos_d = ee_pos_cur - [0, ee_speed[1], 0]
        command = True
        ik_command = True
    elif input_key == 'k':
        print_no_newline(" Moving: z +++              ")
        ee_pos_d = ee_pos_cur + [0, 0, ee_speed[2]]
        command = True
        ik_command = True
    elif input_key == 'm':
        print_no_newline(" Moving: z ---              ")
        ee_pos_d = ee_pos_cur - [0, 0, ee_speed[2]]
        command = True
        ik_command = True
        
        
        
    # keys to adjust view angle    
    elif input_key == 's':
        print_no_newline(" View Elev +++                  ")
        plt_elev += 5
        command = True
      
    elif input_key == 'x':
        print_no_newline(" View Elev ---                  ")
        plt_elev -= 5
        command = True
        
    elif input_key == 'z':
        print_no_newline(" View Azim +++                  ")
        plt_azim += 5
        command = True
        
    elif input_key == 'c':
        print_no_newline(" View Azim ---                  ")
        plt_azim -= 5
        command = True
        

    else:
        print_no_newline(' Unknown command             ')

    
    if command:
        # make sure the goals is within joint limit
        print('-----------------------')

        sys.stdout.write("\033[1B") # move curser down
        sys.stdout.write("\033[1A") # move curser up
        if ik_command:
          # print('desired pose')
          # print(ee_pos_d)
          joint_positions = inverse_kinematics(dh_parameters, ee_pos_d, joint_positions)
          print('IK solution')
          print(joint_positions)
          ik_command = False
        command = False
        # ani = FuncAnimation(fig, update_frame, frames=1, fargs=(dh_parameters, base_frame, joint_positions), interval=100)
        ee_pos_cur = plot_frame(frame, dh_parameters, base_frame, joint_positions)[:3,3]
        # print('current pose')
        # print(ee_pos_cur)
        plt.show(block = False)
        plt.pause(0.01)
        frame += 1