# Inspired by ChatGPT: https://chat.openai.com/share/4a2dcf5a-2c03-4154-867f-124887dcfdc8

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

def dh_to_transformation_matrix(alpha, a, d, theta):
    alpha_rad = np.radians(alpha)
    theta_rad = np.radians(theta)
    return np.array([
        [np.cos(theta_rad), -np.sin(theta_rad), 0, a],
        [np.sin(theta_rad)*np.cos(alpha_rad), np.cos(theta_rad)*np.cos(alpha_rad), -np.sin(alpha_rad), -d*np.sin(alpha_rad)],
        [np.sin(theta_rad)*np.sin(alpha_rad), np.cos(theta_rad)*np.sin(alpha_rad), np.cos(alpha_rad), d*np.cos(alpha_rad)],
        [0, 0, 0, 1]
    ])

def update_frame(frame, dh_parameters, base_frame, total_frames, initial_positions, final_positions):
    ax.clear()
    # ax.set_xlim([-1, 1])
    # ax.set_ylim([-1, 1])
    # ax.set_zlim([-1, 3])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Interpolate joint positions
    joint_positions = initial_positions + (final_positions - initial_positions) * frame / total_frames

    # Starting point (base frame)
    current_position = np.array(base_frame)
    previous_position = np.array(base_frame)

    for i, (alpha, a, d, _) in enumerate(dh_parameters):
        theta = joint_positions[i]  # Update theta with interpolated joint position
        transformation_matrix = dh_to_transformation_matrix(alpha, a, d, theta)

        # Update current position based on transformation matrix
        current_position = previous_position @ transformation_matrix

        # Draw link
        ax.plot([previous_position[0, 3], current_position[0, 3]], [previous_position[1, 3], current_position[1, 3]], [previous_position[2, 3], current_position[2, 3]], 'k-')
        
        # Draw coordinate frame axes for each link
        ax.quiver(current_position[0, 3], current_position[1, 3], current_position[2, 3], current_position[0, 0], current_position[1, 0], current_position[2, 0], color='b', length=0.1)  # X-axis in blue
        ax.quiver(current_position[0, 3], current_position[1, 3], current_position[2, 3], current_position[0, 2], current_position[1, 2], current_position[2, 2], color='r', length=0.1)  # Z-axis in red

        previous_position = current_position

# Example DH Parameters: [a, alpha, d, theta (will be replaced by joint_positions)]
dh_parameters = [
    [0, 0, 62.8, 0],  # Joint 1
    [90, 0, -5.1, 0],   # Joint 2
    [-90, 105, 0, 0],   # Joint 3
    [90, 27, 4.9, 0],
    [90, 0, 10, 0]
]

# Initial and Final Joint Positions (in degrees)
# initial_positions = np.array([0, 0, 0, 0, 0,]) + np.array([90, 90, 0, -90, 0,])
# final_positions = np.array([90, 90, 90, 90, 90]) + np.array([90, 90, 0, -90, 0,])
initial_positions = np.array([45, 45, 45, 45, 45]) + np.array([90, 90, 0, 90, 0,])
final_positions = np.array([45, 45, 45, 45, 45]) + np.array([90, 90, 0, 90, 0,])

# Base Frame (Identity Matrix for simplicity)
base_frame = np.eye(4)

# Setup the plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Number of frames for the animation
total_frames = 100

# Creating the animation
ani = FuncAnimation(fig, update_frame, frames=total_frames, fargs=(dh_parameters, base_frame, total_frames, initial_positions, final_positions), interval=100)

plt.show()
