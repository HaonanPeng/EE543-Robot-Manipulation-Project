# -*- coding: utf-8 -*-
"""
Created on Mon Apr  1 11:02:23 2024

@author: 75678
"""
import numpy as np
import math
import random
import matplotlib.pyplot as plt
from scipy.spatial import KDTree


# The joint can be defined as rotational with 'r', or translational with 't', or fixed end-effector frame with 'f'
# if the last frame is fixed end-effector frame, no need to give joint limits since it is not acutally a joint
# By defination, the d or theta of the DH parameters will be variable and the other one will be constant
joint_type = ['r', 'r', 'r', 'r', 'f']


# define joint limits
joint_limits = [(-90, 90), (-90, 90), (-90, 90), (-90, 90)] # In degree, here we assume all joints are rotational, but we can also define translational joints and give limits in lenth

# DH Parameters given in order of [a, alpha, d, theta] for each joint, angles should be given in degree
# If the joint is rotational, then the 4th entry will be variable and the number given here will be treat as offset of that joint. This is the same if the joint is translational so that d is variable
# dh_parameters = [
#     [0, 0, 62.8, 90],    # Joint 1
#     [90, 0, -5.1, 90],   # Joint 2
#     [-90, 105, 0, 0],   # Joint 3
#     [90, 27, 4.9, 90]   # Joint 4
# ]

dh_parameters = [[0.000, 0.000, 84.504, 0.196],
 [90.000, 20.174, 106.928, 178.207],
 [-45.000, 0.250, -124.481, -0.136],
 [90.000, 0.000, 0.000, 0.000],
 [0.000, 0.000, 167.800, 90.002]]

# define test points in the defult world frame, the simulation will also test if these points are likely within the robot workspace. 
test_points = [[60, 0, 0],
              [0, 70, 0],
              [110, 20, 90]]

# the number of samples of monte-carlo simulation, the more samples, the more accurate workspace we will have, but it also takes longer time
num_samples = 10000

plt_axis_limit = 100  # This value should be changed based on the maximum lenth of the robot arm for better visualization
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

def arm_FK(joint_positions, dh_parameters, base_frame):

    # Starting point (base frame)
    current_position = np.array(base_frame)
    previous_position = np.array(base_frame) 

    for i, (alpha, a, d, theta) in enumerate(dh_parameters):

        if joint_type[i] == 'r':
          theta = theta + joint_positions[i]
        elif joint_type[i] == 't':
          d = d + joint_positions[i]
        elif joint_type[i] == 'f':
          _ = 1 # do nothing for the fixed end-effector joint
        else:
          print('[ERR]: Unknown joint type')
        
        transformation_matrix = dh_to_transformation_matrix(alpha, a, d, theta)
        # Update current position based on transformation matrix
        current_position = previous_position @ transformation_matrix
        previous_position = current_position
        
    return current_position # the return is 4X4 matrix
  
  
# initialize end-effector positions
ee_loc_samples = np.ones((num_samples, 4))

for i in range(num_samples):
  ee_loc_samples[i, 0:3] =  arm_FK([random.uniform(joint_limits[i][0], joint_limits[i][1]) for i in range(len(joint_limits))], dh_parameters, base_frame)[0:3, 3]
  
fig = plt.figure()
ax = fig.add_subplot(projection='3d')

# Draw base frame axes
ax.quiver(base_frame[0, 3], base_frame[1, 3], base_frame[2, 3], base_frame[0, 0], base_frame[1, 0], base_frame[2, 0], color='b', length=0.5*plt_axis_limit)  # X-axis in blue, , length=0.1*plt_axis_limit, width=0.005*plt_axis_limit
ax.quiver(base_frame[0, 3], base_frame[1, 3], base_frame[2, 3], base_frame[0, 2], base_frame[1, 2], base_frame[2, 2], color='r', length=0.5*plt_axis_limit)  # Z-axis in red

# draw point cloud of RAVEN workspace
ax.scatter(ee_loc_samples[:,0], ee_loc_samples[:,1], ee_loc_samples[:,2], s=1.0, c = 'tab:gray')

# draw test points
ax.scatter(np.array(test_points)[:,0], np.array(test_points)[:,1], np.array(test_points)[:,2], s=50.0, c = 'r')

def angle_between_vectors(v1, v2):
  # Calculate the dot product of the two vectors
  dot_product = np.dot(v1, v2)
  
  # Calculate the magnitude (length) of each vector
  magnitude_v1 = np.linalg.norm(v1)
  magnitude_v2 = np.linalg.norm(v2)
  
  # Calculate the cosine of the angle between the vectors
  cos_angle = dot_product / (magnitude_v1 * magnitude_v2)
  
  # Ensure the cosine value is within the range [-1, 1] to avoid numerical issues
  cos_angle = np.clip(cos_angle, -1, 1)
  
  # Calculate the angle in radians and then convert to degrees
  angle_rad = np.arccos(cos_angle)
  angle_deg = np.degrees(angle_rad)
  
  return angle_deg

# function to test if test points are in monte carlo simulation points
def point_in_workspace_check(test_point, mc_points):
  # Build a KDTree for the MC points
  tree = KDTree(mc_points)
  # Find the 100 nearest neighbors to the test point
  distances, indices = tree.query([test_point], k=50)
  # Retrieve the nearest points using the indices
  nearest_points = mc_points[indices]
  nearest_vectors = np.squeeze(nearest_points - [test_point])
  
  angles = []
  for i in range(nearest_vectors.shape[0]):
    for j in range(i+1, nearest_vectors.shape[0]):
      vector_angle = angle_between_vectors(nearest_vectors[i], nearest_vectors[j])
      angles.append(vector_angle)
  if np.mean(angles) > 85.0:
    # print(np.mean(angles))
    return True
  else:
    # print(np.mean(angles))
    return False
  
for test_point in test_points:
  if point_in_workspace_check(test_point, ee_loc_samples[:,0:3]) == True:
    print('Point: ', [test_point[0], test_point[1], test_point[2]], ' is INSIDE workspace')
  else:
    print('Point: ', [test_point[0], test_point[1], test_point[2]], ' is OUTSIDE workspace')
  
  
