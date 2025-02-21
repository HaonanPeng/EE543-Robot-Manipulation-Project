# -*- coding: utf-8 -*-
"""
Created on Tue Apr  2 13:36:15 2024

@author: 75678
"""
import sys
import numpy as np

# [Important]: You should control the motor rotation angles to 0 and then use this code to find DH parameters, so that offsets of joint positions will also be found and you can then have joint positions the same as motor positions

# each joint of the robot arm should be given as two points in the sequence of positive joint translation or rotation (right hand rule)
# The points and the axis direction of frame_0 and frame_e can be given in any coordinate frame, but need to be consistent (all given in the same frame)
joint_points = np.array([[[0, 0, 0],[0, 0, 1]],
                          [[0, 0, 62.8],[1, 0, 62.8]],
                          [[-5.1, 0, 167.8],[-5.1, -1, 167.8]],
                          [[0, 0, 194.8],[1, 0, 194.8]],
                          [[-10, 0, 0],[-10, 0, 1]]
                        ])



# all frames should be defined as three (3,) vectors, the 1st vector is the origin location, the 2nd vector is the unit direction vector of x axis, the 3rd vector is the direction of z axis, no need to set y axis since it will be determined by right hand rule
# the origin and vectors can be given in the world frame, or actually can be given in any frames
"""
[IMPT]: it should be noticed that although z axis (z0) of the base frame can be arbitrarily chosen,
x0 and origin will only surely be assgined by the given value when z0 coinside with joint 1 axis,
otherwise the origin and/or x0 will be overwritten by the the relation ship between axis 0 (z0) and axis 1 (z1)
If z0 and joint 1 axis are parallel, then x0 will be replace by the direction of the CN (common normal)
If z0 and joint 1 axis are intersect of skewed, then both x0 and origin will be determined by the common normal direction and start point
"""
frame_0 = {'org': np.array([0, 0, 0]),
           'x_ax': np.array([1, 0, 0]),
           'z_ax': np.array([0, 0, 1])}

# define the end-effector frame, this is NOT the frame of the last joint. Instead, this frame can be arbitarily chosen to better represent the pose of the end-effector
frame_e = {'org': np.array([-10, 0, 204.8]),
           'x_ax': np.array([0, -1, 0]),
           'z_ax': np.array([0, 0, 1])}

joint_points = np.array([[[10.5, 6, 0],[10.5, 6, -49.3]],
                         [[30.5769, -22.2680,-84.5037],[30.7655, 32.7316, -84.5037]],
                         [[33.7852 , 5.38121, -191.926],[32.7014, 44.2760, -153.054]],
                         [[34.6027, 60.5330, -208.178],[37.7603, 143.466, -291.072]],
                        ])

frame_0 = {'org': np.array([10.5, 6, 0]),
           'x_ax': np.array([1, 0, 0]),
           'z_ax': np.array([0, 0, -1])}

frame_e = {'org': np.array([37.7603, 143.466, -291.072]),
           'x_ax': (np.array([37.5251, 151.921, -282.622]) - np.array([37.7603, 143.466, -291.072])) / np.linalg.norm(np.array([37.5251, 151.921, -282.622]) - np.array([37.7603, 143.466, -291.072])),
           'z_ax': (np.array([37.7603, 143.466, -291.072]) - np.array([34.6027, 60.5330, -208.178])) / np.linalg.norm(np.array([37.7603, 143.466, -291.072]) - np.array([34.6027, 60.5330, -208.178]))}



# line_relation_tolerance = 1e-3
line_relation_tolerance = 0.5


deg2rad = np.pi / 180.0
rad2deg = 180.0 / np.pi


def find_closest_points(P1, d1, Q1, d2):
    # This function is inspired by ChatGPT: https://chat.openai.com/share/c7cb3ed5-6fb0-4a47-a828-c2740ab3297f
    """
    Calculate the closest points on each of two skew lines.

    :param P1: A point on the first line.
    :param d1: The direction vector of the first line.
    :param Q1: A point on the second line.
    :param d2: The direction vector of the second line.
    :return: Closest points on the first and second lines.
    """
    # Compute the vector between points on the two lines
    PQ = Q1 - P1
    # Compute denominators
    denom = np.linalg.norm(np.cross(d1, d2))**2
    # Compute numerators for the parameters t and s
    t = np.linalg.det([PQ, d2, np.cross(d1, d2)]) / denom
    s = np.linalg.det([PQ, d1, np.cross(d1, d2)]) / denom
    
    # Calculate the closest points
    closest_point_on_P = P1 + t * d1
    closest_point_on_Q = Q1 + s * d2
    
    return closest_point_on_P, closest_point_on_Q

"""
# Find the relationship between 2 lines in the 3D space, each line is defined by 2 points.
# If these 2 lines are coincide, parallel, intersect, or skew. 
# If parallel, give a vector that is perpendicular to both lines, pointing from line 1 to line 2 with the length equal to the distance between line 1 and line 2. 
# If intersect, give the point of intersection and a unit vector that is perpendicular to line 1 and line 2. 
# If none of the above, find the common normal of the 2 lines and give the start pint, direction and length of the common normal 
# The decision of coincide, parallel and intersection should be make with a predefined torlerace.
"""
def line_relationship(P1, P2, Q1, Q2, tolerance=1e-9):
    # This function is inspired by ChatGPT: https://chat.openai.com/share/c7cb3ed5-6fb0-4a47-a828-c2740ab3297f
    d1 = P2 - P1  # Direction vector of line 1
    d2 = Q2 - Q1  # Direction vector of line 2
    cross_d1_d2 = np.cross(d1, d2)
    cross_norm = np.linalg.norm(cross_d1_d2)

    # Check for coincidence or parallelism
    if cross_norm <= tolerance:  # Parallel or coincident
        if np.linalg.norm(np.cross(d1, Q1 - P1)) <= tolerance:  # Coincident
            return "Coincide", None
        else:  # Parallel
            # Calculate a vector from P1 to Q1 and project it onto the plane normal to d1 to find the distance vector
            v = Q1 - P1
            distance_vector = v - np.dot(v, d1) / np.dot(d1, d1) * d1
            distance_vector_norm = np.linalg.norm(distance_vector)
            if distance_vector_norm > tolerance:
                distance_vector /= distance_vector_norm  # Normalize the distance vector
            return "Parallel", {'CN_dir' : distance_vector, 'CN_len' : distance_vector_norm}

    # Attempt to find an intersection
    A = np.array([d1, -d2, cross_d1_d2]).T
    b = Q1 - P1
    try:
        t, s, _ = np.linalg.solve(A, b)
        intersection_point = P1 + d1 * t
        if np.allclose(intersection_point, Q1 + d2 * s, atol=tolerance):
            # For the intersection case, calculate the perpendicular unit vector
            perp_vector = np.cross(d1, d2)
            perp_vector_unit = perp_vector / np.linalg.norm(perp_vector)
            return "Intersect", {'CN_pt1' : intersection_point, 'CN_dir' : perp_vector_unit}
    except np.linalg.LinAlgError:
        pass  # No intersection found, proceed to check skewness

    # If not parallel, coincident, or intersecting, lines are skew
    closest_point_on_P, closest_point_on_Q = find_closest_points(P1, d1, Q1, d2)
    perp_vector_unit = perp_vector_unit = (closest_point_on_Q - closest_point_on_P) / np.linalg.norm(closest_point_on_Q - closest_point_on_P)
    distance_vector_norm = np.linalg.norm(closest_point_on_Q - closest_point_on_P)
    
    return "Skew", {'CN_pt1' : closest_point_on_P, 'CN_dir' : perp_vector_unit, 'CN_len' : distance_vector_norm}
  

def rotation_angle(v1, v2, axis):
    # inspired by: https://chat.openai.com/share/38ba452d-32c0-40f9-a761-0c11a8586a5c
    # Normalize the input vectors and axis
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)
    axis = axis / np.linalg.norm(axis)
    
    # Calculate the cosine of the angle using the dot product
    cos_theta = np.dot(v1, v2)
    
    # Ensure the cosine values are within the valid range for arccos
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    
    # Calculate the angle in radians
    angle = np.arccos(cos_theta)
    
    # Use the cross product to determine the direction of rotation
    cross_prod = np.cross(v1, v2)
    
    # If the dot product of the cross product and the axis is negative,
    # the rotation is in the opposite direction, so we negate the angle.
    if np.dot(cross_prod, axis) < 0:
        angle = -angle
    
    # Convert angle to degrees for easier interpretation
    angle_degrees = np.degrees(angle)
    
    return angle_degrees
  
  
"""
# find common normals and point As and Bs
# common normal between each 2 joints, from joint 0 (base frame) to the end effector frame, 
# so that for robot arm with n joints, there will be n+2 frames and n+1 common normals

# Points A_N and B_N, AN: the point where Axis N intersects the CN to Axis N + 1, BN: the point where Axis N intersects the CN to Axis N – 1
# Although we totally have n+2 frames including the base frame and end-effector frame, these 2 frames are given by the user 
and we do not need to assign these 2 frames. Thus, it is meaningless to find point A and B for axis 0 and n+1, they are not real joints and the frames are given by user

In the following sections: assume we have n joints
  common_normals[0] means the common normal from axis 0 to axis 1 (joint 1), so on and so forth
  A_points[0] means point A of axis 0, so on and so forth
  B_points[0] means point B of axis 0, so on and so forth, note that B_points[0] is meaningless since frame 0 is the first frame
"""

num_joints = joint_points.shape[0]  # this is the number of actual joints that can rotate or move, so the base frame and end-effector frame are excluded
num_frames = num_joints + 2

# initialize frames:
frames = [{'org': np.array([0, 0, 0]),'x_ax': np.array([1, 0, 0]),'z_ax': np.array([0, 0, 1])} for i in range(num_frames)] 
frames[0] = frame_0 # assign the forst frame
frames[-1] = frame_e  # assign the end-effector frmae

# initialize  common normals 
# common normal is defined as [[x, y, z], [vx, vy, vz], lenth], 
# [x,y,z] is the start point of the common normal (on former joint axis), [vx, vy, vz] is the unit vector pointing the direction of the common normal, 
# length is the length of the common normal, it can be 0
# the end point of the common normal can be computed as [x, y, z] + length*[vx, vy, vz]
common_normals = [{'p_start': np.array([0, 0, 0]), 'dir': np.array([0, 0, 0]), 'length': 0} for i in range(num_joints+1)]

# initialize A, B points
A_points = np.zeros([num_frames, 3])
B_points = np.zeros([num_frames, 3])
# initialize DH parameters, for each joint, the order is [alpha, a, d, theta]
DH_param = np.zeros([num_joints+1, 4])
for i in range(num_joints+1):
  if i == 0: # the common normal from axis 0 (based frame z) to axis 1 (joint 1)
    relation, relation_info = line_relationship(frames[i]['org'], frames[i]['org']+frames[i]['z_ax'],     # frame 0's origin and z axis point
                                    joint_points[i][0], joint_points[i][1], # 2 points on joint 1
                                    tolerance=line_relation_tolerance)
  elif i < num_joints:  # for all actual joints, so excluding 
    relation, relation_info = line_relationship(joint_points[i-1][0], joint_points[i-1][1],     
                                    joint_points[i][0], joint_points[i][1], 
                                    tolerance=line_relation_tolerance)
  elif i == num_joints:  # for the last common normal, from last actual joint to the end-effector frame
    relation, relation_info = line_relationship(joint_points[i-1][0], joint_points[i-1][1],     
                                    frames[-1]['org'], frames[-1]['org']+frames[-1]['z_ax'], 
                                    tolerance=line_relation_tolerance)
  if relation == 'Coincide':
    # if the 2 axes coincide, the relation_info contains nothing, and we choose common normal start point to be the origin of frame 0, direction the same as x0, and length to be 0
    if i != 0:
      common_normals[i]['p_start'] = frames[i-1]['org']
      common_normals[i]['dir'] = frames[i-1]['x_ax']
      common_normals[i]['length'] = 0
    else:
      common_normals[i]['p_start'] = frames[0]['org']
      common_normals[i]['dir'] = frames[0]['x_ax']
      common_normals[i]['length'] = 0
    
    
  elif relation == 'Parallel':
    # if the 2 axes are parallel, the common normal is chosen to start at the origin of the previous frame
    if i != 0:
      common_normals[i]['p_start'] = frames[i-1]['org']
      common_normals[i]['dir'] = relation_info['CN_dir'] # The unit vector of direction 
      common_normals[i]['length'] = relation_info['CN_len'] # The length of the CN
    else:
      common_normals[i]['p_start'] = frames[0]['org']
      common_normals[i]['dir'] = relation_info['CN_dir'] # The unit vector of direction 
      common_normals[i]['length'] = relation_info['CN_len'] # The length of the CN
   
  elif relation == 'Intersect':
    # if the 2 axes are intersect, the common normal is chosen to start at the intersection point and its length is 0
    common_normals[i]['p_start'] = relation_info['CN_pt1']
    common_normals[i]['dir'] = relation_info['CN_dir'] # The unit vector of direction 
    common_normals[i]['length'] = 0 # The length should be 0 for intersect case
    
  elif relation == 'Skew':
    # if the 2 axes are skewed
    common_normals[i]['p_start'] = relation_info['CN_pt1']
    common_normals[i]['dir'] = relation_info['CN_dir'] # The unit vector of direction 
    common_normals[i]['length'] = relation_info['CN_len'] # The length should be 0 for intersect case
    
  else:
    print('Unknown line relation between Joint: ', i-1, 'and Joint', i)
    sys.exit()
    
  # With the line relation determined, we can now update frame 0 to meet DH parameter based frame assignment
  
  frames[i]['org'] = common_normals[i]['p_start']
  frames[i]['x_ax'] = common_normals[i]['dir']
  # print(common_normals[i]['p_start'])
  if i == 0: # the base frame is given, and B point is meaningless
    frames[i]['z_ax'] = frames[i]['z_ax']  # z axis should not change for frame 0
    A_points[i] =  common_normals[i]['p_start'] # A point should be the start point of the common normal to the next joint axis
    B_points[i] = [0, 0, 0] # For frame 0, this is meaningless
  else:
    frames[i]['z_ax'] = (joint_points[i-1][1] - joint_points[i-1][0]) / np.linalg.norm(joint_points[i-1][1] - joint_points[i-1][0])
    A_points[i] =  common_normals[i]['p_start'] # A point should be the start point of the common normal to the next joint axis
    B_points[i] = common_normals[i-1]['p_start'] + common_normals[i-1]['dir'] * common_normals[i-1]['length']
  
  print('------------------------------------------------------')
  print('Joint {} to Joint {}'.format(i, i+1))
  print('Axis relation: ' + relation)
  print('Common normal:')
  print(common_normals[i])
  print('Frame {}:'.format(i))
  print(frames[i])
  print('-------------------------------------------')

# assign A B points for the fixed axis of the end-effector frame, it will be used to determine d in the last DH row
A_points[i+1] = frames[i+1]['org']
B_points[i+1] = common_normals[i]['p_start'] + common_normals[i]['dir'] * common_normals[i]['length'] 

# now find DH parameters
for i in range(num_joints+1):
  # alpha
  DH_param[i, 0] = rotation_angle(frames[i]['z_ax'], frames[i+1]['z_ax'], frames[i]['x_ax']) 
  # a, the dot product to with the axis vector is to give sign of positive or negative
  DH_param[i, 1] = (common_normals[i]['length']*common_normals[i]['dir']).dot(frames[i]['x_ax'])
  # d, the dot product to with the axis vector is to give sign of positive or negative
  DH_param[i, 2] = (A_points[i+1] - B_points[i+1]).dot(frames[i+1]['z_ax'])
  # theta
  DH_param[i, 3] = rotation_angle(frames[i]['x_ax'], frames[i+1]['x_ax'], frames[i+1]['z_ax']) 

print('|||||||||||||||||||||||||||||||||||||')
print('DH Parameters (alpha, a, d, theta):')
np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})
print(DH_param)




