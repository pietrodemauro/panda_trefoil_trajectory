# -*- coding: utf-8 -*-
"""
Created on Sat Aug 30 10:26:10 2025

@author: pietr
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
import os

def rotm2quat(R):
#to convert rotation Matrices into quaternions
    rot = Rotation.from_matrix(R)
    return rot.as_quat()  # Returns [x, y, z, w]

def rotm2eul(R, sequence='ZYX'):
#to convert rotation Matrices into Euler angles
    rot = Rotation.from_matrix(R)
    return rot.as_euler(sequence, degrees=False)  # Returns in radians

def plot_ee_trajectory(time, position, orientation_rpy):
    
    # Add expected trajectory
    scale_factor = 0.05
    offsetx = 0.4
    offsety = 0.2
    offsetz = 0.5
    
    t = np.linspace(0, 2*np.pi, len(time))
    
    x = (np.sin(t) + 2*np.sin(2*t)) * scale_factor + offsetx
    y = (np.cos(t) - 2*np.cos(2*t)) * scale_factor + offsety
    z = -np.sin(3*t) * scale_factor + offsetz
    
    plt.figure(figsize=(12, 4))
    plt.suptitle('End Effector Position')
    
    plt.subplot(1, 3, 1)
    plt.plot(time, position[:, 0], 'r-', linewidth=2)
    plt.plot(time, x, label='expected X', linewidth=2)
    plt.title('X Position')
    plt.xlabel('Time [s]')
    plt.ylabel('Position [m]')
    plt.grid(True)
    
    plt.subplot(1, 3, 2)
    plt.plot(time, position[:, 1], 'g-', linewidth=2)
    plt.plot(time, y, label='expected Y', linewidth=2)
    plt.title('Y Position')
    plt.xlabel('Time [s]')
    plt.ylabel('Position [m]')
    plt.grid(True)
    
    plt.subplot(1, 3, 3)
    plt.plot(time, position[:, 2], 'k-', linewidth=2)
    plt.plot(time, z, label='expected Z', linewidth=2)
    plt.title('Z Position')
    plt.xlabel('Time [s]')
    plt.ylabel('Position [m]')
    plt.grid(True)
    
    plt.tight_layout()
    
    # Orientation plot (Euler angles)
    plt.figure(figsize=(12, 4))
    plt.suptitle('End Effector Orientation')
    
    plt.subplot(1, 3, 1)
    plt.plot(time, np.rad2deg(orientation_rpy[:, 0]), 'r-', linewidth=2)
    plt.title('Roll')
    plt.xlabel('Time [s]')
    plt.ylabel('Angle [deg]')
    plt.grid(True)
    
    plt.subplot(1, 3, 2)
    plt.plot(time, np.rad2deg(orientation_rpy[:, 1]), 'g-', linewidth=2)
    plt.title('Pitch')
    plt.xlabel('Time [s]')
    plt.ylabel('Angle [deg]')
    plt.grid(True)
    
    plt.subplot(1, 3, 3)
    plt.plot(time, np.rad2deg(orientation_rpy[:, 2]), 'b-', linewidth=2)
    plt.title('Yaw')
    plt.xlabel('Time [s]')
    plt.ylabel('Angle [deg]')
    plt.grid(True)
    
    plt.tight_layout()
    
    # 3D trajectory plot
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(position[:, 0], position[:, 1], position[:, 2], 'b-', linewidth=2)
    ax.scatter(position[0, 0], position[0, 1], position[0, 2], s=100, c='g', marker='o')
    ax.scatter(position[-1, 0], position[-1, 1], position[-1, 2], s=100, c='r', marker='o')
    
    ax.plot(x, y, z, 'r--', linewidth=2)
    
    ax.set_title('3D End Effector Trajectory')
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.legend(['Trajectory', 'Start', 'End', 'Expected Trajectory'])
    ax.grid(True)
    ax.set_box_aspect([1, 1, 1])  # For equal aspect ratio
    
    # Expected coordinates plot
    plt.figure(figsize=(12, 4))
    plt.suptitle('End Effector Expected Coordinates')
    
    plt.subplot(1, 3, 1)
    plt.plot(t, x)
    plt.title('x(t)')
    plt.xlabel('t')
    
    plt.subplot(1, 3, 2)
    plt.plot(t, y)
    plt.title('y(t)')
    plt.xlabel('t')
    
    plt.subplot(1, 3, 3)
    plt.plot(t, z)
    plt.title('z(t)')
    plt.xlabel('t')
    
    plt.tight_layout()
    
    plt.show()
 
csv_filename = "robot_data.csv"
 
# Read the CSV file
data = pd.read_csv(csv_filename, header=None).values
 
# Extract time (first column)
time = data[1:, 0]
 
# The O_T_EE matrix (16 values for 4x4 matrix)
O_T_EE_data = data[1:, 189:205]  # 16 columns for the 4x4 matrix
 
pos_in_time = O_T_EE_data[:, 12:15]  # Columns 13-15
 
# Initialize arrays for position and orientation
num_samples = len(time)
position = np.zeros((num_samples, 3))
orientation_quat = np.zeros((num_samples, 4))
orientation_rpy = np.zeros((num_samples, 3))
 
# Process each sample
for i in range(num_samples):
    # Reshape the 16-element vector into a 4x4 matrix (column-major)
    T = O_T_EE_data[i, :].reshape(4, 4, order='F')
     
    # Extract position (translation) from transformation matrix
    position[i, :] = T[:3, 3]
     
    # Extract rotation matrix
    R = T[:3, :3]
     
    # Convert rotation matrix to quaternion
    quat = rotm2quat(R)
    orientation_quat[i, :] = [quat[3], quat[0], quat[1], quat[2]]  # [w, x, y, z]
     
    # Convert rotation matrix to Euler angles (roll, pitch, yaw)
    orientation_rpy[i, :] = rotm2eul(R, 'ZYX')

#Average of the angles
average_rpy = np.zeros([3,1])
for i in range(len(orientation_rpy[1, :])):
    average_rpy[i] = np.sum(orientation_rpy[:, 1])/len(orientation_rpy[:, 1])
print("Average of the Roll, Pitch and Yaw angles: ",average_rpy)

 
# Plot the results
plot_ee_trajectory(time, position, orientation_rpy)
 