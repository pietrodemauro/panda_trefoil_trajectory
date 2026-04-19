# -*- coding: utf-8 -*-
"""
Created on Sat Aug 30 12:19:06 2025

@author: pietr
"""

import numpy as np
import matplotlib.pyplot as plt
import scipy.io as sio

# Define parameters
scale_factor = 1  # to fit the franka workspace
offsetx = 0
offsety = 0
offsetz = 0

# Create time vector
t = np.arange(0, 2 * np.pi, 0.01)

# Calculate trajectory coordinates
x = (np.sin(t) + 2 * np.sin(2 * t)) * scale_factor + offsetx
y = (np.cos(t) - 2 * np.cos(2 * t)) * scale_factor + offsety
z = -np.sin(3 * t) * scale_factor + offsetz

# --------------------------------------------------
# Plot 3D trajectory
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')
ax.plot(x, y, z)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Trefoil Knot')
ax.set_box_aspect([1, 1, 1])
ax.grid(True)
plt.show()

# --------------------------------------------------
# Calculate derivatives for velocity and acceleration
dt = t[1] - t[0]
dx = np.gradient(x, dt)
dy = np.gradient(y, dt)
dz = np.gradient(z, dt)
ddx = np.gradient(dx, dt)
ddy = np.gradient(dy, dt)
ddz = np.gradient(dz, dt)

# --------------------------------------------------
# Plot position vs time
plt.figure(figsize=(10, 6))
plt.plot(t, x, label='X')
plt.plot(t, y, label='Y')
plt.plot(t, z, label='Z')
plt.title('Position vs Time')
plt.xlabel('Time [s]')
plt.ylabel('Position [m]')
plt.legend()
plt.grid(True)
plt.show()

# --------------------------------------------------
# Plot velocity vs time
plt.figure(figsize=(10, 6))
plt.plot(t, dx, label='dX')
plt.plot(t, dy, label='dY')
plt.plot(t, dz, label='dZ')
plt.title('Velocity vs Time')
plt.xlabel('Time [s]')
plt.ylabel('Velocity [m/s]')
plt.legend()
plt.grid(True)
plt.show()

# --------------------------------------------------
# Check Franka constraints
max_velocity = np.max(np.sqrt(dx**2 + dy**2 + dz**2))
max_acceleration = np.max(np.sqrt(ddx**2 + ddy**2 + ddz**2))

print(f'Maximum velocity: {max_velocity:.3f} m/s')
print(f'Maximum acceleration: {max_acceleration:.3f} m/s²')

# --------------------------------------------------
# Save trajectory
trajectory = np.vstack((t, x, y, z)).T

# Save to .mat file
sio.savemat('trefoil_trajectory.mat', {'trajectory': trajectory})

# Save to .csv file 
np.savetxt('trefoil_trajectory.csv', trajectory, delimiter=',')