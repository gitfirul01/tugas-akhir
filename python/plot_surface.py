import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Define the joint limits
joint_limits = [[-90, 90], [0, 120], [135, 250]]  # Example joint limits in degrees

# Generate the grid of joint angles
q1 = np.linspace(joint_limits[0][0], joint_limits[0][1], 50)
q2 = np.linspace(joint_limits[1][0], joint_limits[1][1], 50)
q3 = np.linspace(joint_limits[2][0], joint_limits[2][1], 50)
q1, q2, q3 = np.meshgrid(q1, q2, q3)

# Compute the workspace coordinates
x = 150*np.cos(np.deg2rad(q1))*np.cos(np.deg2rad(q2)) - 150*np.cos(np.deg2rad(q1))*np.cos(np.deg2rad(q2))*np.sin(np.deg2rad(q3)) - 150*np.cos(np.deg2rad(q1))*np.cos(np.deg2rad(q3))*np.sin(np.deg2rad(q2))
y = 150*np.cos(np.deg2rad(q2))*np.sin(np.deg2rad(q1)) - 150*np.cos(np.deg2rad(q2))*np.sin(np.deg2rad(q1))*np.sin(np.deg2rad(q3)) - 150*np.cos(np.deg2rad(q3))*np.sin(np.deg2rad(q1))*np.sin(np.deg2rad(q2))
z = 150*np.sin(np.deg2rad(q2)) + 150*np.cos(np.deg2rad(q2))*np.cos(np.deg2rad(q3)) - 150*np.sin(np.deg2rad(q2))*np.sin(np.deg2rad(q3)) + 45

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Reshape the arrays to 2D
x = x.flatten()
y = y.flatten()
z = z.flatten()

# Plot the surface
ax.plot_trisurf(x, y, z, cmap='viridis', edgecolor='none', alpha=0.8)

# Set plot labels and limits
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
# ax.set_xlim([-3, 3])
# ax.set_ylim([-3, 3])
# ax.set_zlim([-3, 3])

# Display the plot
plt.show()
