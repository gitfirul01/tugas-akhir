import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

res = 50

# Define the joint limits
joint_limits = [[-90, 90], [0, 120], [135, 250]]  # Example joint limits in degrees

# Generate the grid of joint angles
q1 = np.linspace(joint_limits[0][0], joint_limits[0][1], res)
q2 = np.linspace(joint_limits[1][0], joint_limits[1][1], res)
q3 = np.linspace(joint_limits[2][0], joint_limits[2][1], res)

q1, q2, q3 = np.meshgrid(q1, q2, q3)

# Compute the workspace coordinates
x = 150*np.cos(np.deg2rad(q1))*np.cos(np.deg2rad(q2)) - 150*np.cos(np.deg2rad(q1))*np.cos(np.deg2rad(q2))*np.sin(np.deg2rad(q3)) - 150*np.cos(np.deg2rad(q1))*np.cos(np.deg2rad(q3))*np.sin(np.deg2rad(q2))
y = 150*np.cos(np.deg2rad(q2))*np.sin(np.deg2rad(q1)) - 150*np.cos(np.deg2rad(q2))*np.sin(np.deg2rad(q1))*np.sin(np.deg2rad(q3)) - 150*np.cos(np.deg2rad(q3))*np.sin(np.deg2rad(q1))*np.sin(np.deg2rad(q2))
z = 150*np.sin(np.deg2rad(q2)) + 150*np.cos(np.deg2rad(q2))*np.cos(np.deg2rad(q3)) - 150*np.sin(np.deg2rad(q2))*np.sin(np.deg2rad(q3)) + 45

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

## Plot the outer surface
# for i in range(res):

# ax.plot_surface(x[:,24,:], y[:,24,:], z[:,24,:], cmap='viridis', edgecolor='none', alpha=1)

surf1 = ax.plot_surface(x[:,:,res-1], y[:,:,res-1], z[:,:,res-1], cmap='viridis', edgecolor='none', alpha=0.8)
## Get the minimum and maximum values from the first surface data
vmin, vmax = surf1.get_array().min(), surf1.get_array().max()
## Plot the second surface with the same colormap and continuous scale
surf2 = ax.plot_surface(x[0,:,:], y[0,:,:], z[0,:,:], cmap='viridis', edgecolor='none', alpha=0.8, norm=plt.Normalize(vmin=vmin, vmax=vmax))


# Set plot labels and limits
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
# ax.set_xlim([-3, 3])
# ax.set_ylim([-3, 3])
# ax.set_zlim([-3, 3])

# Display the plot
plt.show()
