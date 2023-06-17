import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

res = 15

# Define the joint limits
joint_limits = [[0, 90], [-55, 55], [14.37, 188.16]]  # Example joint limits in degrees

# Generate the grid of joint angles
q1 = np.linspace(joint_limits[0][0], joint_limits[0][1], res)
q2 = np.linspace(joint_limits[1][0], joint_limits[1][1], res)
q3 = np.linspace(joint_limits[2][0], joint_limits[2][1], res)
q1, q2, q3 = np.meshgrid(q1, q2, q3)
# Compute the workspace coordinates
x = (353434293*np.cos(np.deg2rad(q1)))/2500000 - (197*q3*((259*np.cos(np.deg2rad(q1))*np.sin(np.deg2rad(q2)))/1000 - np.cos(np.deg2rad(q2))*np.sin(np.deg2rad(q1))))/250 - (37191*q3*np.cos(np.deg2rad(q1)))/62500 - (1813*np.cos(np.deg2rad(q1))*np.cos(np.deg2rad(q2)))/1000 + (101408601*np.cos(np.deg2rad(q1))*np.sin(np.deg2rad(q2)))/2500000 - (391539*np.cos(np.deg2rad(q2))*np.sin(np.deg2rad(q1)))/2500 - 7*np.sin(np.deg2rad(q1))*np.sin(np.deg2rad(q2))
y = (3381*np.cos(np.deg2rad(q2)))/500 - (19943*q3)/125000 - (189113337*np.sin(np.deg2rad(q2)))/1250000 + (95151*q3*np.sin(np.deg2rad(q2)))/125000 + 1408193389/5000000
z = (197*q3*(np.cos(np.deg2rad(q1))*np.cos(np.deg2rad(q2)) + (259*np.sin(np.deg2rad(q1))*np.sin(np.deg2rad(q2)))/1000))/250 - (353434293*np.sin(np.deg2rad(q1)))/2500000 + (37191*q3*np.sin(np.deg2rad(q1)))/62500 - (391539*np.cos(np.deg2rad(q1))*np.cos(np.deg2rad(q2)))/2500 - 7*np.cos(np.deg2rad(q1))*np.sin(np.deg2rad(q2)) + (1813*np.cos(np.deg2rad(q2))*np.sin(np.deg2rad(q1)))/1000 - (101408601*np.sin(np.deg2rad(q1))*np.sin(np.deg2rad(q2)))/2500000 + 148

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')





## Plot all of the outer surface
# for i in range(res):
#     ax.plot_surface(x[:,i,:], y[:,i,:], z[:,i,:], cmap='viridis', edgecolor='none', alpha=1)

ax.plot_surface(x[res-1,:,:], y[res-1,:,:], z[res-1,:,:], cmap='viridis', edgecolor='royalblue', alpha=0)
ax.plot_surface(x[0,:,:], y[0,:,:], z[0,:,:], cmap='viridis', edgecolor='royalblue', alpha=0)

ax.plot_surface(x[:,res-1,:], y[:,res-1,:], z[:,res-1,:], cmap='viridis', edgecolor='royalblue', alpha=0)
ax.plot_surface(x[:,0,:], y[:,0,:], z[:,0,:], cmap='viridis', edgecolor='royalblue', alpha=0)

ax.plot_surface(x[:,:,res-1], y[:,:,res-1], z[:,:,res-1], cmap='viridis', edgecolor='royalblue', alpha=0)
ax.plot_surface(x[:,:,0], y[:,:,0], z[:,:,0], cmap='viridis', edgecolor='royalblue', alpha=0)


## Plot the outer surface
# surf1 = ax.plot_surface(x[:,:,res-1], y[:,:,res-1], z[:,:,res-1], cmap='viridis', edgecolor='none', alpha=0.8)
## Get the minimum and maximum values from the first surface data
# vmin, vmax = surf1.get_array().min(), surf1.get_array().max()
## Plot the second surface with the same colormap and continuous scale
# surf2 = ax.plot_surface(x[0,:,:], y[0,:,:], z[0,:,:], cmap='viridis', edgecolor='none', alpha=0.8, norm=plt.Normalize(vmin=vmin, vmax=vmax))

## Plot the outer surface in the wireframe
# ax.plot_surface(x[:,:,res-1], y[:,:,res-1], z[:,:,res-1], edgecolor='royalblue', alpha=0)
# ax.plot_surface(x[0,:,:], y[0,:,:], z[0,:,:], edgecolor='royalblue', alpha=0)





# Set plot labels and limits
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
# ax.set_xlim([-3, 3])
# ax.set_ylim([-3, 3])
# ax.set_zlim([-3, 3])

# Display the plot
plt.show()
