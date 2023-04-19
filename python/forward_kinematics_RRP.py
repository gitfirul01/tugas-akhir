import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Define the link lengths for the robot arm
L1 = 2
L2 = 3
L3 = 2

# Define the desired position and orientation of the end effector
xd = np.array([4, 3, 5]) # desired end effector position
Rd = np.array([
    [-1, 0, 0],
    [0, 1, 0],
    [0, 0, -1]
]) # desired end effector orientation

# Define the inverse kinematics function
def inverse_kinematics(xd, Rd):
    # Extract the desired position and orientation from the desired pose
    xd = xd.reshape((3,1))
    xd_p = xd[:2]
    xd_z = xd[2]
    Rd_p = Rd[:2,:2]
    Rd_z = Rd[2,2]

    # Calculate the joint angles for the first two joints
    c2 = (np.linalg.norm(xd_p)**2-L1**2-L2**2)/(2*L1*L2)
    s2 = np.sqrt(1-c2**2)
    theta2 = np.arctan2(s2, c2)
    k = L1+L2*np.cos(theta2)
    phi = np.arctan2(xd_p[1], xd_p[0])
    theta1 = phi-np.arctan2(xd_z, k)

    # Calculate the joint angle for the third joint
    Rd_z_p = np.sqrt(Rd_p[0,0]**2+Rd_p[1,0]**2)
    if Rd_z_p < 1e-6:
        theta3 = 0
    else:
        theta3 = np.arctan2(Rd_p[1,0], Rd_p[0,0])-theta1-theta2

    # Return the joint angles
    return np.array([theta1, theta2, theta3])

# Calculate the joint angles for the desired end effector pose
theta = inverse_kinematics(xd, Rd)

# Print the joint angles
print('Joint angles: ', theta)

# Plot the robot arm in the desired configuration
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-L1-L2-L3-1, L1+L2+L3+1])
ax.set_ylim([-L1-L2-L3-1, L1+L2+L3+1])
ax.set_zlim([0, L1+L2+L3+1])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.view_init(elev=45, azim=-120)
ax.plot([0, L1*np.cos(theta[0]), L1*np.cos(theta[0])+L2*np.cos(theta[0]+theta[1]), xd[0]], 
        [0, L1*np.sin(theta[0]), L1*np.sin(theta[0])+L2*np.sin(theta[0]+theta[1]), xd[1]], 
        [0, 0, 0, xd[2]], 
        'o-', linewidth=2)
plt.show()
