import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Define the joint angles for the robot arm
theta1 = 30 # in degrees
theta2 = 45 # in degrees
theta3 = 60 # in degrees

# Define the link lengths for the robot arm
l1 = 2
l2 = 3
l3 = 2

# Convert the joint angles to radians
theta1 = np.radians(theta1)
theta2 = np.radians(theta2)
theta3 = np.radians(theta3)

# Calculate the transformation matrix for each link
T01 = np.array([
    [np.cos(theta1), -np.sin(theta1), 0, 0],
    [np.sin(theta1), np.cos(theta1), 0, 0],
    [0, 0, 1, l1],
    [0, 0, 0, 1]
])
T12 = np.array([
    [np.cos(theta2), -np.sin(theta2), 0, l2*np.cos(theta2)],
    [0, 0, -1, 0],
    [np.sin(theta2), np.cos(theta2), 0, l2*np.sin(theta2)],
    [0, 0, 0, 1]
])
T23 = np.array([
    [np.cos(theta3), -np.sin(theta3), 0, l3*np.cos(theta3)],
    [0, 0, 1, 0],
    [-np.sin(theta3), -np.cos(theta3), 0, l3*np.sin(theta3)],
    [0, 0, 0, 1]
])

# Calculate the transformation matrix for the end effector
T03 = T01 @ T12 @ T23

# Extract the position of the end effector
x = T03[0, 3]
y = T03[1, 3]
z = T03[2, 3]

# Print the position of the end effector
print(f"End effector position: ({x}, {y}, {z})")

# Create a 3D plot of the robot arm
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Define the base of the robot arm
ax.plot([0, T01[0, 3]], [0, T01[1, 3]], [0, T01[2, 3]], 'r')

# Define the second link of the robot arm
ax.plot([T01[0, 3], T12[0, 3]], [T01[1, 3], T12[1, 3]], [T01[2, 3], T12[2, 3]], 'g')

# Define the third link of the robot arm
ax.plot([T12[0, 3], x], [T12[1, 3], y], [T12[2, 3], z], 'b')

# Set the limits of the plot
ax.set_xlim(-5, 5)
ax.set_ylim(-5, 5)
ax.set_zlim(0, 10)

# Set labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Robot Arm Simulation')

# Show the plot
plt.show()
