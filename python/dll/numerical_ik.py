import numpy as np
import matplotlib.pyplot as plt

# Define the arm segments and their lengths
L1 = 2.0
L2 = 1.5

# Define the target point to reach
x_target = 2.5
y_target = 1.5

# Define the initial position of the arm
theta1 = np.pi / 4
theta2 = np.pi / 4

# Define the maximum number of iterations
max_iterations = 1000

# Define the tolerance for the error
tolerance = 0.0001

# Define the learning rate
learning_rate = 0.1

# Define the function to calculate the forward kinematics
def forward_kinematics(theta1, theta2):
    x = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    y = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)
    return x, y

# Define the function to calculate the inverse kinematics
def inverse_kinematics(x_target, y_target):
    theta1 = np.arctan2(y_target, x_target) - np.arctan2(L2 * np.sin(np.pi / 2 - theta2), L1 + L2 * np.cos(np.pi / 2 - theta2))
    theta2 = np.arccos((x_target * np.cos(theta1) + y_target * np.sin(theta1) - L1) / L2)
    return theta1, theta2

# Create a figure and axis for plotting
fig, ax = plt.subplots()

# Loop through the iterations
for i in range(max_iterations):
    # Calculate the forward kinematics of the arm
    x, y = forward_kinematics(theta1, theta2)
    
    # Plot the arm
    ax.plot([0, L1 * np.cos(theta1), x], [0, L1 * np.sin(theta1), y], '-o')
    ax.plot(x_target, y_target, 'xr')
    ax.set_xlim(-1, 4)
    ax.set_ylim(-1, 4)
    plt.pause(0.01)
    plt.cla()
    
    # Calculate the error between the target point and the end-effector position
    error = np.sqrt((x_target - x)**2 + (y_target - y)**2)
    
    # If the error is below the tolerance, break out of the loop
    if error < tolerance:
        break
    
    # Calculate the gradients for each joint angle
    theta1_grad = ((x - x_target) * np.sin(theta1) - (y - y_target) * np.cos(theta1)) / (L1 * L2 * np.sin(theta2))
    theta2_grad = ((x - x_target) * np.sin(theta1 + theta2) - (y - y_target) * np.cos(theta1 + theta2)) / (L1 * L2 * np.sin(theta2))
    
    # Update the joint angles using gradient descent
    theta1 -= learning_rate * theta1_grad
    theta2 -= learning_rate * theta2_grad

# Show the final arm position and target point
ax.plot([0, L1 * np.cos(theta1), x], [0, L1 * np.sin(theta1), y], '-o')
ax.plot(x_target, y_target, 'xr')
ax.set_xlim(-1, 4)
ax.set_ylim(-1, 4)
plt.show()