import serial, struct
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

## confgure serial communication
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
ser.reset_input_buffer()


## turn on interactive plot
plt.ion()

## create a 3D figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

## set figure configuration
ax.set_title('3D Simulation')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_xlim(-200, 200)
ax.set_ylim(-200, 200)
ax.set_zlim(-200, 200)


## Define the link lengths for the robot arm
l1 = 45
l2 = 150
l3 = 150

theta1 = 0.0
theta2 = 0.0
theta3 = 0.0

plot = ax.scatter(0, 0, 0)

while True:
    ## Define the joint angles for the robot arm
    if ser.read(1) == b'@':
        data = ser.read_until(b'$')
        data = data[1:len(data)-1]
        try:
            value = struct.unpack('fff', data)
            theta1 = value[0]
            theta2 = value[1]
            theta3 = value[2]
        except:
            None

    ## Convert the joint angles to radians
    # theta1 = np.radians(theta1)
    # theta2 = np.radians(theta2)
    # theta3 = np.radians(theta3)

    ## Calculate the transformation matrix for each link
    T01 = np.array([
        [np.cos(theta1), 0, np.sin(theta1), 0],
        [np.sin(theta1), 0, -np.cos(theta1), 0],
        [0, 1, 0, l1],
        [0, 0, 0, 1]
    ])
    T12 = np.array([
        [np.cos(theta2), -np.sin(theta2), 0, l2*np.cos(theta2)],
        [np.sin(theta2), np.cos(theta2), 0, l2*np.sin(theta2)],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    T23 = np.array([
        [np.cos(theta3), -np.sin(theta3), 0, l3*np.cos(theta3)],
        [np.sin(theta3), np.cos(theta3), 0, l3*np.sin(theta3)],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    ## Calculate the transformation matrix for the end effector
    T03 = T01 @ T12 @ T23

    ## Extract the position of the end effector
    x = T03[0, 3]
    y = T03[1, 3]
    z = T03[2, 3]

    # Print the position of the end effector
    print(f"End effector position: ({x}, {y}, {z})")

    ## Define the base of the robot arm
    # ax.plot([0, T01[0, 3]], [0, T01[1, 3]], [0, T01[2, 3]], 'r')

    ## Define the second link of the robot arm
    # ax.plot([T01[0, 3], T12[0, 3]], [T01[1, 3], T12[1, 3]], [T01[2, 3], T12[2, 3]], 'g')

    ## Define the third link of the robot arm
    # ax.plot([T12[0, 3], x], [T12[1, 3], y], [T12[2, 3], z], 'b')

    ## Set the limits of the plot
    # ax.set_xlim(-5, 5)
    # ax.set_ylim(-5, 5)
    # ax.set_zlim(0, 10)

    ## Set labels and title
    # ax.set_xlabel('X')
    # ax.set_ylabel('Y')
    # ax.set_zlabel('Z')
    # ax.set_title('3D Robot Arm Simulation')

    # draw the plot
    plot.remove()
    plot = ax.scatter(x, y, z)
    # show the plot
    fig.canvas.draw()
    fig.canvas.flush_events()
    plt.pause(0.001)
    
