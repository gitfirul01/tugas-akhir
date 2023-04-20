import serial, struct
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


## confgure serial communication
try:
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    ser.reset_input_buffer()
except:
    print("Serial not connected!")
    exit()


## define the link lengths for the robot arm
l1 = 45
l2 = 150
l3 = 150
theta1 = 0.0
theta2 = 0.0
theta3 = 0.0

## turn on interactive plot
plt.ion()

## create a 3D figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')


while True:
    ## read joint angle from serial
    if ser.read(1) == b'@':
        data = ser.read_until(b'$')
        data = data[1:len(data)-1]
        try:
            value = struct.unpack('fff', data)
            theta1 = value[0]
            theta2 = value[1]
            theta3 = value[2]
        except:
            pass

    ## convert the joint angles to radians
    Theta1 = np.radians(theta1)
    Theta2 = np.radians(theta2)
    Theta3 = np.radians(theta3)

    ## Calculate the transformation matrix for each link
    # T01 = np.array([
    #     [np.cos(Theta1), 0, np.sin(Theta1), 0],
    #     [np.sin(Theta1), 0, -np.cos(Theta1), 0],
    #     [0, 1, 0, l1],
    #     [0, 0, 0, 1]
    # ])
    # T12 = np.array([
    #     [np.cos(Theta2), -np.sin(Theta2), 0, l2*np.cos(Theta2)],
    #     [np.sin(Theta2), np.cos(Theta2), 0, l2*np.sin(Theta2)],
    #     [0, 0, 1, 0],
    #     [0, 0, 0, 1]
    # ])
    # T23 = np.array([
    #     [np.cos(Theta3), -np.sin(Theta3), 0, l3*np.cos(Theta3)],
    #     [np.sin(Theta3), np.cos(Theta3), 0, l3*np.sin(Theta3)],
    #     [0, 0, 1, 0],
    #     [0, 0, 0, 1]
    # ])

    T01 = np.array([
        [np.cos(Theta1), -np.sin(Theta1), 0, 0],
        [np.sin(Theta1), np.cos(Theta1), 0, 0],
        [0, 0, 1, l1],
        [0, 0, 0, 1]
    ])
    T12 = np.array([
        [np.cos(Theta2), -np.sin(Theta2), 0, l2*np.cos(Theta2)],
        [0, 0, -1, 0],
        [np.sin(Theta2), np.cos(Theta2), 0, l2*np.sin(Theta2)],
        [0, 0, 0, 1]
    ])
    T23 = np.array([
        [np.cos(Theta3), -np.sin(Theta3), 0, l3*np.cos(Theta3)],
        [0, 0, 1, 0],
        [-np.sin(Theta3), -np.cos(Theta3), 0, l3*np.sin(Theta3)],
        [0, 0, 0, 1]
    ])

    ## Calculate the transformation matrix for the end effector
    T03 = T01 @ T12 @ T23

    ## Extract the position of the end effector
    x = T03[0,3]
    y = T03[1,3]
    z = T03[2,3]

    ## print the position of the end effector
    print("Angle: ({:.2f}, {:.2f}, {:.2f})   EE-pos: ({:.2f}, {:.2f}, {:.2f})".format(theta1,theta2,theta3,x,y,z))

    ## clear axes plot
    ax.cla()
    
    ## redraw plot
    ax.plot([0, T01[0,3]],          [0, T01[1,3]],          [0, T01[2,3]],          'r')
    ax.plot([T01[0,3], T12[0,3]],   [T01[1,3], T12[1,3]],   [T01[2,3], T12[2,3]],   'g')
    ax.plot([T12[0,3], x],          [T12[1,3], y],          [T12[2,3], z],          'b')
    ax.scatter(x, y, z)

    ## set figure configuration
    # ax.set_title('3D Simulation')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim(0, 300)
    ax.set_ylim(-150, 150)
    ax.set_zlim(0, 300)

    ## show the plot
    fig.canvas.draw()
    fig.canvas.flush_events()
    plt.pause(0.001)
