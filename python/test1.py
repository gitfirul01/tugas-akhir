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

## joint angle (x_i to xi over z_i)
theta1 = 0.0
theta2 = 0.0
theta3 = 0.0
## joint offset (O_i to xi over z_i)
d1 = 45
d2 = 0
d3 = 0
## link lengths (Oi to z_i over xi)
a1 = 0
a2 = 150
a3 = 150
## link twist (z_i to zi over xi)
alpha1 = 90
alpha2 = 0
alpha3 = 0

    

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

    ## convert degree to radian
    Theta1 = np.deg2rad(theta1)
    Theta2 = np.deg2rad(theta2)
    Theta3 = np.deg2rad(theta3)
    Alpha1 = np.deg2rad(alpha1)
    Alpha2 = np.deg2rad(alpha2)
    Alpha3 = np.deg2rad(alpha3)

    # declare the Denavit-Hartenberg table with 4 column:
    # theta, alpha, a, and d
    dh_table = np.array([[Theta1, Alpha1, a1, d1],
                         [Theta2, Alpha2, a2, d2],
                         [Theta3, Alpha3, a3, d3]]) 
    
    # Homogeneous transformation matrix from frame 0 to frame 1
    i = 0
    T01 = np.array([[np.cos(dh_table[i,0]),    -np.sin(dh_table[i,0])*np.cos(dh_table[i,1]),    np.sin(dh_table[i,0])*np.sin(dh_table[i,1]),    dh_table[i,2]*np.cos(dh_table[i,0])],
                    [np.sin(dh_table[i,0]),     np.cos(dh_table[i,0])*np.cos(dh_table[i,1]),   -np.cos(dh_table[i,0])*np.sin(dh_table[i,1]),    dh_table[i,2]*np.sin(dh_table[i,0])],
                    [0,                         np.sin(dh_table[i,1]),                          np.cos(dh_table[i,1]),                          dh_table[i,3]],
                    [0,                         0,                                              0,                                              1]])  
    # Homogeneous transformation matrix from frame 1 to frame 2
    i = 1
    T12 = np.array([[np.cos(dh_table[i,0]),    -np.sin(dh_table[i,0])*np.cos(dh_table[i,1]),    np.sin(dh_table[i,0])*np.sin(dh_table[i,1]),    dh_table[i,2]*np.cos(dh_table[i,0])],
                    [np.sin(dh_table[i,0]),     np.cos(dh_table[i,0])*np.cos(dh_table[i,1]),   -np.cos(dh_table[i,0])*np.sin(dh_table[i,1]),    dh_table[i,2]*np.sin(dh_table[i,0])],
                    [0,                         np.sin(dh_table[i,1]),                          np.cos(dh_table[i,1]),                          dh_table[i,3]],
                    [0,                         0,                                              0,                                              1]])  
    # Homogeneous transformation matrix from frame 2 to frame 3
    i = 2
    T23 = np.array([[np.cos(dh_table[i,0]),    -np.sin(dh_table[i,0])*np.cos(dh_table[i,1]),    np.sin(dh_table[i,0])*np.sin(dh_table[i,1]),    dh_table[i,2]*np.cos(dh_table[i,0])],
                    [np.sin(dh_table[i,0]),     np.cos(dh_table[i,0])*np.cos(dh_table[i,1]),   -np.cos(dh_table[i,0])*np.sin(dh_table[i,1]),    dh_table[i,2]*np.sin(dh_table[i,0])],
                    [0,                         np.sin(dh_table[i,1]),                          np.cos(dh_table[i,1]),                          dh_table[i,3]],
                    [0,                         0,                                              0,                                              1]])  

    ## calculate the transformation matrix for the end effector
    T02 = T01 @ T12
    T03 = T02 @ T23

    ## extract the position of the end effector
    x = T03[0,3]
    y = T03[1,3]
    z = T03[2,3]

    ## print the position of the end effector
    print("Angle: ({:.2f}, {:.2f}, {:.2f})   EE-pos: ({:.2f}, {:.2f}, {:.2f})".format(theta1,theta2,theta3,x,y,z))

    ## clear axes plot
    ax.cla()
    
    ## redraw plot
    ax.plot([0, T01[0,3]],          [0, T01[1,3]],          [0, T01[2,3]],          'r')
    ax.plot([T01[0,3], T02[0,3]],   [T01[1,3], T02[1,3]],   [T01[2,3], T02[2,3]],   'g')
    ax.plot([T02[0,3], x],          [T02[1,3], y],          [T02[2,3], z],          'b')
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
