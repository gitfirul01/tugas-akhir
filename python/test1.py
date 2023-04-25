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


    # define relative coordinate frames for each link
    O = np.array([[0, 0, 0]]).T
    X1 = T01[:3, :3] @ np.array([[1, 0, 0]]).T
    Y1 = T01[:3, :3] @ np.array([[0, 1, 0]]).T
    Z1 = T01[:3, :3] @ np.array([[0, 0, 1]]).T

    X2 = T02[:3, :3] @ np.array([[1, 0, 0]]).T
    Y2 = T02[:3, :3] @ np.array([[0, 1, 0]]).T
    Z2 = T02[:3, :3] @ np.array([[0, 0, 1]]).T

    X3 = T03[:3, :3] @ np.array([[1, 0, 0]]).T
    Y3 = T03[:3, :3] @ np.array([[0, 1, 0]]).T
    Z3 = T03[:3, :3] @ np.array([[0, 0, 1]]).T

    ax.quiver(T01[0, 3], T01[1, 3], T01[2, 3], X1[0, 0], X1[1, 0], X1[2, 0], color='r', length=25)
    ax.quiver(T01[0, 3], T01[1, 3], T01[2, 3], Y1[0, 0], Y1[1, 0], Y1[2, 0], color='g', length=25)
    ax.quiver(T01[0, 3], T01[1, 3], T01[2, 3], Z1[0, 0], Z1[1, 0], Z1[2, 0], color='b', length=25) 
    
    ax.quiver(T02[0, 3], T02[1, 3], T02[2, 3], X2[0, 0], X2[1, 0], X2[2, 0], color='r', length=25)
    ax.quiver(T02[0, 3], T02[1, 3], T02[2, 3], Y2[0, 0], Y2[1, 0], Y2[2, 0], color='g', length=25)
    ax.quiver(T02[0, 3], T02[1, 3], T02[2, 3], Z2[0, 0], Z2[1, 0], Z2[2, 0], color='b', length=25) 
    
    ax.quiver(T03[0, 3], T03[1, 3], T03[2, 3], X3[0, 0], X3[1, 0], X3[2, 0], color='r', length=25)
    ax.quiver(T03[0, 3], T03[1, 3], T03[2, 3], Y3[0, 0], Y3[1, 0], Y3[2, 0], color='g', length=25)
    ax.quiver(T03[0, 3], T03[1, 3], T03[2, 3], Z3[0, 0], Z3[1, 0], Z3[2, 0], color='b', length=25) 
    

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
