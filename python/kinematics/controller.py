'''
    CONTROLLER::PUMA560
    EULER CONVENTION: ZYX

    HOME POSITION   V-STRAIGHT      H-STRAIGHT      NOMINAL POSE
    | theta1 = 0    | theta1 = 0    | theta1 = 0    | theta1 = 0
    | theta2 = 0    | theta2 = 90   | theta2 = 0    | theta2 = 45
    | theta3 = 0    | theta3 = -90  | theta3 = -90  | theta3 = -180
    | theta4 = 0    | theta4 = 0    | theta4 = 0    | theta4 = 0
    | theta5 = 0    | theta5 = 0    | theta5 = 0    | theta5 = 45
    | theta6 = 0    | theta6 = 0    | theta6 = 0    | theta6 = 0

'''

from function import *

# import serial, struct
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


## confgure serial communication
# try:
#     ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
#     ser.reset_input_buffer()
# except:
#     print("Serial not connected!")
#     exit()
## turn on interactive plot
plt.ion()
## create a 3D figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')


## joint angle (x_i to xi over z_i)
theta1 = 0#var
theta2 = 45#var
theta3 = -180#var
theta4 = 0#var
theta5 = 90#var
theta6 = 0#var
## joint offset (O_i to xi over z_i)
d1 = 45#
d2 = 0
d3 = 0
d4 = 150#
d5 = 0
d6 = 0
## link lengths (Oi to z_i over xi)
a1 = 0
a2 = 150#
a3 = 0
a4 = 0
a5 = 0
a6 = 0
## link twist (z_i to zi over xi)
alpha1 = 90#
alpha2 = 0
alpha3 = -90#
alpha4 = 90#
alpha5 = -90#
alpha6 = 0


while True:
    # theta1+=1
    # theta2+=1
    # theta3+=1
    # theta4+=1
    # theta5+=1
    # theta6+=1

    ## read joint angle from serial
    # if ser.read(1) == b'@':
    #     data = ser.read_until(b'$')
    #     data = data[1:len(data)-1]
    #     try:
    #         value = struct.unpack('fff', data)
    #         theta1 = value[0]
    #         theta2 = value[1]
    #         theta3 = value[2]
    #     except:
    #         pass

    ## homogeneous transformation matrix from frame i to frame j (T_ij)
    T01 = dh2tf(theta1, d1, a1, alpha1)
    T12 = dh2tf(theta2, d2, a2, alpha2)
    T23 = dh2tf(theta3, d3, a3, alpha3)
    T34 = dh2tf(theta4, d4, a4, alpha4)
    T45 = dh2tf(theta5, d5, a5, alpha5)
    T56 = dh2tf(theta6, d6, a6, alpha6)
    # additional plot for end-effector
    T6E = np.array([[np.cos(np.deg2rad(90)), 0, np.sin(np.deg2rad(90)), 0],
                    [0, 1, 0, 0],
                    [-np.sin(np.deg2rad(90)), 0, np.cos(np.deg2rad(90)), 0],
                    [0, 0, 0, 1]])
    ## calculate the transformation matrix for the end effector
    T02 = T01 @ T12
    T03 = T02 @ T23
    T04 = T03 @ T34
    T05 = T04 @ T45
    T06 = T05 @ T56
    ## additional transformation matrix for end-effector and pen
    T0E = T06 @ T6E
    T_PEN = T06 @ np.array([[1, 0, 0, 0], 
                            [0, 1, 0, 0], 
                            [0, 0, 1, 75],
                            [0, 0, 0, 1]])
    ## extract the position of the end effector
    x = T06[0,3]
    y = T06[1,3]
    z = T06[2,3]

    ## clear axes plot
    ax.cla()    
    ## redraw plot
    ax.plot([0, T01[0,3]],          [0, T01[1,3]],          [0, T01[2,3]],          'r')
    ax.plot([T01[0,3], T02[0,3]],   [T01[1,3], T02[1,3]],   [T01[2,3], T02[2,3]],   'g')
    ax.plot([T02[0,3], T03[0,3]],   [T02[1,3], T03[1,3]],   [T02[2,3], T03[2,3]],   'b')
    ax.plot([T03[0,3], T04[0,3]],   [T03[1,3], T04[1,3]],   [T03[2,3], T04[2,3]],   'r')
    ax.plot([T04[0,3], T05[0,3]],   [T04[1,3], T05[1,3]],   [T04[2,3], T05[2,3]],   'g')
    ax.plot([T05[0,3], x],          [T05[1,3], y],          [T05[2,3], z],          'b')
    ax.scatter(x, y, z)
    ## additional plot for pen
    ax.plot([x, T_PEN[0,3]],        [y, T_PEN[1,3]],        [z, T_PEN[2,3]],        'b')

    # add relative coordinate frames for each link
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

    X4 = T04[:3, :3] @ np.array([[1, 0, 0]]).T
    Y4 = T04[:3, :3] @ np.array([[0, 1, 0]]).T
    Z4 = T04[:3, :3] @ np.array([[0, 0, 1]]).T

    X5 = T05[:3, :3] @ np.array([[1, 0, 0]]).T
    Y5 = T05[:3, :3] @ np.array([[0, 1, 0]]).T
    Z5 = T05[:3, :3] @ np.array([[0, 0, 1]]).T

    X6 = T06[:3, :3] @ np.array([[1, 0, 0]]).T
    Y6 = T06[:3, :3] @ np.array([[0, 1, 0]]).T
    Z6 = T06[:3, :3] @ np.array([[0, 0, 1]]).T

    XE = T0E[:3, :3] @ np.array([[1, 0, 0]]).T
    YE = T0E[:3, :3] @ np.array([[0, 1, 0]]).T
    ZE = T0E[:3, :3] @ np.array([[0, 0, 1]]).T

    ax.quiver(T01[0, 3], T01[1, 3], T01[2, 3], X1[0, 0], X1[1, 0], X1[2, 0], color='r', length=25)
    ax.quiver(T01[0, 3], T01[1, 3], T01[2, 3], Y1[0, 0], Y1[1, 0], Y1[2, 0], color='g', length=25)
    ax.quiver(T01[0, 3], T01[1, 3], T01[2, 3], Z1[0, 0], Z1[1, 0], Z1[2, 0], color='b', length=25) 
    
    ax.quiver(T02[0, 3], T02[1, 3], T02[2, 3], X2[0, 0], X2[1, 0], X2[2, 0], color='r', length=25)
    ax.quiver(T02[0, 3], T02[1, 3], T02[2, 3], Y2[0, 0], Y2[1, 0], Y2[2, 0], color='g', length=25)
    ax.quiver(T02[0, 3], T02[1, 3], T02[2, 3], Z2[0, 0], Z2[1, 0], Z2[2, 0], color='b', length=25) 
    
    ax.quiver(T03[0, 3], T03[1, 3], T03[2, 3], X3[0, 0], X3[1, 0], X3[2, 0], color='r', length=25)
    ax.quiver(T03[0, 3], T03[1, 3], T03[2, 3], Y3[0, 0], Y3[1, 0], Y3[2, 0], color='g', length=25)
    ax.quiver(T03[0, 3], T03[1, 3], T03[2, 3], Z3[0, 0], Z3[1, 0], Z3[2, 0], color='b', length=25)

    ax.quiver(T04[0, 3], T04[1, 3], T04[2, 3], X4[0, 0], X4[1, 0], X4[2, 0], color='r', length=25)
    ax.quiver(T04[0, 3], T04[1, 3], T04[2, 3], Y4[0, 0], Y4[1, 0], Y4[2, 0], color='g', length=25)
    ax.quiver(T04[0, 3], T04[1, 3], T04[2, 3], Z4[0, 0], Z4[1, 0], Z4[2, 0], color='b', length=25)

    ax.quiver(T05[0, 3], T05[1, 3], T05[2, 3], X5[0, 0], X5[1, 0], X5[2, 0], color='r', length=25)
    ax.quiver(T05[0, 3], T05[1, 3], T05[2, 3], Y5[0, 0], Y5[1, 0], Y5[2, 0], color='g', length=25)
    ax.quiver(T05[0, 3], T05[1, 3], T05[2, 3], Z5[0, 0], Z5[1, 0], Z5[2, 0], color='b', length=25)

    ax.quiver(T06[0, 3], T06[1, 3], T06[2, 3], X6[0, 0], X6[1, 0], X6[2, 0], color='r', length=25)
    ax.quiver(T06[0, 3], T06[1, 3], T06[2, 3], Y6[0, 0], Y6[1, 0], Y6[2, 0], color='g', length=25)
    ax.quiver(T06[0, 3], T06[1, 3], T06[2, 3], Z6[0, 0], Z6[1, 0], Z6[2, 0], color='b', length=25) 

    ax.quiver(T0E[0, 3], T0E[1, 3], T0E[2, 3], XE[0, 0], XE[1, 0], XE[2, 0], color='r', length=35)
    ax.quiver(T0E[0, 3], T0E[1, 3], T0E[2, 3], YE[0, 0], YE[1, 0], YE[2, 0], color='g', length=35)
    ax.quiver(T0E[0, 3], T0E[1, 3], T0E[2, 3], ZE[0, 0], ZE[1, 0], ZE[2, 0], color='b', length=35) 

    ## set figure configuration
    # ax.set_title('3D Simulation')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim(0, 350)
    ax.set_ylim(-175, 175)
    ax.set_zlim(0, 350)
    
    ## print the position of the end effector
    print("Angle: ({:.2f}, {:.2f}, {:.2f})   EE-pos: ({:.2f}, {:.2f}, {:.2f})".format(theta1,theta2,theta3,x,y,z))
    # print(get_euler_convention(T06))
    print(get_euler_angle(T0E))
    # print(np.round(get_quaternion(T0E), 4))

    ## show the plot
    fig.canvas.draw()
    fig.canvas.flush_events()
    plt.pause(0.001)
