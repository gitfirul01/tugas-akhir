## -- DEPENDENCIES ------------------------------------------ ##
# import serial, struct
from manipulator import *
from controller import *

import numpy as np
import sympy as sp
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

## -- SETUP ------------------------------------------------- ##
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
ax1 = fig.add_subplot(121, projection='3d')
ax2 = fig.add_subplot(122, projection='3d')

controller = Controller()
manipulator = Manipulator()



theta1 = 0
theta2 = 0
theta3 = 0
d3     = 0
theta4 = 0
theta5 = 0
theta6 = 0

## -- LOOP ------------------------------------------------- ##
while True:
    # theta1 += 1
    # theta2 += 1
    # theta3 += 1
    # d3     += 1
    # theta4 += 1
    # theta5 += 1
    # theta6 += 1

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
    
    '''
    Target:
    Joint [0, 90, -90, 0, 0, 0]
    Pose  [0.0, 0.0, 345.0, 0.0, 90.0, 0.0]
          [0.0, 0.0, 345.0, 0.7071, 0.0, 0.7071, -0.0]
    '''
    # controller.differential_inverse_kinematics([50.0, 50.0, 300.0, 10.0, 80.0, 10.0])
    # controller.differential_inverse_kinematics([50.0, 50.0, 300.0, 0.7071, 0.0, 0.0, 0.0], 'quaternion')
    # controller.differential_inverse_kinematics([50.0, 50.0, 300.0], [45, 0, 0])
    # controller.forward_kinematics(q = [theta1, theta2, theta3, theta4, theta5, theta6])
    # controller.update_plot(fig, ax1)

    '''
    Joint  [45, 50, 100, -180, -90, 0]
    Pose   [319.324, -0.0, 217.9868, 90.0, 53.0, 180.0]
           [319.324, -0.0, 217.9868, 0.3155, -0.3155, 0.6328, 0.6328]
    '''
    manipulator.differential_inverse_kinematics([319, 0, 217], [90.0, 53.0, 180.0])
    # manipulator.forward_kinematics(q = [theta1, theta2, d3, theta4, theta5, theta6])
    manipulator.update_plot(fig, ax2)
    
    plt.pause(0.001)