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
fig1 = plt.figure()
ax1 = fig1.add_subplot(111, projection='3d')

# fig2 = plt.figure()
# ax2 = fig2.add_subplot(111, projection='3d')


controller = Controller()
# manipulator = Manipulator()

q = get_quaternion(euler_to_rotation_matrix(45,0,0))

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
    # controller.inverse_kinematics([50.0, 50.0, 300.0, 10.0, 80.0, 10.0])
    # controller.inverse_kinematics([50.0, 50.0, 300.0, 0.7071, 0.0, 0.0, 0.0], 'quaternion')
    controller.inverse_kinematics([50.0, 50.0, 300.0, q[0], q[1], q[2], q[3]], 'quaternion')
#     controller.update_joint_variable([0, 90, -90, 0, 0, 0])
#     controller.update_joint_variable([0, 0, 0, 0, 0, 0])
    controller.update_plot(fig1, ax1)

    '''
    Joint  [45, 50, 100, -180, -90, 0]
    Pose   [250.7938, 12.2689, 138.4819, -165.33, 70.06, -112.83]
    '''
    # manipulator.inverse_kinematics([250.7938, 12.2689, 138.4819, -165.33, 70.06, -112.83])
    # manipulator.update_joint_variable([0, 90, -90, 0, 0, 0])
    # manipulator.update_plot(fig2, ax2)
    
    plt.pause(0.001)
