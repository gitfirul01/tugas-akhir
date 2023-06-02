## -- DEPENDENCIES ------------------------------------------ ##
from sys import getsizeof
import serial, struct
# from manipulator import *
from manipulator_v2 import *
from controller import *

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from time import time

## -- SETUP ------------------------------------------------- ##
## confgure serial communication
try:
    ser = serial.Serial('COM3', 115200, timeout=1)
    ser.reset_input_buffer()
    print("Serial connected!")
except:
    print("Serial not connected!")
    pass

## turn on interactive plot
plt.ion()
## create a 3D figure
fig = plt.figure()
ax1 = fig.add_subplot(221, projection='3d')
ax2 = fig.add_subplot(222, projection='3d')
ax3 = fig.add_subplot(223, projection='3d')
ax4 = fig.add_subplot(224, projection='3d')

controller = Controller()
manipulator = Manipulator()

grasp = 0
theta1 = 0
theta2 = 0
theta3 = 0
d3     = 0
theta4 = 0
theta5 = 0
theta6 = 0

# controller.plot_workspace(ax1)
pos = [0,0,0]
prev_pos = [0,0,0]
ax3.set_xlabel('X')
ax3.set_ylabel('Y')
ax3.set_zlabel('Z')
ax3.set_xlim(0, 350)
ax3.set_ylim(-175, 175)
ax3.set_zlim(0, 350)
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
    try:
        if ser.read(1) == b'@':
            data = ser.read_until(b'$')
            data = data[1:len(data)-1]
            try:
                value = struct.unpack('fffffff', data)
                grasp = value[0]
                theta1 = value[1]
                theta2 = value[2]
                theta3 = value[3]
                theta4 = value[4]
                theta5 = value[5]
                theta6 = value[6]
            except:
                pass
    except:
        pass
    
    controller.differential_inverse_kinematics([50.0, 50.0, 250.0], [45, 0, 0])
    # controller.forward_kinematics(q = [theta1, theta2, theta3, theta4, theta5, theta6])
    controller.update_plot(fig, ax1)
    
    pos = controller.forward_kinematics()[:3]
    ax3.plot([prev_pos[0], pos[0]], [prev_pos[1], pos[1]], [prev_pos[2], pos[2]], 'b-')
    prev_pos = pos
    
    # manipulator.differential_inverse_kinematics([200.9458, -136.7872, 29.8378], [0.0, 0.0, 0.0])
    # manipulator.forward_kinematics(q = [theta1, theta2, d3, theta4, theta5, theta6])
    # manipulator.update_plot(fig, ax2)
    
    # print("grasp  :", grasp)
    # print("theta1 :", theta1)
    # print("theta2 :", theta2)
    # print("theta3 :", theta3)
    # print("d3     :", d3)
    # print("theta4 :", theta4)
    # print("theta5 :", theta5)
    # print("theta6 :", theta6)

    # print(time())
    plt.show()
    plt.pause(0.001)