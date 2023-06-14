## -- DEPENDENCIES ------------------------------------------ ##
from sys import getsizeof
import serial, struct
from manipulator import *
from controller import *
from kalman_filter import *

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from time import time, sleep


## -- VARIABLES --------------------------------------------- ##
c_pose = [[],[]] # present and last value of controller position
r_pose = [[],[]] # present and last value of robot position
ratio = 2   # ratio of controller : actuator movement.

grasp = 0
theta1 = 0
theta2 = 0
theta3 = 0
d3     = 0
theta4 = 0
theta5 = 0
theta6 = 0
# theta1 = 0#var
# theta2 = 45#var
# theta3 = -180#var
# theta4 = 0#var
# theta5 = 90#var
# theta6 = 0#var


## -- SETUP ------------------------------------------------- ##
## confgure serial communication
try:
    ser = serial.Serial('COM3', 38400, timeout=0.1)
    ser.reset_input_buffer()
    print("serial connected!")
except:
    print("serial not connected!")
## turn on interactive plot
plt.ion()
## create a 3D figure
fig = plt.figure()
ax1 = fig.add_subplot(121, projection='3d')
ax2 = fig.add_subplot(122, projection='3d')

controller = Controller()
manipulator = Manipulator()


## -- INITIATE ---------------------------------------------- ##
# controller.plot_workspace(ax1, 20)
# manipulator.plot_workspace(ax2, 5)
c_pose[1] = np.array(controller.forward_kinematics(rep='quaternion'))    # initial controller pose
r_pose[1] = np.array(manipulator.forward_kinematics(rep='quaternion'))   # initial robot pose

t1 = time()
## -- LOOP ------------------------------------------------- ##
try:
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
            ser.write(b'@')
            while(1):
                if ser.read(1) == b'$':
                    data = ser.read_until(b'&')
                    data = data[1:len(data)-1]

                    value = struct.unpack('7H', data) # unsigned short 2-byte
                    grasp = np.round(fsr_read(value[0]), 3)
                    theta1 = int(link1_read(value[1]))
                    theta2 = int(link2_read(value[2]))
                    theta3 = int(link3_read(value[3], theta2))
                    theta4 = int(link4_read(value[4]))
                    theta5 = int(link5_read(value[5]))
                    theta6 = int(link6_read(value[6]))
                        
                    break
            ## print received data
            # print([grasp, theta1, theta2, theta3, theta4, theta5, theta6])
        except:
            print("data lost!")
            try: ## try to reconnect the serial
                ser.close()
                ser = serial.Serial('COM3', 38400, timeout=0.1)
                ser.reset_input_buffer()
                print("serial connected!")
            except:
                print("serial not connected!")
        
        
        # controller.differential_inverse_kinematics([50.0, 50.0, 250.0], [45, 0, 0])
        # controller.forward_kinematics(q = [theta1, theta2, theta3, theta4, theta5, theta6])
        # controller.update_plot(fig, ax1, trace='on')
        
        # manipulator.differential_inverse_kinematics([200.9458, -136.7872, 29.8378], [0.0, 0.0, -45.0])
        # manipulator.forward_kinematics(q = [theta1, theta2, d3, theta4, theta5, theta6])
        # manipulator.update_plot(fig, ax2, trace='on')

        c_pose[0] = np.array(controller.forward_kinematics(q = [theta1, theta2, theta3, theta4, theta5, theta6], rep='quaternion')) # new/updated controller position
        if grasp >= 0.8:
            ## position adjustment
            delta_c_position = c_pose[0][:3] - c_pose[1][:3]    # movement of controller pose
            delta_r_position = 1/ratio*delta_c_position         # movement of robot pose, in ratio with movement of controller position
            r_pose[0][:3] = r_pose[1][:3] + delta_r_position    # actual / desired robot pose
            ## orientation adjustment
            r_pose[0][3:] = c_pose[0][3:]
            ## differential IK
            manipulator.differential_inverse_kinematics(r_pose[0][:3], r_pose[0][3:])
            ## update variables
            r_pose[1] = r_pose[0]
        c_pose[1] = c_pose[0]
            
        controller.update_plot(fig, ax1, trace='on')
        manipulator.update_plot(fig, ax2, trace='on')

        plt.pause(0.05)
        print(time()-t1)
        t1 = time()
except:
    try:
        ser.close()
    except:
        pass
    quit()