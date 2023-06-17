## -- DEPENDENCIES ------------------------------------------ ##
# from sys import getsizeof
import serial, struct
import numpy as np
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D
# from time import time, sleep
## private libraries
from manipulator import *
from controller import *
from kalmanFilter import *
from manipulator_outer_surface_point import *


## -- VARIABLES --------------------------------------------- ##
c_pose = [[],[]] # present and last value of controller position
r_desired_pose = [[],[]] # present and last value of robot position
ratio = 2 # ratio of controller : actuator movement

grasp = 0
theta1 = 0
theta2 = 0
theta3 = 0
d3     = 0
theta4 = 0
theta5 = 0
theta6 = 0


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
filter1 = KalmanFilter(R=[[0.062]])
filter2 = KalmanFilter(R=[[0.022]])
filter3 = KalmanFilter(R=[[0.043]])
filter4 = KalmanFilter(R=[[0.800]])
filter5 = KalmanFilter(R=[[0.910]])
filter6 = KalmanFilter(R=[[1.090]])


## -- INITIATE ---------------------------------------------- ##
# controller.plot_outer_workspace(res=25, axes='yz')
# manipulator.plot_outer_workspace(res=25, axes='yz')

c_pose[1] = np.array(controller.forward_kinematics(rep='quaternion'))    # initial controller pose
r_desired_pose[1] = np.array(manipulator.forward_kinematics(rep='quaternion'))   # initial robot pose


## -- LOOP ------------------------------------------------- ##
try:
    while True:
        # t1 = time()
        # theta1 += 1
        # theta2 += 1
        # theta3 += 1
        # d3     += 1
        # theta4 += 1
        # theta5 += 1
        # theta6 += 1

        # controller.differential_inverse_kinematics([50.0, 50.0, 250.0], [45, 0, 0])
        # controller.forward_kinematics(q = [theta1, theta2, theta3, theta4, theta5, theta6])
        # controller.update_plot(fig, ax1, trace='on')
        
        # manipulator.differential_inverse_kinematics([200.9458, -136.7872, 29.8378], [0.0, 0.0, -45.0])
        # manipulator.forward_kinematics(q = [theta1, theta2, d3, theta4, theta5, theta6])
        # manipulator.update_plot(fig, ax2, trace='on')


        ## read joint angle from serial
        try:
            ser.write(b'@')
            while(1):
                if ser.read(1) == b'$':
                    data = ser.read_until(b'&')
                    data = data[1:len(data)-1]

                    value = struct.unpack('7H', data) # unsigned short 2-byte
                    # grasp = value[0]
                    # theta1 = value[1]
                    # theta2 = value[2]
                    # theta3 = value[3]
                    # theta4 = value[4]
                    # theta5 = value[5]
                    # theta6 = value[6]
                    grasp = np.round(fsr_read(value[0]), 3)
                    theta1 = int(link1_read(filter1.step(0.087890625*value[1], mode='circural')))
                    theta2 = int(link2_read(filter2.step(0.087890625*value[2], mode='circural')))
                    theta3 = int(link3_read(filter3.step(0.087890625*value[3], mode='circural'), theta2))
                    theta4 = int(link4_read(filter4.step(0.087890625*value[4], mode='circural')))
                    theta5 = int(link5_read(filter5.step(0.087890625*value[5], mode='circural')))
                    theta6 = int(link6_read(filter6.step(0.087890625*value[6], mode='circural')))
                    break
            ## print received data
            print([grasp, theta1, theta2, theta3, theta4, theta5, theta6])
        except:
            print("data lost!")
            try: ## try to reconnect the serial
                ser.close()
                ser = serial.Serial('COM3', 38400, timeout=0.1)
                ser.reset_input_buffer()
                print("serial connected!")
            except:
                print("serial not connected!")
        
        ## update controller pose
        c_pose[0] = np.array(controller.forward_kinematics(q = [theta1, theta2, theta3, theta4, theta5, theta6], rep='quaternion')) # new/updated controller position 
        ## check button for movement
        if grasp >= 0.7:
            ## position adjustment
            delta_c_position = c_pose[0][:3] - c_pose[1][:3]    # movement of controller pose
            delta_r_position = 1/ratio*delta_c_position         # movement of robot pose, in ratio with movement of controller position
            r_desired_pose[0][:3] = r_desired_pose[1][:3] + delta_r_position    # actual / desired robot pose
            

            ## check if desired point exceed the workspace limit
            distance = None
            ## search for nearest distance of the outer surface point and the desired point
            desired_point_distance = distance_3d(r_desired_pose[0][0], 0, r_desired_pose[0][1], 0, r_desired_pose[0][2], 0)
            nearest_distance = abs(dist_out_surf[0] - desired_point_distance)
            index = 0
            for i in range(dist_out_surf): 
                ## get: distance = (x_desired^2 - x_out_surf^2)^0.5
                distance = dist_out_surf[i]
                if abs(distance - desired_point_distance) < nearest_distance:
                    nearest_distance = distance
                    index = i
            if desired_point_distance > distance:
                r_desired_pose[0][:3] = [x_out_surf[index], y_out_surf[index], z_out_surf[index]]


            ## orientation adjustment
            r_desired_pose[0][3:] = c_pose[0][3:]
            ## differential IK
            manipulator.differential_inverse_kinematics(r_desired_pose[0][:3], r_desired_pose[0][3:])
            ## update variables
            r_desired_pose[1] = r_desired_pose[0]        
        c_pose[1] = c_pose[0]
        # print(r_desired_pose[0])

        controller.update_plot(ax1, trace='on')
        manipulator.update_plot(ax2, trace='on')
        plt.pause(0.05)
        # print(time()-t1)
except:
    try:
        ser.close()
    except:
        pass
    quit()