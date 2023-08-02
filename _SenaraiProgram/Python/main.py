## -- DEPENDENCIES ------------------------------------------ ##
import serial, struct
import numpy as np
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D
from time import time#, sleep
# from sys import getsizeof

from manipulator import *
from controller import *
from filter import *
from function import *


## -- VARIABLES --------------------------------------------- ##
RAW_TO_DEG = 360.0 / 4096    # raw to degree scale factor
REACH      = False           # controller end effecter reach manipulator end effector

BAUDRATE   = 38400           # serial baudrate
TIMEOUT    = 0.1             # serial timeout
PORT_CONTROLLER  = 'COM3'    # serial port
PORT_MANIPULATOR = 'COM33'   # serial port

c_pose = [[0,0,0,0,0,0,0],[0,0,0,0,0,0,0]]          # present and last value of controller position
r_desired_pose = [[0,0,0,0,0,0,0],[0,0,0,0,0,0,0]]  # present and last value of robot position
ratio = 4                                           # ratio of controller:actuator movement for control mode 2

jaw = 0
theta1 = 0
theta2 = 0
theta3 = 0
d3     = 0
theta4 = 0
theta5 = 0
theta6 = 0


## -- FUNCTION ---------------------------------------------- ##
def serial_connect_to_(serial_port, baudrate, timeout=TIMEOUT):
    try:
        ser = serial.Serial(serial_port, baudrate, timeout=timeout)
        ser.reset_input_buffer()
        print("Serial connected to", serial_port, "at", baudrate, "bps")
        return ser
    except:
        print("Serial connection failed")

def ser_read_joint_angle(serial):
    serial.write(b'@') # initiate communication with STM32
    while(1):
        if serial.read(1) == b'$': # check header byte send by STM32
            data = serial.read_until(b'&') # read until trailer byte send by STM32
            data = data[1:len(data)-1]
            value = struct.unpack('7H', data) # unsigned short 2-byte

            return value[0], value[1], value[2], value[3], value[4], value[5], value[6]

def ser_send_joint_angle(serial, value, jaw):
    serial.write(b'%') # initiate communication with STM32
    if value is not None:
        serial.write(
            struct.pack('7h', value[0], value[1], value[2], value[3], value[4], value[5], jaw) # pack for 7 short (int) variable
            )
    serial.write(b'^') # end communication with STM32

def trajectory(ax, point1, point2, color='y'):
    '''
    @ax: axes instance
    @point1: initial point
    @point2: desired point
    '''
    ## plot desired point
    ax.scatter(point2[0], point2[1], point2[2], color = color)
    ## plot trajectory line
    ax.plot([point1[0], point2[0]], # x
            [point1[1], point2[1]], # y
            [point1[2], point2[2]], # z
            linestyle='--', color = 'c')



if __name__ == "__main__":
    ## -- SETUP ------------------------------------------------- ##
    ser_controller = serial_connect_to_(PORT_CONTROLLER, BAUDRATE, TIMEOUT)   # confgure serial communication
    ser_manipulator = serial_connect_to_(PORT_MANIPULATOR, BAUDRATE, TIMEOUT) # confgure serial communication

    plt.ion()
    fig = plt.figure()
    ax1 = fig.add_subplot(121, projection='3d')
    ax2 = fig.add_subplot(122, projection='3d')
    # fig2, ax3 = plt.subplots(subplot_kw={'projection': '3d'})

    ## Robot Instance
    controller = Controller()
    manipulator = Manipulator()

    ## Initial Variable
    c_pose[1] = np.array(controller.forward_kinematics(rep='quaternion'))           # initial controller pose
    r_desired_pose[1] = np.array(manipulator.forward_kinematics(rep='quaternion'))  # initial robot pose
    r_desired_pose[0] = r_desired_pose[1]                                           # initial robot pose
    try:
        x0 = ser_read_joint_angle(ser_controller)
    except:
        x0 = [0,0,0,0,0,0]

    ## Filter Instance
    Filter1 = EMA(alpha=0.5, x0=RAW_TO_DEG*x0[0])
    Filter2 = EMA(alpha=0.5, x0=RAW_TO_DEG*x0[1])
    Filter3 = EMA(alpha=0.5, x0=RAW_TO_DEG*x0[2])
    Filter4 = EMA(alpha=0.5, x0=RAW_TO_DEG*x0[3])
    Filter5 = EMA(alpha=0.5, x0=RAW_TO_DEG*x0[4])
    Filter6 = EMA(alpha=0.5, x0=RAW_TO_DEG*x0[5])

    # controller.plot_workspace_point(res=20, axes='xyz')
    # controller.plot_outer_workspace_point(res=20, axes='xy')
    # controller.plot_outer_workspace_wireframe(ax=ax3, res=15, color='g', alpha=0.2)
    # manipulator.plot_workspace_point(res=20, axes='xyz')
    # manipulator.plot_outer_workspace_point(res=20, axes='xy')
    # manipulator.plot_outer_workspace_wireframe(ax=ax3, res=15, color='b', alpha=0.2)



    ## -- LOOP ------------------------------------------------- ##
    while True:
        '''
        controller.differential_inverse_kinematics([100,200,300],[90,90,90])
        manipulator.differential_inverse_kinematics([100,200,300],[90,90,90])
        controller.update_plot(ax1)
        manipulator.update_plot(ax2)
        '''

        ## ----- READ JOINT ANGLE ----- ##
        try:
            jaw, raw1, raw2, raw3, raw4, raw5, raw6 = ser_read_joint_angle(ser_controller)

            # theta1 = link1_read(RAW_TO_DEG*raw1)
            # theta2 = link2_read(RAW_TO_DEG*raw2)
            # theta3 = link3_read(RAW_TO_DEG*raw3, theta2)
            # theta4 = link4_read(RAW_TO_DEG*raw4)
            # theta5 = link5_read(RAW_TO_DEG*raw5)
            # theta6 = link6_read(RAW_TO_DEG*raw6)

            theta1 = link1_read(Filter1.step(RAW_TO_DEG*raw1, ang_avg=True))
            theta2 = link2_read(Filter2.step(RAW_TO_DEG*raw2, ang_avg=True))
            theta3 = link3_read(Filter3.step(RAW_TO_DEG*raw3, ang_avg=True), theta2)
            theta4 = link4_read(Filter4.step(RAW_TO_DEG*raw4, ang_avg=True))
            theta5 = link5_read(Filter5.step(RAW_TO_DEG*raw5, ang_avg=True))
            theta6 = link6_read(Filter6.step(RAW_TO_DEG*raw6, ang_avg=True))

            # print([
            #     raw1, raw2, raw3, raw4, raw5, raw6,
            #     np.round(theta1,3),np.round(theta2,3),np.round(theta3,3),np.round(theta4,3),np.round(theta5,3),np.round(theta6,3)
            # ]) # print received data 
        except:
            try:
                print("Controller data lost! reconnecting . . .")
                ser_controller.close()
                ser_controller = serial_connect_to_(PORT_CONTROLLER, BAUDRATE, TIMEOUT)
            except:
                pass
            

        ## ----- MOVING THE ROBOT ----- ##
        c_pose[0] = np.array(
            controller.forward_kinematics(                                              ## update controller pose
                q = [theta1, theta2, theta3, theta4, theta5, theta6], rep='quaternion'  ## new/updated controller position 
                )
            )


        ## ----- CONTROL MODE 1 ----- ##
        ## add the re-mapping value
        s = [1.2, 0.55, 0.5]
        t = [-140, 280, -15]
        c_pose[0][0] = s[0]*c_pose[0][0] + t[0]
        c_pose[0][1] = s[1]*c_pose[0][1] + t[1]
        c_pose[0][2] = s[2]*c_pose[0][2] + t[2]
        ## matching the pose
        if np.linalg.norm(c_pose[0] - r_desired_pose[0]) <= 20: # check for match
            REACH = True
        if REACH: # if match, then manipulator controlled
            # c_pose[0][:3] = 1/ratio*c_pose[0][:3] # add ratio for robot position movement
            r_desired_pose[0] = c_pose[0]
        ## ----- END OF CONTROL MODE 1 ----- ##

        
        # ## ----- CONTROL MODE 2 ----- ##
        # ## check command for movement
        # if jaw >= 0.7*2000:
        #     REACH = True # force reach variable to True, so that the desired point will generated
        #     ## position adjustment
        #     delta_c_position = c_pose[0][:3] - c_pose[1][:3]                    # movement of controller pose
        #     delta_r_position = 1/ratio*delta_c_position                         # movement of robot pose, in ratio with movement of controller position
        #     r_desired_pose[0][:3] = r_desired_pose[1][:3] + delta_r_position    # actual / desired robot pose
        #     ## orientation adjustment
        #     r_desired_pose[0][3:] = c_pose[0][3:]
        # ## ----- END OF CONTROL MODE 2 ----- ##


            ## ----- WORKSPACE LIMIT CHECK ----- ##
            ## check if desired point exceed the workspace limit
            index = 0
            nearest_distance = 999999
            ## find desired_point_distance from origin (0, 250, 50 is the estimated origin of the manipulator workspace)
            desired_point_distance_to_origin = point_distance(r_desired_pose[0][0], 0,
                                                              r_desired_pose[0][1], 250, 
                                                              r_desired_pose[0][2], 50)
            ## search for nearest distance from the outer surface point and the desired point
            for i in range(len(surface_point[0])):
                ## find the distance of surface_point to desired_point
                surface_point_to_desired_point = point_distance(surface_point[0][i], r_desired_pose[0][0], 
                                                                surface_point[1][i], r_desired_pose[0][1], 
                                                                surface_point[2][i], r_desired_pose[0][2])
                ## find nearest distance of surface_point and desired_point_distance
                if surface_point_to_desired_point < nearest_distance:
                    nearest_distance = surface_point_to_desired_point
                    index = i
            ## check if the desired point distance (from origin) exceed the outer surface point distance (from origin)
            if desired_point_distance_to_origin > point_distance(surface_point[0][index], 0, 
                                                                 surface_point[1][index], 250, 
                                                                 surface_point[2][index], 50):
                print("WARNING! Exceed the workspace:", [surface_point[0][index], surface_point[1][index], surface_point[2][index]])
                ## constraint the desired position
                r_desired_pose[0][:3] = [surface_point[0][index], surface_point[1][index], surface_point[2][index]]


            ## ----- DIFFERENTIAL IK ----- ##
            desired_joint = manipulator.differential_inverse_kinematics(r_desired_pose[0][:3], r_desired_pose[0][3:])            
            desired_joint[0] = int(desired_joint[0])
            desired_joint[1] = int(desired_joint[1])
            desired_joint[2] = int(desired_joint[2])
            desired_joint[3] = int(desired_joint[3])
            desired_joint[4] = int(desired_joint[4])
            desired_joint[5] = int(desired_joint[5])
            ## send the desired_joint configuration to manipulator STM32
            try:
                ser_send_joint_angle(ser_manipulator, desired_joint, jaw)
            except:
                try:
                    print("Manipulator failed to send joint data! reconnecting . . .")
                    ser_manipulator.close()
                    ser_manipulator = serial_connect_to_(PORT_MANIPULATOR, BAUDRATE, TIMEOUT)
                except:
                    pass
                    
            ## update manipulator robot variables
            r_desired_pose[1] = r_desired_pose[0]
        ## update controller robot variables
        c_pose[1] = c_pose[0]
        # print(np.round(r_desired_pose[0], 3).tolist())


        ## ----- PLOTTING THE ROBOT ----- ##
        ## plot link frame
        controller.update_plot(ax1, trace='off')
        manipulator.update_plot(ax2, trace='off')

        ## plot workspace
        # controller.plot_outer_workspace_wireframe(ax = ax1, res=15, color='g', alpha=0.2)
        manipulator.plot_outer_workspace_wireframe(ax = ax2, res=15, color='b', alpha=0.2)
        
        if REACH: # if controller has reached the point
            trajectory(ax2, manipulator.forward_kinematics(rep='quaternion'), r_desired_pose[0], color='g')
        else:
            ## point plot
            trajectory(ax2, manipulator.forward_kinematics(rep='quaternion'), c_pose[0]) # c_pose[0] has been scalled before
            ## generate the frame coordinate
            u = controller.T0_E[:3, :3] @ np.array([[1, 0, 0]]).T
            v = controller.T0_E[:3, :3] @ np.array([[0, 1, 0]]).T
            w = controller.T0_E[:3, :3] @ np.array([[0, 0, 1]]).T
            ## scalled the frame coordinate point, to displayed in manipulator frame
            s = [1.2, 0.55, 0.5]
            t = [-140, 280, -15]
            x = controller.T0_E[0, 3]*s[0] + t[0]
            y = controller.T0_E[1, 3]*s[1] + t[1]
            z = controller.T0_E[2, 3]*s[2] + t[2]

            ax2.quiver(x, y, z, u[0, 0], u[1, 0], u[2, 0], color='r', length=25)
            ax2.quiver(x, y, z, v[0, 0], v[1, 0], v[2, 0], color='g', length=25)
            ax2.quiver(x, y, z, w[0, 0], w[1, 0], w[2, 0], color='b', length=25)

        plt.pause(0.05)
