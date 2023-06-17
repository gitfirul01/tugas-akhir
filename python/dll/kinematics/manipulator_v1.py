'''
    ROBOT MANIPULATOR USING STANDARD DH-PARAMETER
'''
from function import *

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class Manipulator:
    def __init__(self):
        ## joint angle (x_i to xi over z_i)
        self.theta1 = 0#var
        self.theta2 = 0#var
        self.theta3 = 0
        self.theta4 = 0#var
        self.theta5 = 0#var
        self.theta6 = 0#var
        ## joint offset (O_i to xi over z_i)
        self.d1 = 180
        self.d2 = 0
        self.d3 = 0#var
        self.d4 = 200
        self.d5 = 0
        self.d6 = 0
        ## link lengths (Oi to z_i over xi)
        self.a1 = 240
        self.a2 = 300
        self.a3 = 0
        self.a4 = 0
        self.a5 = 20
        self.a6 = 30
        ## link twist (z_i to zi over xi)
        self.alpha1 = 90
        self.alpha2 = 0
        self.alpha3 = 0
        self.alpha4 = 0
        self.alpha5 = 90
        self.alpha6 = 90
        ## Transformation matrix
        self.T0_1 = None
        self.T0_2 = None
        self.T0_3 = None
        self.T0_4 = None
        self.T0_5 = None
        self.T0_6 = None
        self.T0_E = None
        self.update_tf_matrix()
        ## define joint limits
        self.joint_limits = [
            # [min, max]
            [-90, 0],
            [-30, 120],
            [0, 200],
            [-180, 180],
            [-180, 180],
            [-180, 180]
        ]

    def update_tf_matrix(self):
        ## homogeneous transformation matrix from frame 0 to frame 1
        self.T0_1 = Rz(90)        @ Tz(100)      @ Ty(0)        @ Ry(0)     @ Rx(90)     @ Tx(0)
        T1_2 = Rz(self.theta1)    @ Tz(180)      @ Ty(240)      @ Ry(0)     @ Rx(75)     @ Tx(0)  
        T2_3 = Rz(self.theta2)    @ Tz(0)        @ Ty(300)      @ Ry(0)     @ Rx(-128)   @ Tx(0)
        T3_4 = Rz(0)              @ Tz(-self.d3) @ Ty(0)        @ Ry(0)     @ Rx(0)      @ Tx(0)      
        T4_5 = Rz(self.theta4)    @ Tz(-self.d4) @ Ty(0)        @ Ry(0)     @ Rx(-90)    @ Tx(0)        
        T5_6 = Rz(self.theta5+90) @ Tz(0)        @ Ty(0)        @ Ry(0)     @ Rx(90)     @ Tx(20)  
        T6_E = Rz(self.theta6)    @ Tz(0)        @ Ty(0)        @ Ry(0)     @ Rx(0)      @ Tx(30)

        # self.T0_1 = Rz(0)      @ Tz(0)        @ Tx(0)        @ Rx(self.alpha1)   @ Ry(90)  @ Ty(100)
        # T1_2 = Rz(self.theta1) @ Tz(self.d1)  @ Tx(self.a1)  @ Rx(0)             @ Ry(-75) @ Ty(0)
        # T2_3 = Rz(self.theta2) @ Tz(0)        @ Tx(self.a2)  @ Rx(0)             @ Ry(128) @ Ty(0) @ Rz(self.theta3)
        # T3_4 = Rz(0)           @ Tz(-self.d3) @ Tx(0)        @ Rx(0)             @ Ry(0)   @ Ty(0)
        # T4_5 = Rz(self.theta4) @ Tz(-self.d4) @ Tx(0)        @ Rx(self.alpha5)   @ Ry(0)   @ Ty(0)
        # T5_6 = Rz(self.theta5) @ Tz(0)        @ Tx(self.a5)  @ Rx(self.alpha6)   @ Ry(0)   @ Ty(0)
        # T6_E = Rz(self.theta6) @ Tz(0)        @ Tx(self.a6)  @ Rx(0)             @ Ry(0)   @ Ty(0)

        # self.T0_1 = dh2tf(-90, 148, 0, -90)
        # T1_2 = dh2tf(self.theta1, 179, 240, 0)
        # T2_3 = dh2tf(-90, 0, 38.5, -75)
        # T3_4 = dh2tf(-90+self.theta2, 0, 0, -90)
        # T4_5 = dh2tf(0, 150, 0, -38)
        # T5_6 = dh2tf(0, 188+self.d3, 0, 90)
        # T67 = dh2tf(0, 122, 0, 90)
        # T78 = dh2tf(-90+self.theta4, 20, 0, -90)
        # T89 = dh2tf(-90+self.theta5, 0, 20, -90)
        # T9E = dh2tf(self.theta6, 0, 20, 0)
        # self.T0_2 = self.T0_1 @ T1_2
        # self.T0_3 = self.T0_2 @ T2_3
        # self.T0_4 = self.T0_3 @ T3_4
        # self.T0_5 = self.T0_4 @ T4_5
        # self.T0_6 = self.T0_5 @ T5_6
        # self.T0_7 = self.T0_6 @ T67
        # self.T0_8 = self.T0_7 @ T78
        # self.T0_9 = self.T0_8 @ T89
        # self.T0_E = self.T0_9 @ T9E

        ## calculate the transformation matrix for the end effector
        self.T0_2 = self.T0_1 @ T1_2
        self.T0_3 = self.T0_2 @ T2_3
        self.T0_4 = self.T0_3 @ T3_4
        self.T0_5 = self.T0_4 @ T4_5
        self.T0_6 = self.T0_5 @ T5_6
        self.T0_E = self.T0_6 @ T6_E

        return self.T0_E

    def update_plot(self, fig, ax):
        ## Function
        def plot_frame_coordinate(T, length = 25):
            ## add relative coordinate frames for each link
            O = np.array([[0, 0, 0]]).T
            X = T[:3, :3] @ np.array([[1, 0, 0]]).T
            Y = T[:3, :3] @ np.array([[0, 1, 0]]).T
            Z = T[:3, :3] @ np.array([[0, 0, 1]]).T

            ax.quiver(T[0, 3], T[1, 3], T[2, 3], X[0, 0], X[1, 0], X[2, 0], color='r', length=length)
            ax.quiver(T[0, 3], T[1, 3], T[2, 3], Y[0, 0], Y[1, 0], Y[2, 0], color='g', length=length)
            ax.quiver(T[0, 3], T[1, 3], T[2, 3], Z[0, 0], Z[1, 0], Z[2, 0], color='b', length=length) 

        def plot_frame_scatter(T, color = 'blue'):
            x = T[0,3]
            y = T[1,3]
            z = T[2,3]
            ax.scatter(x, y, z, color = color)

        def plot_link(T1, T2, color='r'):
            ax.plot([T1[0,3], T2[0,3]],   [T1[1,3], T2[1,3]],   [T1[2,3], T2[2,3]],   color)

        ## extract the position of the end effector
        x = self.T0_E[0,3]
        y = self.T0_E[1,3]
        z = self.T0_E[2,3]
        ## clear axes plot
        ax.cla()
        ## redraw plot
        plot_link(np.zeros(([4,4])), self.T0_1, 'b')
        plot_link(self.T0_1, self.T0_2, 'r')
        plot_link(self.T0_2, self.T0_3, 'r')
        plot_link(self.T0_3, self.T0_4, 'b')
        plot_link(self.T0_4, self.T0_5, 'r')
        plot_link(self.T0_5, self.T0_6, 'r')
        plot_link(self.T0_6, self.T0_E, 'r')
        
        # ax.plot([self.T0_6[0,3], self.T0_7[0,3]],   [self.T0_6[1,3], self.T0_7[1,3]],   [self.T0_6[2,3], self.T0_7[2,3]],   'g')
        # ax.plot([self.T0_7[0,3], self.T0_8[0,3]],   [self.T0_7[1,3], self.T0_8[1,3]],   [self.T0_7[2,3], self.T0_8[2,3]],   'g')
        # ax.plot([self.T0_8[0,3], self.T0_9[0,3]],   [self.T0_8[1,3], self.T0_9[1,3]],   [self.T0_8[2,3], self.T0_9[2,3]],   'g')
        # ax.plot([self.T0_9[0,3], x],               [self.T0_9[1,3], y],               [self.T0_9[2,3], z],               'b')
        
        ax.scatter(x, y, z)

        plot_frame_scatter(self.T0_1)
        plot_frame_scatter(self.T0_2)
        plot_frame_scatter(self.T0_3)
        plot_frame_scatter(self.T0_4)
        plot_frame_scatter(self.T0_5)
        plot_frame_scatter(self.T0_6)

        # plot_frame_coordinate(self.T0_1)
        # plot_frame_coordinate(self.T0_2)
        # plot_frame_coordinate(self.T0_3)
        # plot_frame_coordinate(self.T0_4)
        # plot_frame_coordinate(self.T0_5)
        # plot_frame_coordinate(self.T0_6)
        plot_frame_coordinate(self.T0_E)

        ## set figure configuration
        # ax.set_title('3D Simulation')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_xlim(0, 600)
        ax.set_ylim(-300, 300)
        ax.set_zlim(0, 600)
        
        ## print the position of the end effector
        # print("Joint ", self.get_joint_variable())
        # print("Pose  ", self.forward_kinematics(rep = 'quaternion'))
        # print(get_euler_convention(self.T0_E))

        ## show the plot
        fig.canvas.draw()
        fig.canvas.flush_events()

    def get_joint_variable(self, arg = 'degree'):
        if arg == 'degree':
            return [self.theta1, self.theta2, self.d3, self.theta4, self.theta5, self.theta6]
        elif arg == 'radian':
            return [np.deg2rad(self.theta1), np.deg2rad(self.theta2), self.d3, np.deg2rad(self.theta4), np.deg2rad(self.theta5), np.deg2rad(self.theta6)]

    def get_position_jacobian(self):
        '''
        @ Return [numpy.array] of Jacobian matrix
        '''
        # Define the unit vectors along the axes of rotation of each joint
        A = [0,0,0,0,0,0]
        A[0] = np.array([0, 0, 1])
        A[1] = self.T0_1[:3, 2]
        A[2] = self.T0_2[:3, 2]
        A[3] = self.T0_3[:3, 2]
        A[4] = self.T0_4[:3, 2]
        A[5] = self.T0_5[:3, 2]
        # Compute the position of the end effector in the base frame
        p = self.T0_E[:3, 3]
        # Compute the Jacobian matrix using the formula:
        Jp = np.zeros((3, 6))

        Jp[:, 0] = np.cross(A[0], p)
        Jp[:, 1] = np.cross(A[1], p - self.T0_1[:3, 3])
        Jp[:, 2] = A[2]
        # Jp[:, 3] = np.cross(A[3], p - self.T0_3[:3, 3])
        # Jp[:, 4] = np.cross(A[4], p - self.T0_4[:3, 3])
        # Jp[:, 5] = np.cross(A[5], p - self.T0_5[:3, 3])

        return Jp

    def get_orientation_jacobian(self):
        '''
        @ Return [numpy.array] of Jacobian matrix
        '''
        # Define the unit vectors along the axes of rotation of each joint
        A = [0,0,0,0,0,0]
        A[0] = np.array([0, 0, 1])
        A[1] = self.T0_1[:3, 2]
        A[2] = np.array([0, 0, 0])
        A[3] = self.T0_3[:3, 2]
        A[4] = self.T0_4[:3, 2]
        A[5] = self.T0_5[:3, 2]

        Jo = np.zeros((3, 6))

        Jo[:, 0] = A[0] # joint1
        Jo[:, 1] = A[1] # joint2
        Jo[:, 2] = A[2] # joint3
        Jo[:, 3] = A[3] # joint4
        Jo[:, 4] = A[4] # joint5
        Jo[:, 5] = A[5] # joint6

        return Jo

    def forward_kinematics(self, q = None, unit = 'degree', rep = 'euler'):
        '''
        Set the robot to joint configuration (q),
        update the transformation matrix,
        and then calculate for end-effector pose

        Parameter:
            q    = vector of joint variables
            rep  = orientation representation (euler / quaternion)
            unit = unit of angle (degree / radian)
        Output:
            set of pose
            [x, y, z, roll, pitch, yaw] if rep is 'euler'
            or
            [x, y, z, w, i, j, k] if rep is 'quaternion'
        '''
        ## update joint variables based on q
        if q != None:
            if unit == 'degree':
                self.theta1 = q[0] % 360
                self.theta2 = q[1] % 360
                self.d3     = q[2]
                self.theta4 = q[3] % 360
                self.theta5 = q[4] % 360
                self.theta6 = q[5] % 360
            elif unit == 'radian':
                self.theta1 = np.rad2deg(q[0]) % 360
                self.theta2 = np.rad2deg(q[1]) % 360
                self.d3     = q[2]
                self.theta4 = np.rad2deg(q[3]) % 360
                self.theta5 = np.rad2deg(q[4]) % 360
                self.theta6 = np.rad2deg(q[5]) % 360
        H = self.update_tf_matrix()
        ## extract the position of the end effector
        x = np.round(self.T0_E[0,3], 4)
        y = np.round(self.T0_E[1,3], 4)
        z = np.round(self.T0_E[2,3], 4)
        if rep == 'euler':
            roll, pitch, yaw = np.round(get_euler_angle(H), 4)
            return [x, y, z, roll, pitch, yaw]
        elif rep == 'quaternion':
            w, i, j, k = np.round(get_quaternion(H), 4)
            return [x, y, z, w, i, j, k]

    def differential_inverse_kinematics(self, position_d, orientation_d):
        position_thresh = 1e-4
        orientation_thresh = 1e-4
        Ko = 1
        Kp = 1
        alpha = 0.5

        ## pre-allocation
        q_d = np.zeros(6)       # desired joint configuration
        error_o = np.zeros(3)   # orientation error
        error_p = np.zeros(3)   # position error
        delta_q = np.zeros(6)   # delta q for joint configuration
        delta_p = np.zeros(6)   # delta p for pose

        ## first estimate
        q = self.get_joint_variable()

        ## convert euler angle to quaternion
        if len(orientation_d) != 4:
            orientation_d = get_quaternion(euler_to_rotation_matrix(orientation_d[0],orientation_d[1],orientation_d[2]))
        position_d = np.array(position_d)
        orientation_d = np.array(orientation_d)

        ## forward kinematics
        pose_current = self.forward_kinematics(rep = 'quaternion')
        position_current = np.array(pose_current[:3])
        orientation_current = np.array(pose_current[3:])

        ## jacobian matrix
        Jp = self.get_position_jacobian()
        Jo = self.get_orientation_jacobian()
        J = np.vstack((Jo, Jp))

        ## error
        error_o = np.array(-orientation_d[:1]*orientation_current[1:] + orientation_current[:1]*orientation_d[1:] - np.cross(orientation_d[1:], orientation_current[1:])) # orientation
        error_p = np.array(position_d - position_current) # position

        if np.linalg.norm(error_o) > orientation_thresh or np.linalg.norm(error_p) > position_thresh:
            ## delta_p
            delta_p[:3] = Ko*error_o
            delta_p[3:] = Kp*error_p

            ## delta_q
            # delta_q = np.dot(np.linalg.inv(J.T.dot(J)), J.T.dot(delta_p))
            # delta_q = np.linalg.solve(J.T.dot(J), J.T.dot(delta_p)) 
            delta_q = np.linalg.pinv(J) @ delta_p

            delta_q = self.apply_joint_limits(q, delta_q, self.joint_limits)

            q_d = np.round((q + alpha*delta_q), 4).tolist()
            
            ## print
            print("joint old   ", np.round(q, 4))
            print("pose old    ", pose_current)
            print("pose target ", position_d, orientation_d)
            print("velocity    ", np.round(delta_q, 4).tolist())
            print("joint new   ", np.round(self.get_joint_variable(), 4))
            print("pose new    ", self.forward_kinematics(rep = 'quaternion'))

            self.forward_kinematics(q = q_d)
            return q_d

    def apply_joint_limits(self, q, dq, joint_limits):
        # apply joint constraints to the desired joint velocities
        q_new = q + dq
        
        # check if any joint exceeds the joint limits
        for i in range(len(q_new)):
            if q_new[i] < joint_limits[i][0]:
                q_new[i] = joint_limits[i][0]
            elif q_new[i] > joint_limits[i][1]:
                q_new[i] = joint_limits[i][1]
        
        # calculate the adjusted joint velocities
        dq_new = q_new - q
        return dq_new
    