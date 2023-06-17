'''
    ROBOT MANIPULATOR USING MODIFIED DH-PARAMETER
'''
from function import *

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class Manipulator:
    def __init__(self):
        ## theta
        self.theta1 = 90
        self.theta2 = 0#var1
        self.theta3 = 90
        self.theta4 = 0#var2
        self.theta5 = 90
        self.theta6 = 0
        self.theta7 = 0
        self.theta8 = 90#var4
        self.theta9 = -90#var5
        self.theta10 = 0#var6
        self.theta11 = 0
        ## d
        self.d1 = 148
        self.d2 = 0
        self.d3 = 179.33
        self.d4 = 0
        self.d5 = 0
        self.d6 = 14.37#var3
        self.d7 = 122.15
        self.d8 = 485#-485
        self.d9 = 0
        self.d10 = 0
        self.d11 = 0
        ## a-1
        self.a1 = 0
        self.a2 = 0
        self.a3 = 240.21
        self.a4 = 38.5
        self.a5 = 150.32
        self.a6 = 0
        self.a7 = 0
        self.a8 = 0
        self.a9 = 0
        self.a10 = 10
        self.a11 = 7
        ## alpha-1
        self.alpha1 = 0
        self.alpha2 = 90
        self.alpha3 = 0
        self.alpha4 = 105
        self.alpha5 = 0
        self.alpha6 = 52
        self.alpha7 = 90
        self.alpha8 = 90#-90
        self.alpha9 = 90
        self.alpha10 = 90
        self.alpha11 = 0
        ## Transformation matrix
        self.T0_1 = None
        self.T0_2 = None
        self.T0_3 = None
        self.T0_4 = None
        self.T0_5 = None
        self.T0_6 = None
        self.T0_7 = None
        self.T0_8 = None
        self.T0_9 = None
        self.T0_10 = None
        self.T0_11 = None
        self.update_tf_matrix()
        ## define joint limits
        self.joint_limits = [
            # [min, max]
            [0, 90],
            [-55, 55],
            [14.37, 188.16],
            [-45, 225],
            [0, 180],
            [-90, 90]
        ]
        ## trace variable
        self.pos = [[],[],[]]

    def update_tf_matrix(self):
        ## homogeneous transformation matrix from frame 0 to frame 1
        self.T0_1 = mdh2tf(self.alpha1, self.a1, self.d1, self.theta1)
        T1_2 = mdh2tf(self.alpha2, self.a2, self.d2, self.theta2)
        T2_3 = mdh2tf(self.alpha3, self.a3, self.d3, self.theta3)
        T3_4 = mdh2tf(self.alpha4, self.a4, self.d4, self.theta4)
        T4_5 = mdh2tf(self.alpha5, self.a5, self.d5, self.theta5)
        T5_6 = mdh2tf(self.alpha6, self.a6, self.d6, self.theta6)
        T6_7 = mdh2tf(self.alpha7, self.a7, self.d7, self.theta7)
        T7_8 = mdh2tf(self.alpha8, self.a8, self.d8, self.theta8)
        T8_9 = mdh2tf(self.alpha9, self.a9, self.d9, self.theta9)
        T9_10 = mdh2tf(self.alpha10, self.a10, self.d10, self.theta10)
        T10_11 = mdh2tf(self.alpha11, self.a11, self.d11, self.theta11)

        ## calculate the transformation matrix for the end effector
        self.T0_2 = self.T0_1 @ T1_2
        self.T0_3 = self.T0_2 @ T2_3
        self.T0_4 = self.T0_3 @ T3_4
        self.T0_5 = self.T0_4 @ T4_5
        self.T0_6 = self.T0_5 @ T5_6
        self.T0_7 = self.T0_6 @ T6_7
        self.T0_8 = self.T0_7 @ T7_8
        self.T0_9 = self.T0_8 @ T8_9
        self.T0_10 = self.T0_9 @ T9_10
        self.T0_11 = self.T0_10 @ T10_11

        return self.T0_11

    def update_plot(self, fig, ax, trace='off'):
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
        x = self.T0_11[0,3]
        y = self.T0_11[1,3]
        z = self.T0_11[2,3]
        ## clear axes plot
        ax.cla()
        ## redraw plot
        plot_link(np.zeros(([4,4])), self.T0_1, 'b')
        plot_link(self.T0_1, self.T0_2)
        plot_link(self.T0_2, self.T0_3, 'g')
        plot_link(self.T0_3, self.T0_4, 'r')
        plot_link(self.T0_4, self.T0_5, 'r')
        plot_link(self.T0_5, self.T0_6, 'b')
        plot_link(self.T0_6, self.T0_7, 'b')
        plot_link(self.T0_7, self.T0_8, 'b')
        plot_link(self.T0_8, self.T0_9, 'g')
        plot_link(self.T0_9, self.T0_10, 'r')
        plot_link(self.T0_10, self.T0_11, 'r')

        ## joint point plot
        # plot_frame_scatter(self.T0_1)
        plot_frame_scatter(self.T0_2)
        # plot_frame_scatter(self.T0_3)
        plot_frame_scatter(self.T0_4)
        # plot_frame_scatter(self.T0_5)
        plot_frame_scatter(self.T0_6)
        # plot_frame_scatter(self.T0_7)
        # plot_frame_scatter(self.T0_8)
        # plot_frame_scatter(self.T0_9)
        # plot_frame_scatter(self.T0_10)
        plot_frame_scatter(self.T0_11)
        
        ## frame plot
        plot_frame_coordinate(self.T0_1)
        # plot_frame_coordinate(self.T0_2)
        # plot_frame_coordinate(self.T0_3)
        plot_frame_coordinate(self.T0_4)
        # plot_frame_coordinate(self.T0_5)
        plot_frame_coordinate(self.T0_6)
        # plot_frame_coordinate(self.T0_7)
        # plot_frame_coordinate(self.T0_8)
        # plot_frame_coordinate(self.T0_9)
        # plot_frame_coordinate(self.T0_10)
        plot_frame_coordinate(self.T0_11)
        ## trace plot
        if trace == 'on':
            self.pos[0].append(x)
            self.pos[1].append(y)
            self.pos[2].append(z)
            ax.plot(self.pos[0], self.pos[1], self.pos[2], 'c-')
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
        # print(get_euler_convention(self.T0_11))

        ## show the plot
        fig.canvas.draw()
        fig.canvas.flush_events()

    def plot_workspace(self, ax, res=5):
        ## linear space joint limits and convert to radian
        q1 = np.linspace(self.joint_limits[0][0], self.joint_limits[0][1], res)
        q2 = np.linspace(self.joint_limits[1][0], self.joint_limits[1][1], res)
        q3 = np.linspace(self.joint_limits[2][0], self.joint_limits[2][1], res)
        q4 = np.linspace(self.joint_limits[3][0], self.joint_limits[3][1], res)
        q5 = np.linspace(self.joint_limits[4][0], self.joint_limits[4][1], res)
        q6 = np.linspace(self.joint_limits[5][0], self.joint_limits[5][1], res)

        x = []
        y = []
        z = []
        for i in range(len(q1)):
            for j in range(len(q2)):
                for k in range(len(q3)):
                    for l in range(len(q4)):
                        for m in range(len(q5)):
                            for n in range(len(q6)):
                                T = mdh2tf(self.alpha1, self.a1, self.d1, self.theta1) \
                                    @ mdh2tf(self.alpha2, self.a2, self.d2, q1[i]) \
                                    @ mdh2tf(self.alpha3, self.a3, self.d3, self.theta3) \
                                    @ mdh2tf(self.alpha4, self.a4, self.d4, q2[j]) \
                                    @ mdh2tf(self.alpha5, self.a5, self.d5, self.theta5) \
                                    @ mdh2tf(self.alpha6, self.a6, q3[k], self.theta6) \
                                    @ mdh2tf(self.alpha7, self.a7, self.d7, self.theta7) \
                                    @ mdh2tf(self.alpha8, self.a8, self.d8, q4[l]) \
                                    @ mdh2tf(self.alpha9, self.a9, self.d9, q5[m]) \
                                    @ mdh2tf(self.alpha10, self.a10, self.d10, q6[n]) \
                                    @ mdh2tf(self.alpha11, self.a11, self.d11, self.theta11)
                                x.append(T[0, 3])
                                y.append(T[1, 3])
                                z.append(T[2, 3])
        '''
        ax.scatter()
            c: color
                c=z: set the color map based on the z-axis values
                c=y: set the color map based on the y-axis values
                c=x: set the color map based on the x-axis values
                c='r': red
                etc
            cmap: color map 
            s: size
        '''
        ax.scatter(x, y, z, c=z, cmap='viridis', s=5)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

    def get_joint_variable(self, arg = 'degree'):
        if arg == 'degree':
            return [self.theta2, self.theta4, self.d6, self.theta8, self.theta9, self.theta10]
        elif arg == 'radian':
            return [np.deg2rad(self.theta2), np.deg2rad(self.theta4), self.d6, np.deg2rad(self.theta8), np.deg2rad(self.theta9), np.deg2rad(self.theta10)]

    def get_position_jacobian(self):
        '''
        @ Return [numpy.array] of Jacobian matrix
        '''
        Jp = np.zeros((3, 6))
        ## Compute the position of the end effector in the base frame
        p = self.T0_11[:3, 3]
        ## Define the unit vectors along the axes of rotation of each joint
        A = [0,0,0,0,0,0,0,0,0,0,0]
        A[0] = self.T0_1[:3, 2]
        A[1] = self.T0_2[:3, 2]
        A[2] = self.T0_3[:3, 2]
        A[3] = self.T0_4[:3, 2]
        A[4] = self.T0_5[:3, 2]
        A[5] = self.T0_6[:3, 2]
        A[6] = self.T0_7[:3, 2]
        A[7] = self.T0_8[:3, 2]
        A[8] = self.T0_9[:3, 2]
        A[9] = self.T0_10[:3, 2]
        ## Compute the Jacobian matrix using the formula:
        Jp[:, 0] = np.cross(A[1], p - self.T0_2[:3, 3]) # revolute
        Jp[:, 1] = np.cross(A[3], p - self.T0_4[:3, 3]) # revolute
        Jp[:, 2] = A[5]                                 # prismatic
        Jp[:, 3] = np.cross(A[7], p - self.T0_8[:3, 3]) # revolute
        Jp[:, 4] = np.cross(A[8], p - self.T0_9[:3, 3]) # revolute
        Jp[:, 5] = np.cross(A[9], p - self.T0_10[:3, 3]) # revolute

        return Jp

    def get_orientation_jacobian(self):
        '''
        @ Return [numpy.array] of Jacobian matrix
        '''
        Jo = np.zeros((3, 6))
        # Define the unit vectors along the axes of rotation of each joint
        A = [0,0,0,0,0,0,0,0,0,0]
        A[0] = self.T0_1[:3, 2]
        A[1] = self.T0_2[:3, 2] # revolute
        A[2] = self.T0_3[:3, 2]
        A[3] = self.T0_4[:3, 2] # revolute
        A[4] = self.T0_5[:3, 2]
        A[5] = np.array([0, 0, 0]) # prismatic
        A[6] = self.T0_7[:3, 2]
        A[7] = self.T0_8[:3, 2] # revolute
        A[8] = self.T0_9[:3, 2] # revolute
        A[9] = self.T0_10[:3, 2] # revolute

        Jo[:, 0] = A[1] # joint1
        Jo[:, 1] = A[3] # joint2
        Jo[:, 2] = A[5] # joint3
        Jo[:, 3] = A[7] # joint4
        Jo[:, 4] = A[8] # joint5
        Jo[:, 5] = A[9] # joint6

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
                self.theta2 = q[0] % 360
                self.theta4 = q[1] % 360
                self.d6     = q[2]
                self.theta8 = q[3] % 360
                self.theta9 = q[4] % 360
                self.theta10 = q[5] % 360
            elif unit == 'radian':
                self.theta2 = np.rad2deg(q[0]) % 360
                self.theta4 = np.rad2deg(q[1]) % 360
                self.d6     = q[2]
                self.theta8 = np.rad2deg(q[3]) % 360
                self.theta9 = np.rad2deg(q[4]) % 360
                self.theta10 = np.rad2deg(q[5]) % 360
        H = self.update_tf_matrix()
        ## extract the position of the end effector
        x = np.round(self.T0_11[0,3], 4)
        y = np.round(self.T0_11[1,3], 4)
        z = np.round(self.T0_11[2,3], 4)
        if rep == 'euler':
            roll, pitch, yaw = np.round(get_euler_angle(H), 4)
            return [x, y, z, roll, pitch, yaw]
        elif rep == 'quaternion':
            w, i, j, k = np.round(get_quaternion(H), 4)
            return [x, y, z, w, i, j, k]

    def differential_inverse_kinematics(self, position_d, orientation_d):
        position_thresh = 1e-4
        orientation_thresh = 1e-4
        Ko = 15.0
        Kp = 2.7
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
    