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
        self.T01 = None
        self.T02 = None
        self.T03 = None
        self.T04 = None
        self.T05 = None
        self.T06 = None
        self.T0E = None
        self.update_tf_matrix()

    def update_tf_matrix(self):
        ## homogeneous transformation matrix from frame 0 to frame 1
        self.T01 = Rz(90)        @ Tz(100)      @ Ty(0)        @ Ry(0)     @ Rx(90)     @ Tx(0)
        T12 = Rz(self.theta1)    @ Tz(180)      @ Ty(240)      @ Ry(0)     @ Rx(75)     @ Tx(0)  
        T23 = Rz(self.theta2)    @ Tz(0)        @ Ty(300)      @ Ry(0)     @ Rx(-128)   @ Tx(0)
        T34 = Rz(0)              @ Tz(-self.d3) @ Ty(0)        @ Ry(0)     @ Rx(0)      @ Tx(0)      
        T45 = Rz(self.theta4)    @ Tz(-self.d4) @ Ty(0)        @ Ry(0)     @ Rx(-90)    @ Tx(0)        
        T56 = Rz(self.theta5+90) @ Tz(0)        @ Ty(0)        @ Ry(0)     @ Rx(90)     @ Tx(20)  
        T6E = Rz(self.theta6)    @ Tz(0)        @ Ty(0)        @ Ry(0)     @ Rx(0)      @ Tx(30)

        # self.T01 = Rz(0)      @ Tz(0)        @ Tx(0)        @ Rx(self.alpha1)   @ Ry(90)  @ Ty(100)
        # T12 = Rz(self.theta1) @ Tz(self.d1)  @ Tx(self.a1)  @ Rx(0)             @ Ry(-75) @ Ty(0)
        # T23 = Rz(self.theta2) @ Tz(0)        @ Tx(self.a2)  @ Rx(0)             @ Ry(128) @ Ty(0) @ Rz(self.theta3)
        # T34 = Rz(0)           @ Tz(-self.d3) @ Tx(0)        @ Rx(0)             @ Ry(0)   @ Ty(0)
        # T45 = Rz(self.theta4) @ Tz(-self.d4) @ Tx(0)        @ Rx(self.alpha5)   @ Ry(0)   @ Ty(0)
        # T56 = Rz(self.theta5) @ Tz(0)        @ Tx(self.a5)  @ Rx(self.alpha6)   @ Ry(0)   @ Ty(0)
        # T6E = Rz(self.theta6) @ Tz(0)        @ Tx(self.a6)  @ Rx(0)             @ Ry(0)   @ Ty(0)
        ## calculate the transformation matrix for the end effector
        self.T02 = self.T01 @ T12
        self.T03 = self.T02 @ T23
        self.T04 = self.T03 @ T34
        self.T05 = self.T04 @ T45
        self.T06 = self.T05 @ T56
        self.T0E = self.T06 @ T6E

    def update_plot(self, fig, ax):
        ## extract the position of the end effector
        x = self.T0E[0,3]
        y = self.T0E[1,3]
        z = self.T0E[2,3]

        ## clear axes plot
        ax.cla()
        ## redraw plot
        ax.plot([0, self.T01[0,3]],               [0, self.T01[1,3]],               [0, self.T01[2,3]],               'b')
        ax.plot([self.T01[0,3], self.T02[0,3]],   [self.T01[1,3], self.T02[1,3]],   [self.T01[2,3], self.T02[2,3]],   'r')
        ax.plot([self.T02[0,3], self.T03[0,3]],   [self.T02[1,3], self.T03[1,3]],   [self.T02[2,3], self.T03[2,3]],   'g')
        ax.plot([self.T03[0,3], self.T04[0,3]],   [self.T03[1,3], self.T04[1,3]],   [self.T03[2,3], self.T04[2,3]],   'b')
        ax.plot([self.T04[0,3], self.T05[0,3]],   [self.T04[1,3], self.T05[1,3]],   [self.T04[2,3], self.T05[2,3]],   'r')
        ax.plot([self.T05[0,3], self.T06[0,3]],   [self.T05[1,3], self.T06[1,3]],   [self.T05[2,3], self.T06[2,3]],   'g')
        ax.plot([self.T06[0,3], x],               [self.T06[1,3], y],               [self.T06[2,3], z],               'b')
        ax.scatter(x, y, z)

        def plot_frame_coordinate(T, length):
                # add relative coordinate frames for each link
                O = np.array([[0, 0, 0]]).T
                X = T[:3, :3] @ np.array([[1, 0, 0]]).T
                Y = T[:3, :3] @ np.array([[0, 1, 0]]).T
                Z = T[:3, :3] @ np.array([[0, 0, 1]]).T

                ax.quiver(T[0, 3], T[1, 3], T[2, 3], X[0, 0], X[1, 0], X[2, 0], color='r', length=length)
                ax.quiver(T[0, 3], T[1, 3], T[2, 3], Y[0, 0], Y[1, 0], Y[2, 0], color='g', length=length)
                ax.quiver(T[0, 3], T[1, 3], T[2, 3], Z[0, 0], Z[1, 0], Z[2, 0], color='b', length=length) 

        plot_frame_coordinate(self.T01, 25)
        plot_frame_coordinate(self.T02, 25)
        plot_frame_coordinate(self.T03, 25)
        plot_frame_coordinate(self.T04, 25)
        plot_frame_coordinate(self.T05, 25)
        plot_frame_coordinate(self.T06, 25)
        plot_frame_coordinate(self.T0E, 50)

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
        # print(get_euler_convention(self.T0E))

        ## show the plot
        fig.canvas.draw()
        fig.canvas.flush_events()

    def get_joint_variable(self, arg = 'degree'):
        if arg == 'degree':
            return [self.theta1, self.theta2, self.d3, self.theta4, self.theta5, self.theta6]
        elif arg == 'radian':
            return [np.deg2rad(self.theta1), np.deg2rad(self.theta2), self.d3, np.deg2rad(self.theta4), np.deg2rad(self.theta5), np.deg2rad(self.theta6)]

    def get_jacobian(self):
        '''
        @ Return [numpy.array] of Jacobian matrix
        '''
        # Define the unit vectors along the axes of rotation of each joint
        z = [0,0,0,0,0,0]
        z[0] = np.array([0, 0, 1])
        z[1] = self.T01[:3, 2]
        z[2] = np.array([0, 0, 0])
        z[3] = self.T03[:3, 2]
        z[4] = self.T04[:3, 2]
        z[5] = self.T05[:3, 2]
        # Compute the position of the end effector in the base frame
        p = self.T0E[:3, 3]
        # Compute the Jacobian matrix using the formula:
        Jp = np.zeros((3, 6))
        Jo = np.zeros((3, 6))

        Jp[:, 0] = np.cross(z[0], p)
        Jp[:, 1] = np.cross(z[1], p - self.T01[:3, 3])
        Jp[:, 2] = np.cross(z[2], p - self.T02[:3, 3])
        # Jp[:, 3] = np.cross(z[3], p - self.T03[:3, 3])
        # Jp[:, 4] = np.cross(z[4], p - self.T04[:3, 3])
        # Jp[:, 5] = np.cross(z[5], p - self.T05[:3, 3])

        Jo[:, 0] = z[0]
        Jo[:, 1] = z[1]
        Jo[:, 2] = z[2]
        Jo[:, 3] = z[3]
        Jo[:, 4] = z[4]
        Jo[:, 5] = z[5]

        return np.round(np.vstack((Jo, Jp)), 2)

    def get_position_jacobian(self):
        '''
        @ Return [numpy.array] of Jacobian matrix
        '''
        # Define the unit vectors along the axes of rotation of each joint
        z = [0,0,0,0,0,0]
        z[0] = np.array([0, 0, 1])
        z[1] = self.T01[:3, 2]
        z[2] = np.array([0, 0, 0])
        z[3] = self.T03[:3, 2]
        z[4] = self.T04[:3, 2]
        z[5] = self.T05[:3, 2]
        # Compute the position of the end effector in the base frame
        p = self.T0E[:3, 3]
        # Compute the Jacobian matrix using the formula:
        Jp = np.zeros((3, 6))

        Jp[:, 0] = np.cross(z[0], p)
        Jp[:, 1] = np.cross(z[1], p - self.T01[:3, 3])
        Jp[:, 2] = np.cross(z[2], p - self.T02[:3, 3])
        # Jp[:, 3] = np.cross(z[3], p - self.T03[:3, 3])
        # Jp[:, 4] = np.cross(z[4], p - self.T04[:3, 3])
        # Jp[:, 5] = np.cross(z[5], p - self.T05[:3, 3])

        return Jp

    def get_orientation_jacobian(self):
        '''
        @ Return [numpy.array] of Jacobian matrix
        '''
        # Define the unit vectors along the axes of rotation of each joint
        z = [0,0,0,0,0,0]
        z[0] = np.array([0, 0, 1])
        z[1] = self.T01[:3, 2]
        z[2] = np.array([0, 0, 0])
        z[3] = self.T03[:3, 2]
        z[4] = self.T04[:3, 2]
        z[5] = self.T05[:3, 2]

        Jo = np.zeros((3, 6))

        Jo[:, 0] = z[0] # joint1
        Jo[:, 1] = z[1] # joint2
        Jo[:, 2] = z[2] # joint3
        Jo[:, 3] = z[3] # joint4
        Jo[:, 4] = z[4] # joint5
        Jo[:, 5] = z[5] # joint6

        return Jo

    def get_jacobian_form(self):
        theta1, theta2, d3, theta4, theta5, theta6 = sp.symbols('theta1 theta2 d3 theta4 theta5 theta6')
        ## homogeneous transformation matrix from frame 0 to frame 1
        T01 = Rz(0)             * Tz(0)         * Tx(0)         * Rx(self.alpha1)   * Ry(90)    * Ty(100)
        T12 = Rz(theta1, True)  * Tz(self.d1)   * Tx(self.a1)   * Rx(0)             * Ry(-75)   * Ty(0)
        T23 = Rz(theta2, True)  * Tz(0)         * Tx(self.a2)   * Rx(0)             * Ry(128)   * Ty(0)     * Rz(self.theta3)
        T34 = Rz(0)             * Tz(-d3, True) * Tx(0)         * Rx(0)             * Ry(0)     * Ty(0)
        T45 = Rz(theta4, True)  * Tz(-self.d4)  * Tx(0)         * Rx(self.alpha5)   * Ry(0)     * Ty(0)
        T56 = Rz(theta5, True)  * Tz(0)         * Tx(self.a5)   * Rx(self.alpha6)   * Ry(0)     * Ty(0)
        T6E = Rz(theta6, True)  * Tz(0)         * Tx(self.a6)   * Rx(0)             * Ry(0)     * Ty(0)
        ## calculate the transformation matrix for the end effector
        T02 = T01 * T12
        T03 = T02 * T23
        T04 = T03 * T34
        T05 = T04 * T45
        T06 = T05 * T56
        T0E = T06 * T6E
        ## quaternion
        w = sp.sqrt(1 + T0E[0,0] + T0E[1,1] + T0E[2,2]) /2;
        i = (T0E[2,1] - T0E[1,2]) / (4*w);
        j = (T0E[0,2] - T0E[2,0]) / (4*w);
        k = (T0E[1,0] - T0E[0,1]) / (4*w);
        ## position
        x = T0E[0,3];
        y = T0E[1,3];
        z = T0E[2,3];
        ## jacobian matrix
        J = sp.Matrix([
            [sp.diff(x, theta1), sp.diff(x, theta2), sp.diff(x, d3), sp.diff(x, theta4), sp.diff(x, theta5), sp.diff(x, theta6)],
            [sp.diff(y, theta1), sp.diff(y, theta2), sp.diff(y, d3), sp.diff(y, theta4), sp.diff(y, theta5), sp.diff(y, theta6)],
            [sp.diff(z, theta1), sp.diff(z, theta2), sp.diff(z, d3), sp.diff(z, theta4), sp.diff(z, theta5), sp.diff(z, theta6)],
            [sp.diff(w, theta1), sp.diff(w, theta2), sp.diff(w, d3), sp.diff(w, theta4), sp.diff(w, theta5), sp.diff(w, theta6)],
            [sp.diff(i, theta1), sp.diff(i, theta2), sp.diff(i, d3), sp.diff(i, theta4), sp.diff(i, theta5), sp.diff(i, theta6)],
            [sp.diff(j, theta1), sp.diff(j, theta2), sp.diff(j, d3), sp.diff(j, theta4), sp.diff(j, theta5), sp.diff(j, theta6)],
            [sp.diff(k, theta1), sp.diff(k, theta2), sp.diff(k, d3), sp.diff(k, theta4), sp.diff(k, theta5), sp.diff(k, theta6)]
            ]).evalf(4)

        return J

    def forward_kinematics(self, q = None, rep = 'euler', unit = 'degree'):
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
            self.update_tf_matrix()

        ## extract the position of the end effector
        x = np.round(self.T0E[0,3], 4)
        y = np.round(self.T0E[1,3], 4)
        z = np.round(self.T0E[2,3], 4)
        if rep == 'euler':
            roll, pitch, yaw = np.round(get_euler_angle(self.T0E), 4)
            return [x, y, z, roll, pitch, yaw]
        elif rep == 'quaternion':
            w, i, j, k = np.round(get_quaternion(self.T0E), 4)
            return [x, y, z, w, i, j, k]

    def differential_inverse_kinematics(self, position_d, orientation_d):
        position_thresh = 1e-4
        orientation_thresh = 1e-4
        Ko = 3
        Kp = 8
        alpha = 2

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
        error_p = np.array(position_d - np.array(pose_current[:3])) # position

        if np.linalg.norm(error_o) > orientation_thresh or np.linalg.norm(error_p) > position_thresh:
            ## delta_p
            delta_p[:3] = Ko*error_o
            delta_p[3:] = Kp*error_p

            ## delta_q
            # delta_q = np.dot(np.linalg.inv(J.T.dot(J)), J.T.dot(delta_p))
            # delta_q = np.linalg.solve(J.T.dot(J), J.T.dot(delta_p)) 
            delta_q = np.linalg.pinv(J) @ delta_p

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
