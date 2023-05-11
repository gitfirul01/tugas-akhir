from function import *

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class Manipulator:
    def __init__(self):
        ## joint angle (x_i to xi over z_i)
        self.theta1 = 90#var
        self.theta2 = 0#var
        self.theta3 = 180
        self.theta4 = -180#var
        self.theta5 = -90#var
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
        self.T01 = Rz(0)      @ Tz(0)        @ Tx(0)        @ Rx(self.alpha1)   @ Ry(90)  @ Ty(100)
        T12 = Rz(self.theta1) @ Tz(self.d1)  @ Tx(self.a1)  @ Rx(0)             @ Ry(-75) @ Ty(0)
        T23 = Rz(self.theta2) @ Tz(0)        @ Tx(self.a2)  @ Rx(0)             @ Ry(128) @ Ty(0) @ Rz(self.theta3)
        T34 = Rz(0)           @ Tz(-self.d3) @ Tx(0)        @ Rx(0)             @ Ry(0)   @ Ty(0)
        T45 = Rz(self.theta4) @ Tz(-self.d4) @ Tx(0)        @ Rx(self.alpha5)   @ Ry(0)   @ Ty(0)
        T56 = Rz(self.theta5) @ Tz(0)        @ Tx(self.a5)  @ Rx(self.alpha6)   @ Ry(0)   @ Ty(0)
        T6E = Rz(self.theta6) @ Tz(0)        @ Tx(self.a6)  @ Rx(0)             @ Ry(0)   @ Ty(0)
        ## calculate the transformation matrix for the end effector
        self.T02 = self.T01 @ T12
        self.T03 = self.T02 @ T23
        self.T04 = self.T03 @ T34
        self.T05 = self.T04 @ T45
        self.T06 = self.T05 @ T56
        self.T0E = self.T06 @ T6E

    def update_joint_variable(self, q):
        '''
        Set the robot to joint configuration (q).
        Then calculate & update the transformation matrix.
        '''
        self.theta1 = q[0] % 360
        self.theta2 = q[1] % 360
        self.d3 = q[2]
        self.theta4 = q[3] % 360
        self.theta5 = q[4] % 360
        self.theta6 = q[5] % 360
        
        self.update_tf_matrix()

    def update_plot(self, fig, ax):
        ## extract the position of the end effector
        x = self.T0E[0,3]
        y = self.T0E[1,3]
        z = self.T0E[2,3]

        ## clear axes plot
        ax.cla()
        ## redraw plot
        ax.plot([0, self.T01[0,3]],               [0, self.T01[1,3]],               [0, self.T01[2,3]],               'r')
        ax.plot([self.T01[0,3], self.T02[0,3]],   [self.T01[1,3], self.T02[1,3]],   [self.T01[2,3], self.T02[2,3]],   'g')
        ax.plot([self.T02[0,3], self.T03[0,3]],   [self.T02[1,3], self.T03[1,3]],   [self.T02[2,3], self.T03[2,3]],   'b')
        ax.plot([self.T03[0,3], self.T04[0,3]],   [self.T03[1,3], self.T04[1,3]],   [self.T03[2,3], self.T04[2,3]],   'r')
        ax.plot([self.T04[0,3], self.T05[0,3]],   [self.T04[1,3], self.T05[1,3]],   [self.T04[2,3], self.T05[2,3]],   'g')
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
        plot_frame_coordinate(self.T0E, 25)

        ## set figure configuration
        # ax.set_title('3D Simulation')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_xlim(0, 600)
        ax.set_ylim(-300, 300)
        ax.set_zlim(0, 600)
        
        ## print the position of the end effector
        # print("Joint: ", self.get_joint_variable())
        # print("Pose: ", self.forward_kinematics())
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
        z0 = np.array([0, 0, 1])
        z1 = self.T01[:3, 2]
        z2 = np.array([0, 0, 0])
        z3 = self.T03[:3, 2]
        z4 = self.T04[:3, 2]
        z5 = self.T05[:3, 2]

        # Compute the position of the end effector in the base frame
        p = self.T0E[:3, 3]

        # Compute the Jacobian matrix using the formula:
        # Jv = [z0 x (p - p0), z1 x (p - p1), z2 x (p - p2), z3 x (p - p3), z4 x (p - p4), z5 x (p - p5)]
        # Jw = [z0, z1, z2, z3, z4, z5]
        Jv = np.zeros((3, 6))
        Jw = np.zeros((3, 6))

        Jv[:, 0] = np.cross(z0, p)
        Jv[:, 1] = np.cross(z1, p - self.T01[:3, 3])
        Jv[:, 2] = np.cross(z2, p - self.T02[:3, 3])
        Jv[:, 3] = np.cross(z3, p - self.T03[:3, 3])
        Jv[:, 4] = np.cross(z4, p - self.T04[:3, 3])
        Jv[:, 5] = np.cross(z5, p - self.T05[:3, 3])

        Jw[:, 0] = z0
        Jw[:, 1] = z1
        Jw[:, 2] = z2
        Jw[:, 3] = z3
        Jw[:, 4] = z4
        Jw[:, 5] = z5

        return np.round(np.vstack((Jv, Jw)), 2)

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

    def forward_kinematics(self, arg = 'euler'):
        '''
        @ Return [list] of robot pose (x, y, z, yaw, pitch, roll)
        '''
        ## extract the position of the end effector
        x = np.round(self.T0E[0,3], 4)
        y = np.round(self.T0E[1,3], 4)
        z = np.round(self.T0E[2,3], 4)
        if arg == 'euler':
            yaw, pitch, roll = get_euler_angle(self.T0E)
            return [x, y, z], [yaw, pitch, roll]
        elif arg == 'quaternion':
            w, i, j, k = np.round(get_quaternion(self.T0E), 4)
            return [x, y, z], [w, i, j, k]

    def inverse_kinematics(self, p_d):
        ## read joint variable: q
        q = [self.theta1, self.theta2, self.d3, self.theta4, self.theta5, self.theta6]
        print("joint old: ", q)
        
        ## calculate Jacobian matrix with joint variable: J(q)
        # J = np.array(get_jacobian(q)).astype(np.float64)
        J = self.get_jacobian()
        
        ## get actual pose of end-effector: p_a
        p_a = self.forward_kinematics()
        print("pose old: ", p_a)
        print("pose target: ", p_d)

        ## calculate desired velocity: v_d
        distPerUpdate = 0.05 * 345
        targetVector = np.array(p_d) - np.array(p_a)
        targetUnitVector = targetVector / np.linalg.norm(targetVector)
        v_d = (distPerUpdate) * targetUnitVector

        '''
        v_d = ( 0.05 * 345 ) * (np.array(p_d) - np.array(p_a)) / np.linalg.norm(np.array(p_d) - np.array(p_a))
        '''

        # v_d = np.array(p_d) - np.array(p_a)
        print("desired velocity: ", np.round(v_d,4).tolist())

        ## calculate Jacobian inverse to obtain delta joint variable value: delta_q
        J_inv = np.linalg.pinv(J)
        delta_q = J_inv @ v_d
        
        ## calculate new value of joint variable: q_new
        q_new = np.round((q + delta_q), 4).tolist()
        self.update_joint_variable(q_new)

        print("joint new: ", self.get_joint_variable())
        print("pose new: ", self.forward_kinematics())
        return q_new
