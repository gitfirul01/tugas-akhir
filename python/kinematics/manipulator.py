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
        self.calculate_tf_matrix()

    def calculate_tf_matrix(self):
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
        self.theta1 = q[0]
        self.theta2 = q[1]
        self.d3 = q[2]
        self.theta4 = q[3]
        self.theta5 = q[4]
        self.theta6 = q[5]
        
        self.calculate_tf_matrix()
    
    def get_joint_variable(self):
        return [self.theta1, self.theta2, self.theta4, self.theta5, self.theta6, self.d3]

    def forward_kinematics(self):
        '''
        @ Return [list] of robot pose (x, y, z, yaw, pitch, roll)
        '''
        ## extract the position of the end effector
        x = np.round(self.T0E[0,3], 4)
        y = np.round(self.T0E[1,3], 4)
        z = np.round(self.T0E[2,3], 4)
        # w, i, j, k = np.round(get_quaternion(self.T0E), 4)
        yaw, pitch, roll = get_euler_angle(self.T0E)
        # return [x, y, z, w, i, j, k]
        return [x, y, z, yaw, pitch, roll]

    def get_jacobian(self):
        '''
        @ Return [numpy.array] of Jacobian matrix
        '''
        unit_vector = np.array([0,0,1])
        J_v = np.zeros((3, 6))
        J_w = np.zeros((3, 6))

        J_v[:,0] = np.cross(unit_vector, self.T0E[:3,3])
        J_w[:,0] = unit_vector

        J_v[:,1] = np.cross(self.T01[:3,:3]@unit_vector, (self.T0E[:3,3]-self.T01[:3,3]))
        J_w[:,1] = self.T01[:3,:3]@unit_vector

        J_v[:,2] = np.cross(self.T02[:3,:3]@unit_vector, (self.T0E[:3,3]-self.T02[:3,3]))
        J_w[:,2] = np.array([0,0,0])

        J_v[:,3] = np.cross(self.T03[:3,:3]@unit_vector, (self.T0E[:3,3]-self.T03[:3,3]))
        J_w[:,3] = self.T03[:3,:3]@unit_vector

        J_v[:,4] = np.cross(self.T04[:3,:3]@unit_vector, (self.T0E[:3,3]-self.T04[:3,3]))
        J_w[:,4] = self.T04[:3,:3]@unit_vector

        J_v[:,5] = np.cross(self.T05[:3,:3]@unit_vector, (self.T0E[:3,3]-self.T05[:3,3]))
        J_w[:,5] = self.T05[:3,:3]@unit_vector

        return np.round(np.vstack((J_v, J_w)), 2)

    def inverse_kinematics(self, p_d):
        ## read joint variable: q
        q = [self.theta1, self.theta2, self.d3, self.theta4, self.theta5, self.theta6]
        
        ## calculate Jacobian matrix with joint variable: J(q)
        # J = np.array(get_jacobian(q)).astype(np.float64)
        J = self.get_jacobian()
        
        ## get actual pose of end-effector: p_a
        p_a = self.forward_kinematics()
        
        ## calculate desired velocity: v_d
        v_d = np.array(p_d) - np.array(p_a)
        
        ## calculate Jacobian inverse to obtain delta joint variable value: delta_q
        J_inv = np.linalg.pinv(J)
        delta_q = J_inv @ v_d
        
        ## calculate new value of joint variable: q_new
        q_new = np.round((q + delta_q), 4).tolist()

        self.update_joint_variable(q_new)
        return q_new

    def plot(self):
        ## turn on interactive plot
        plt.ion()
        ## create a 3D figure
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        while True:
            self.calculate_tf_matrix()
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

            # add relative coordinate frames for each link
            O = np.array([[0, 0, 0]]).T
            X1 = self.T01[:3, :3] @ np.array([[1, 0, 0]]).T
            Y1 = self.T01[:3, :3] @ np.array([[0, 1, 0]]).T
            Z1 = self.T01[:3, :3] @ np.array([[0, 0, 1]]).T

            X2 = self.T02[:3, :3] @ np.array([[1, 0, 0]]).T
            Y2 = self.T02[:3, :3] @ np.array([[0, 1, 0]]).T
            Z2 = self.T02[:3, :3] @ np.array([[0, 0, 1]]).T

            X3 = self.T03[:3, :3] @ np.array([[1, 0, 0]]).T
            Y3 = self.T03[:3, :3] @ np.array([[0, 1, 0]]).T
            Z3 = self.T03[:3, :3] @ np.array([[0, 0, 1]]).T

            X4 = self.T04[:3, :3] @ np.array([[1, 0, 0]]).T
            Y4 = self.T04[:3, :3] @ np.array([[0, 1, 0]]).T
            Z4 = self.T04[:3, :3] @ np.array([[0, 0, 1]]).T

            X5 = self.T05[:3, :3] @ np.array([[1, 0, 0]]).T
            Y5 = self.T05[:3, :3] @ np.array([[0, 1, 0]]).T
            Z5 = self.T05[:3, :3] @ np.array([[0, 0, 1]]).T

            X6 = self.T06[:3, :3] @ np.array([[1, 0, 0]]).T
            Y6 = self.T06[:3, :3] @ np.array([[0, 1, 0]]).T
            Z6 = self.T06[:3, :3] @ np.array([[0, 0, 1]]).T

            XE = self.T0E[:3, :3] @ np.array([[1, 0, 0]]).T
            YE = self.T0E[:3, :3] @ np.array([[0, 1, 0]]).T
            ZE = self.T0E[:3, :3] @ np.array([[0, 0, 1]]).T

            ax.quiver(self.T01[0, 3], self.T01[1, 3], self.T01[2, 3], X1[0, 0], X1[1, 0], X1[2, 0], color='r', length=25)
            ax.quiver(self.T01[0, 3], self.T01[1, 3], self.T01[2, 3], Y1[0, 0], Y1[1, 0], Y1[2, 0], color='g', length=25)
            ax.quiver(self.T01[0, 3], self.T01[1, 3], self.T01[2, 3], Z1[0, 0], Z1[1, 0], Z1[2, 0], color='b', length=25) 
            
            ax.quiver(self.T02[0, 3], self.T02[1, 3], self.T02[2, 3], X2[0, 0], X2[1, 0], X2[2, 0], color='r', length=25)
            ax.quiver(self.T02[0, 3], self.T02[1, 3], self.T02[2, 3], Y2[0, 0], Y2[1, 0], Y2[2, 0], color='g', length=25)
            ax.quiver(self.T02[0, 3], self.T02[1, 3], self.T02[2, 3], Z2[0, 0], Z2[1, 0], Z2[2, 0], color='b', length=25) 
            
            ax.quiver(self.T03[0, 3], self.T03[1, 3], self.T03[2, 3], X3[0, 0], X3[1, 0], X3[2, 0], color='r', length=25)
            ax.quiver(self.T03[0, 3], self.T03[1, 3], self.T03[2, 3], Y3[0, 0], Y3[1, 0], Y3[2, 0], color='g', length=25)
            ax.quiver(self.T03[0, 3], self.T03[1, 3], self.T03[2, 3], Z3[0, 0], Z3[1, 0], Z3[2, 0], color='b', length=25)

            ax.quiver(self.T04[0, 3], self.T04[1, 3], self.T04[2, 3], X4[0, 0], X4[1, 0], X4[2, 0], color='r', length=25)
            ax.quiver(self.T04[0, 3], self.T04[1, 3], self.T04[2, 3], Y4[0, 0], Y4[1, 0], Y4[2, 0], color='g', length=25)
            ax.quiver(self.T04[0, 3], self.T04[1, 3], self.T04[2, 3], Z4[0, 0], Z4[1, 0], Z4[2, 0], color='b', length=25)

            ax.quiver(self.T05[0, 3], self.T05[1, 3], self.T05[2, 3], X5[0, 0], X5[1, 0], X5[2, 0], color='r', length=25)
            ax.quiver(self.T05[0, 3], self.T05[1, 3], self.T05[2, 3], Y5[0, 0], Y5[1, 0], Y5[2, 0], color='g', length=25)
            ax.quiver(self.T05[0, 3], self.T05[1, 3], self.T05[2, 3], Z5[0, 0], Z5[1, 0], Z5[2, 0], color='b', length=25)

            ax.quiver(self.T06[0, 3], self.T06[1, 3], self.T06[2, 3], X6[0, 0], X6[1, 0], X6[2, 0], color='r', length=25)
            ax.quiver(self.T06[0, 3], self.T06[1, 3], self.T06[2, 3], Y6[0, 0], Y6[1, 0], Y6[2, 0], color='g', length=25)
            ax.quiver(self.T06[0, 3], self.T06[1, 3], self.T06[2, 3], Z6[0, 0], Z6[1, 0], Z6[2, 0], color='b', length=25) 

            ax.quiver(self.T0E[0, 3], self.T0E[1, 3], self.T0E[2, 3], XE[0, 0], XE[1, 0], XE[2, 0], color='r', length=35)
            ax.quiver(self.T0E[0, 3], self.T0E[1, 3], self.T0E[2, 3], YE[0, 0], YE[1, 0], YE[2, 0], color='g', length=35)
            ax.quiver(self.T0E[0, 3], self.T0E[1, 3], self.T0E[2, 3], ZE[0, 0], ZE[1, 0], ZE[2, 0], color='b', length=35) 

            ## set figure configuration
            # ax.set_title('3D Simulation')
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.set_xlim(0, 600)
            ax.set_ylim(-300, 300)
            ax.set_zlim(0, 600)
            
            ## print the position of the end effector
            print("Angle: ({:.2f}, {:.2f}, {:.2f})   EE-pos: ({:.2f}, {:.2f}, {:.2f})".format(self.theta1,self.theta2,self.d3,x,y,z))
            # print(get_euler_convention(self.T0E))
            print(get_euler_angle(self.T0E))
            # print(np.round(get_quaternion(self.T0E), 4))

            ## show the plot
            fig.canvas.draw()
            fig.canvas.flush_events()
            plt.pause(0.001)
