'''
    CONTROLLER::PUMA560
    EULER CONVENTION: ZYX

    HOME POSITION   V-STRAIGHT      H-STRAIGHT      NOMINAL POSE
    | theta1 = 0    | theta1 = 0    | theta1 = 0    | theta1 = 0
    | theta2 = 0    | theta2 = 90   | theta2 = 0    | theta2 = 45
    | theta3 = 0    | theta3 = -90  | theta3 = -90  | theta3 = -180
    | theta4 = 0    | theta4 = 0    | theta4 = 0    | theta4 = 0
    | theta5 = 0    | theta5 = 0    | theta5 = 0    | theta5 = 45
    | theta6 = 0    | theta6 = 0    | theta6 = 0    | theta6 = 0
'''

from function import *

# import serial, struct
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from time import time

class Controller:
    def __init__(self, DH_param = 'mod'):
        self.DH_param = DH_param
        ## joint angle (x_i to xi over z_i)
        self.theta1 = 0#var
        self.theta2 = 45#var
        self.theta3 = -180#var
        self.theta4 = 0#var
        self.theta5 = 135#var
        self.theta6 = 0#var
        ## joint offset (O_i to xi over z_i)
        self.d1 = 85#
        self.d2 = 0
        self.d3 = 0
        self.d4 = 150#
        self.d5 = 0
        self.d6 = 0
        ## link lengths (Oi to z_i over xi)
        self.a1 = 0
        self.a2 = 150#
        self.a3 = 0
        self.a4 = 0
        self.a5 = 0
        self.a6 = 0
        ## link twist (z_i to zi over xi)
        self.alpha1 = 90#
        self.alpha2 = 0
        self.alpha3 = -90#
        self.alpha4 = 90#
        self.alpha5 = -90#
        self.alpha6 = 0
        ## Transformation matrix
        self.T0_1 = None
        self.T0_2 = None
        self.T0_3 = None
        self.T0_4 = None
        self.T0_5 = None
        self.T0_6 = None
        self.T0_E = None
        self.T_PEN = None
        self.update_tf_matrix()
        ## define joint limits
        self.joint_limits = [
            # [min, max]
            [-60,    60],
            [0,      90],
            [125,    215],
            [-179,   150],
            [-90,    90+45],
            [-160,   160]
        ]
        ## trace variable
        self.pos = [[],[],[]]
        ## IK variable
        self.position_thresh = 1e-4
        self.orientation_thresh = 1e-4
        self.Ko = 8
        self.Kp = 3
        self.alpha = 1.5 # update rate
        self.max_speed_pos = 20
        self.max_speed_ori = 25
        ## workspace variable
        self.s = [1.2, 0.65, 0.65] # scale
        self.t = [-140, 280, -22.5] # translate
        # self.s = [1,1,1] # scale
        # self.t = [0,0,0] # translate

    def update_tf_matrix(self):
        ## homogeneous transformation matrix from frame i to frame j (T_ij)
        if self.DH_param == 'mod':
            ##                     alpha-1      a-1        d        theta
            self.T0_1 = mdh2tf(          0,       0, self.d1, self.theta1)
            T1_2      = mdh2tf(self.alpha1, self.a1, self.d2, self.theta2)
            T2_3      = mdh2tf(self.alpha2, self.a2, self.d3, self.theta3)
            T3_4      = mdh2tf(self.alpha3, self.a3, self.d4, self.theta4)
            T4_5      = mdh2tf(self.alpha4, self.a4, self.d5, self.theta5)
            T5_6      = mdh2tf(self.alpha5, self.a5, self.d6, self.theta6)
        elif self.DH_param == 'std':
            ##                      theta        d        a        alpha
            self.T0_1 = dh2tf(self.theta1, self.d1, self.a1, self.alpha1)
            T1_2      = dh2tf(self.theta2, self.d2, self.a2, self.alpha2)
            T2_3      = dh2tf(self.theta3, self.d3, self.a3, self.alpha3)
            T3_4      = dh2tf(self.theta4, self.d4, self.a4, self.alpha4)
            T4_5      = dh2tf(self.theta5, self.d5, self.a5, self.alpha5)
            T5_6      = dh2tf(self.theta6, self.d6, self.a6, self.alpha6)

        ## additional plot for end effector => rotate about y-axis to fit end effector orientation
        T6_E = np.array([[np.cos(np.deg2rad(90)), 0, np.sin(np.deg2rad(90)), 0],
                        [0, 1, 0, 0],
                        [-np.sin(np.deg2rad(90)), 0, np.cos(np.deg2rad(90)), 0],
                        [0, 0, 0, 1]])
        ## calculate the transformation matrix for the end effector
        self.T0_2 = self.T0_1 @ T1_2
        self.T0_3 = self.T0_2 @ T2_3
        self.T0_4 = self.T0_3 @ T3_4
        self.T0_5 = self.T0_4 @ T4_5
        self.T0_6 = self.T0_5 @ T5_6
        ## additional transformation matrix for end-effector and pen
        self.T0_E = self.T0_6 @ T6_E
        self.T_PEN = self.T0_6 @ np.array([[1, 0, 0, 0], 
                                [0, 1, 0, 0], 
                                [0, 0, 1, 75],
                                [0, 0, 0, 1]])
        return self.T0_E
        
    def update_plot(self, ax, trace='off'):
        ## Function
        def plot_frame_coordinate(T, length = 25):
            '''
            Add relative coordinate frames for each link
            Param:
            @T: 4x4 transformation matrix
            @length: coordinate line length
            '''
            u = T[:3, :3] @ np.array([[1, 0, 0]]).T
            v = T[:3, :3] @ np.array([[0, 1, 0]]).T
            w = T[:3, :3] @ np.array([[0, 0, 1]]).T

            x = T[0, 3]
            y = T[1, 3]
            z = T[2, 3]

            ax.quiver(x, y, z, u[0, 0], u[1, 0], u[2, 0], color='r', length=length)
            ax.quiver(x, y, z, v[0, 0], v[1, 0], v[2, 0], color='g', length=length)
            ax.quiver(x, y, z, w[0, 0], w[1, 0], w[2, 0], color='b', length=length) 

        def plot_frame_scatter(T, color = 'b'):
            x = T[0, 3]
            y = T[1, 3]
            z = T[2, 3]
            ax.scatter(x, y, z, color = color)

        def plot_link(T1, T2, color='r'):
            ax.plot([T1[0,3], T2[0,3]],   [T1[1,3], T2[1,3]],   [T1[2,3], T2[2,3]],   color)

        ## extract the position of the end effector
        x = self.T0_E[0,3]
        y = self.T0_E[1,3]
        z = self.T0_E[2,3]
        
        ## clear axes plot
        ax.cla() 
        
        ## link plot
        plot_link(np.zeros(([4,4])), self.T0_1, 'b')
        plot_link(self.T0_1, self.T0_2, 'r')
        plot_link(self.T0_2, self.T0_3, 'r')
        plot_link(self.T0_3, self.T0_4, 'b')
        plot_link(self.T0_4, self.T0_5, 'r')
        plot_link(self.T0_5, self.T0_6, 'r')
        plot_link(self.T0_6, self.T0_E, 'r')
        plot_link(self.T0_E, self.T_PEN, 'b')

        ## joint point plot
        # plot_frame_scatter(self.T0_1)
        # plot_frame_scatter(self.T0_2)
        # plot_frame_scatter(self.T0_3)
        # plot_frame_scatter(self.T0_4)
        # plot_frame_scatter(self.T0_5)
        # plot_frame_scatter(self.T0_6)
        plot_frame_scatter(self.T0_E)

        ## frame plot
        # plot_frame_coordinate(self.T0_1)
        # plot_frame_coordinate(self.T0_2)
        # plot_frame_coordinate(self.T0_3)
        # plot_frame_coordinate(self.T0_4)
        # plot_frame_coordinate(self.T0_5)
        # plot_frame_coordinate(self.T0_6)
        plot_frame_coordinate(self.T0_E)

        ## trace plot
        if trace == 'on':
            self.pos[0].append(x)
            self.pos[1].append(y)
            self.pos[2].append(z)
            ax.plot(self.pos[0], self.pos[1], self.pos[2], 'c-')
        ## set figure configuration
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_xlim(-100, 300)
        ax.set_ylim(-300, 300)
        ax.set_zlim(0, 400)
        ## set panes color
        ax.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        ax.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        ax.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))

        ## print the position of the end effector
        # print("Joint ", self.get_joint_variable())
        # print("Pose  ", self.forward_kinematics(rep = 'quaternion'))
        # print(get_euler_convention(self.T0_E))
        
        ## show the plot
        # fig.canvas.draw()
        # fig.canvas.flush_events()

    def plot_workspace_point(self, ax=None, res=5, size=5, color=None, axes='xyz', alpha=1):
        ## linear space joint limits and convert to radian
        q1 = np.linspace(self.joint_limits[0][0], self.joint_limits[0][1], res)
        q2 = np.linspace(self.joint_limits[1][0], self.joint_limits[1][1], res)
        q3 = np.linspace(self.joint_limits[2][0], self.joint_limits[2][1], res)
        
        x = []
        y = []
        z = []
        for i in range(len(q1)):
            for j in range(len(q2)):
                for k in range(len(q3)):
                    T = mdh2tf(0, 0, self.d1, np.rad2deg(np.deg2rad(q1)[i])) \
                        @ mdh2tf(self.alpha1, self.a1, self.d2, np.rad2deg(np.deg2rad(q2)[j])) \
                        @ mdh2tf(self.alpha2, self.a2, self.d3, np.rad2deg(np.deg2rad(q3)[k])) \
                        @ mdh2tf(self.alpha3, self.a3, self.d4, 0)
                    x.append(T[0, 3])
                    y.append(T[1, 3])
                    z.append(T[2, 3])
        # rescale the workspace
        x = ( self.s[0]*np.array(x) + self.t[0] ).tolist()
        y = ( self.s[1]*np.array(y) + self.t[1] ).tolist()
        z = ( self.s[2]*np.array(z) + self.t[2] ).tolist()

        if ax == None: fig, ax_c = plt.subplots(subplot_kw={'projection': '3d'})
        else: ax_c = ax

        if color==None: ax_c.scatter(x, y, z, c=z, cmap='viridis', s=5, alpha=alpha)
        else: ax_c.scatter(x, y, z, c=color, s=5, alpha=alpha)

        # (plane, (elev, azim, roll))
        views = [(90, -90, 0),  # XY
                 (0, -90, 0),   # XZ
                 (0,   0, 0),   # YZ
                 (-90,  90, 0), # -XY
                 (0,  90, 0),   # -XZ
                 (0, 180, 0)]   # -YZ
        if axes == 'xy': ax_c.view_init(elev=views[0][0], azim=views[0][1], roll=views[0][2])
        elif axes == 'xz': ax_c.view_init(elev=views[1][0], azim=views[1][1], roll=views[1][2])
        elif axes == 'yz': ax_c.view_init(elev=views[2][0], azim=views[2][1], roll=views[2][2])
        ax_c.set_xlabel('X')
        ax_c.set_ylabel('Y')
        ax_c.set_zlabel('Z')
        ax_c.set_proj_type('ortho')
        ax_c.set_box_aspect(None, zoom=1.25)
        ax_c.grid(True)
        ## set panes color
        ax_c.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        ax_c.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        ax_c.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))

    def plot_outer_workspace_point(self, ax=None, res=5, size=5, axes='xyz'):
        q1 = np.linspace(self.joint_limits[0][0], self.joint_limits[0][1], res*2)
        q2 = np.linspace(self.joint_limits[1][0], self.joint_limits[1][1], res)
        q3 = np.linspace(self.joint_limits[2][0], self.joint_limits[2][1], res)
        
        x = []
        y = []
        z = []
        for i in range(len(q2)):
            for j in range(len(q3)):
                T = mdh2tf(0, 0, self.d1, self.joint_limits[0][0]) \
                    @ mdh2tf(self.alpha1, self.a1, self.d2, np.rad2deg(np.deg2rad(q2)[i])) \
                    @ mdh2tf(self.alpha2, self.a2, self.d3, np.rad2deg(np.deg2rad(q3)[j])) \
                    @ mdh2tf(self.alpha3, self.a3, self.d4, 0)
                x.append(T[0, 3])
                y.append(T[1, 3])
                z.append(T[2, 3])
        for i in range(len(q2)):
            for j in range(len(q3)):
                T = mdh2tf(0, 0, self.d1, self.joint_limits[0][1]) \
                    @ mdh2tf(self.alpha1, self.a1, self.d2, np.rad2deg(np.deg2rad(q2)[i])) \
                    @ mdh2tf(self.alpha2, self.a2, self.d3, np.rad2deg(np.deg2rad(q3)[j])) \
                    @ mdh2tf(self.alpha3, self.a3, self.d4, 0)
                x.append(T[0, 3])
                y.append(T[1, 3])
                z.append(T[2, 3])
                
        for i in range(len(q1)):
            for j in range(len(q3)):
                T = mdh2tf(0, 0, self.d1, np.rad2deg(np.deg2rad(q1)[i])) \
                    @ mdh2tf(self.alpha1, self.a1, self.d2, self.joint_limits[1][0]) \
                    @ mdh2tf(self.alpha2, self.a2, self.d3, np.rad2deg(np.deg2rad(q3)[j])) \
                    @ mdh2tf(self.alpha3, self.a3, self.d4, 0)
                x.append(T[0, 3])
                y.append(T[1, 3])
                z.append(T[2, 3])
        for i in range(len(q1)):
            for j in range(len(q3)):
                T = mdh2tf(0, 0, self.d1, np.rad2deg(np.deg2rad(q1)[i])) \
                    @ mdh2tf(self.alpha1, self.a1, self.d2, self.joint_limits[1][1]) \
                    @ mdh2tf(self.alpha2, self.a2, self.d3, np.rad2deg(np.deg2rad(q3)[j])) \
                    @ mdh2tf(self.alpha3, self.a3, self.d4, 0)
                x.append(T[0, 3])
                y.append(T[1, 3])
                z.append(T[2, 3])
                
        for i in range(len(q1)):
            for j in range(len(q2)):
                T = mdh2tf(0, 0, self.d1, np.rad2deg(np.deg2rad(q1)[i])) \
                    @ mdh2tf(self.alpha1, self.a1, self.d2, np.rad2deg(np.deg2rad(q2)[j])) \
                    @ mdh2tf(self.alpha2, self.a2, self.d3, self.joint_limits[2][0]) \
                    @ mdh2tf(self.alpha3, self.a3, self.d4, 0)
                x.append(T[0, 3])
                y.append(T[1, 3])
                z.append(T[2, 3])
        for i in range(len(q1)):
            for j in range(len(q2)):
                T = mdh2tf(0, 0, self.d1, np.rad2deg(np.deg2rad(q1)[i])) \
                    @ mdh2tf(self.alpha1, self.a1, self.d2, np.rad2deg(np.deg2rad(q2)[j])) \
                    @ mdh2tf(self.alpha2, self.a2, self.d3, self.joint_limits[2][1]) \
                    @ mdh2tf(self.alpha3, self.a3, self.d4, 0)
                x.append(T[0, 3])
                y.append(T[1, 3])
                z.append(T[2, 3])    
        # rescale the workspace
        x = self.s[0]*x + self.t[0]
        y = self.s[1]*y + self.t[1]
        z = self.s[2]*z + self.t[2]

        if ax == None: fig, ax_c_p = plt.subplots(subplot_kw={'projection': '3d'})
        else: ax_c_p = ax
        ax_c_p.scatter(x, y, z, c=z, cmap='viridis', s=5)

        # (plane, (elev, azim, roll))
        views = [(90, -90, 0),  # XY
                 (0, -90, 0),   # XZ
                 (0,   0, 0),   # YZ
                 (-90,  90, 0), # -XY
                 (0,  90, 0),   # -XZ
                 (0, 180, 0)]   # -YZ
        if axes == 'xy': ax_c_p.view_init(elev=views[0][0], azim=views[0][1], roll=views[0][2])
        elif axes == 'xz': ax_c_p.view_init(elev=views[1][0], azim=views[1][1], roll=views[1][2])
        elif axes == 'yz': ax_c_p.view_init(elev=views[2][0], azim=views[2][1], roll=views[2][2])
        ax_c_p.set_xlabel('X')
        ax_c_p.set_ylabel('Y')
        ax_c_p.set_zlabel('Z')
        ax_c_p.set_proj_type('ortho')
        ax_c_p.set_box_aspect(None, zoom=1.25)
        ax_c_p.grid(True)
        ## set panes color
        ax_c_p.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        ax_c_p.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        ax_c_p.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))

    def plot_outer_workspace_wireframe(self, ax=None, res=5, color='b', alpha=0.5, axes='xyz'):
        # Create a 3D plot
        if ax == None:
            fig, ax_c_w = plt.subplots(subplot_kw={'projection': '3d'})
        else:
            ax_c_w = ax
        # Generate the grid of joint angles
        q1 = np.linspace(self.joint_limits[0][0], self.joint_limits[0][1], res)
        q2 = np.linspace(self.joint_limits[1][0], self.joint_limits[1][1], res)
        q3 = np.linspace(self.joint_limits[2][0], self.joint_limits[2][1], res)
        q1, q2, q3 = np.meshgrid(q1, q2, q3)
        # Compute the workspace coordinates
        x = 150*np.cos(np.deg2rad(q1))*np.cos(np.deg2rad(q2)) - 150*np.cos(np.deg2rad(q1))*np.cos(np.deg2rad(q2))*np.sin(np.deg2rad(q3)) - 150*np.cos(np.deg2rad(q1))*np.cos(np.deg2rad(q3))*np.sin(np.deg2rad(q2))
        y = 150*np.cos(np.deg2rad(q2))*np.sin(np.deg2rad(q1)) - 150*np.cos(np.deg2rad(q2))*np.sin(np.deg2rad(q1))*np.sin(np.deg2rad(q3)) - 150*np.cos(np.deg2rad(q3))*np.sin(np.deg2rad(q1))*np.sin(np.deg2rad(q2))
        z = 150*np.sin(np.deg2rad(q2)) + 150*np.cos(np.deg2rad(q2))*np.cos(np.deg2rad(q3)) - 150*np.sin(np.deg2rad(q2))*np.sin(np.deg2rad(q3)) + 85
        # rescale the workspace
        x = self.s[0]*x + self.t[0]
        y = self.s[1]*y + self.t[1]
        z = self.s[2]*z + self.t[2]

        ax_c_w.plot_wireframe(x[res-1,:,:], y[res-1,:,:], z[res-1,:,:], color=color, alpha=alpha)
        ax_c_w.plot_wireframe(x[0,:,:], y[0,:,:], z[0,:,:], color=color, alpha=alpha)

        ax_c_w.plot_wireframe(x[:,res-1,:], y[:,res-1,:], z[:,res-1,:], color=color, alpha=alpha)
        ax_c_w.plot_wireframe(x[:,0,:], y[:,0,:], z[:,0,:], color=color, alpha=alpha)

        ax_c_w.plot_wireframe(x[:,:,res-1], y[:,:,res-1], z[:,:,res-1], color=color, alpha=alpha)
        ax_c_w.plot_wireframe(x[:,:,0], y[:,:,0], z[:,:,0], color=color, alpha=alpha)
        # Set plot labels and limits
        ax_c_w.set_xlabel('X')
        ax_c_w.set_ylabel('Y')
        ax_c_w.set_zlabel('Z')
        ## set panes color
        ax_c_w.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        ax_c_w.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        ax_c_w.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))

        # (plane, (elev, azim, roll))
        views = [(90, -90, 0),  # XY
                 (0, -90, 0),   # XZ
                 (0,   0, 0),   # YZ
                 (-90,  90, 0), # -XY
                 (0,  90, 0),   # -XZ
                 (0, 180, 0)]   # -YZ
        if axes == 'xy': ax_c_w.view_init(elev=views[0][0], azim=views[0][1], roll=views[0][2])
        elif axes == 'xz': ax_c_w.view_init(elev=views[1][0], azim=views[1][1], roll=views[1][2])
        elif axes == 'yz': ax_c_w.view_init(elev=views[2][0], azim=views[2][1], roll=views[2][2])
        ax_c_w.set_xlabel('X')
        ax_c_w.set_ylabel('Y')
        ax_c_w.set_zlabel('Z')
        ax_c_w.set_proj_type('ortho')
        ax_c_w.set_box_aspect(None, zoom=1.25)
        ax_c_w.grid(True)
        ## set panes color
        ax_c_w.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        ax_c_w.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        ax_c_w.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))

    def get_joint_variable(self, arg = 'degree'):
        if arg == 'degree':
            return [self.theta1, self.theta2, self.theta3, self.theta4, self.theta5, self.theta6]
        elif arg == 'radian':
            return [np.deg2rad(self.theta1), np.deg2rad(self.theta2), np.deg2rad(self.theta3), np.deg2rad(self.theta4), np.deg2rad(self.theta5), np.deg2rad(self.theta6)]

    def get_position_jacobian(self):
        '''
        @ Return [numpy.array] of Jacobian matrix
        '''
        Jp = np.zeros((3, 6))
        ## Compute the position of the end effector in the base frame
        p = self.T0_E[:3, 3]
        ## Define the unit vectors along the axes of rotation of each joint
        A = [0,0,0,0,0,0]
        if self.DH_param == 'mod':
            A[0] = self.T0_1[:3, 2]
            A[1] = self.T0_2[:3, 2]
            A[2] = self.T0_3[:3, 2]
            A[3] = self.T0_4[:3, 2]
            A[4] = self.T0_5[:3, 2]
            A[5] = self.T0_6[:3, 2]
            ## Compute the Jacobian matrix using the formula:
            Jp[:, 0] = np.cross(A[0], p - self.T0_1[:3, 3])
            Jp[:, 1] = np.cross(A[1], p - self.T0_2[:3, 3])
            Jp[:, 2] = np.cross(A[2], p - self.T0_3[:3, 3])
            # Jp[:, 3] = np.cross(A[3], p - self.T0_4[:3, 3])
            # Jp[:, 4] = np.cross(A[4], p - self.T0_5[:3, 3])
            # Jp[:, 5] = np.cross(A[5], p - self.T0_6[:3, 3])
        elif self.DH_param == 'std':
            A[0] = np.array([0, 0, 1])
            A[1] = self.T0_1[:3, 2]
            A[2] = self.T0_2[:3, 2]
            A[3] = self.T0_3[:3, 2]
            A[4] = self.T0_4[:3, 2]
            A[5] = self.T0_5[:3, 2]
            ## Compute the Jacobian matrix using the formula:
            Jp[:, 0] = np.cross(A[0], p)
            Jp[:, 1] = np.cross(A[1], p - self.T0_1[:3, 3])
            Jp[:, 2] = np.cross(A[2], p - self.T0_2[:3, 3])
            # Jp[:, 3] = np.cross(A[3], p - self.T0_3[:3, 3])
            # Jp[:, 4] = np.cross(A[4], p - self.T0_4[:3, 3])
            # Jp[:, 5] = np.cross(A[5], p - self.T0_5[:3, 3])

        return Jp

    def get_orientation_jacobian(self):
        '''
        @ Return [numpy.array] of Jacobian matrix
        '''
        Jo = np.zeros((3, 6))
        # Define the unit vectors along the axes of rotation of each joint
        A = [0,0,0,0,0,0]

        if self.DH_param == 'mod':
            A[0] = self.T0_1[:3, 2]
            A[1] = self.T0_2[:3, 2]
            A[2] = self.T0_3[:3, 2]
            A[3] = self.T0_4[:3, 2]
            A[4] = self.T0_5[:3, 2]
            A[5] = self.T0_6[:3, 2]
        elif self.DH_param == 'std':
            A[0] = np.array([0, 0, 1])
            A[1] = self.T0_1[:3, 2]
            A[2] = self.T0_2[:3, 2]
            A[3] = self.T0_3[:3, 2]
            A[4] = self.T0_4[:3, 2]
            A[5] = self.T0_5[:3, 2]

        # Jo[:, 0] = A[0] # joint1
        # Jo[:, 1] = A[1] # joint2
        # Jo[:, 2] = A[2] # joint3
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
            if unit == 'radian': ## transform it first to degree if unit in radian
                q[0] = np.rad2deg(q[0])
                q[1] = np.rad2deg(q[1])
                q[2] = np.rad2deg(q[2])
                q[3] = np.rad2deg(q[3])
                q[4] = np.rad2deg(q[4])
                q[5] = np.rad2deg(q[5])
            ## normalize if unit in degree
            q[0] = angle_normalize(q[0])
            q[1] = angle_normalize(q[1])
            q[2] = angle_normalize(q[2])
            q[3] = angle_normalize(q[3])
            q[4] = angle_normalize(q[4])
            q[5] = angle_normalize(q[5])
            ## apply joint limit
            # for i in range(len(q)):
            #     if q[i] < self.joint_limits[i][0]:
            #         q[i] = self.joint_limits[i][0]
            #     elif q[i] > self.joint_limits[i][1]:
            #         q[i] = self.joint_limits[i][1]
            ## update joint limits of theta3 based on theta2
            # self.joint_limits[2] = [linear_interpolate(self.theta2, 30, 0, 125, 240), linear_interpolate(self.theta2, 30, 0, 235, 250)] if 0 <= self.theta2 < 30 \
            #                         else [125, 235] if 30 <= self.theta2 < 70 \
            #                         else [125, linear_interpolate(self.theta2, 30, 0, 215, 235)] if 70 <= self.theta2 < 90 \
            #                         else [125, linear_interpolate(self.theta2, 30, 0, 145, 215)] if 90 <= self.theta2 < 160 \
            #                         else [125, 145]
            ## apply angle to joint
            self.theta1 = q[0]
            self.theta2 = q[1]
            self.theta3 = q[2]
            self.theta4 = q[3]
            self.theta5 = q[4]
            self.theta6 = q[5]

        H = self.update_tf_matrix()
        ## extract the position of the end effector
        x = np.round(H[0,3], 4)
        y = np.round(H[1,3], 4)
        z = np.round(H[2,3], 4)

        if rep == 'euler':
            # extract euler angle if rep == euler
            roll, pitch, yaw = np.round(get_euler_angle(H), 4)
            return [x, y, z, roll, pitch, yaw]
        elif rep == 'quaternion':
            # extract euler angle if rep == quaternions
            w, i, j, k = np.round(get_quaternion(H), 4)
            return [x, y, z, w, i, j, k]

    def differential_inverse_kinematics(self, position_d, orientation_d):
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

        if np.linalg.norm(error_o) > self.orientation_thresh or np.linalg.norm(error_p) > self.position_thresh:
            ## delta_p
            delta_p[:3] = self.Ko*error_o
            delta_p[3:] = self.Kp*error_p

            ## delta_q
            # delta_q = np.dot(np.linalg.inv(J.T.dot(J)), J.T.dot(delta_p))
            # delta_q = np.linalg.solve(J.T.dot(J), J.T.dot(delta_p))
            delta_q = np.linalg.pinv(J) @ delta_p

            ## joint speed limit
            for i in range(len(delta_q)):
                if i < 3: # position speed limit
                    if delta_q[i] < -self.max_speed_pos: delta_q[i] = -self.max_speed_pos
                    elif delta_q[i] > self.max_speed_pos: delta_q[i] = self.max_speed_pos
                else: # orientation speed limit
                    if delta_q[i] < -self.max_speed_ori: delta_q[i] = -self.max_speed_ori
                    elif delta_q[i] > self.max_speed_ori: delta_q[i] = self.max_speed_ori

            q_d = np.round((q + self.alpha*delta_q), 4).tolist()

            ## print
            # print("joint old   ", np.round(q, 4))
            # print("pose old    ", pose_current)
            # print("pose target ", position_d, orientation_d)
            # print("velocity    ", np.round(delta_q, 4).tolist())
            # print("joint new   ", np.round(self.get_joint_variable(), 4))
            # print("pose new    ", self.forward_kinematics(rep = 'quaternion'))

            self.forward_kinematics(q = q_d)
            return q_d
       