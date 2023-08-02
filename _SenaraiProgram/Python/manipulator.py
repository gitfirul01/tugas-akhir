'''
    ROBOT MANIPULATOR USING MODIFIED DH-PARAMETER
'''
from function import *

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class Manipulator:
    def __init__(self):
        ## alpha-1
        self.alpha1 = 0
        self.alpha2 = 90
        self.alpha3 = 0
        self.alpha4 = 105
        self.alpha5 = 0
        self.alpha6 = 52
        self.alpha7 = 90
        self.alpha8 = 90
        self.alpha9 = 90
        self.alpha10 = 90
        self.alpha11 = 0
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
        self.a11 = 8
        ## d
        self.d1 = 148
        self.d2 = 0
        self.d3 = 179.33
        self.d4 = 0
        self.d5 = 0
        self.d6 = 100#var3
        self.d7 = 132
        self.d8 = 483
        self.d9 = 0
        self.d10 = 0
        self.d11 = 0
        ## theta
        self.theta1 = 180#90 #-switched
        self.theta2 = 45#var1
        self.theta3 = 90
        self.theta4 = 0#var2
        self.theta5 = 90
        self.theta6 = 0
        self.theta7 = 0
        self.theta8 = 90#var4
        self.theta9 = 90#var5
        self.theta10 = 0#var6
        self.theta11 = 0
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
            [39.37, 188.37],
            [-45, 225],
            [0, 180],
            [-90, 90]
        ]
        ## trace variable
        self.pos = [[],[],[]]
        ## IK variable
        self.position_thresh = 1
        self.orientation_thresh = 1e-3
        self.Ko = 30
        self.Kp = 20
        self.alpha = 0.3 # update rate
        self.max_speed_pos = 5
        self.max_speed_ori = 10
        ## workspace variable
        self.s = [1, 1, 1] # scale
        self.t = [0, 0, 0] # translate

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
        x = self.T0_11[0,3]
        y = self.T0_11[1,3]
        z = self.T0_11[2,3]
        
        ## clear axes plot
        ax.cla()

        ## link plot
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
        # plot_frame_scatter(self.T0_2)##var
        # plot_frame_scatter(self.T0_3)
        # plot_frame_scatter(self.T0_4)##var
        # plot_frame_scatter(self.T0_5)
        # plot_frame_scatter(self.T0_6)##var
        # plot_frame_scatter(self.T0_7)
        # plot_frame_scatter(self.T0_8)
        # plot_frame_scatter(self.T0_9)
        # plot_frame_scatter(self.T0_10)
        plot_frame_scatter(self.T0_11)
        
        ## frame plot
        # plot_frame_coordinate(self.T0_1)
        # plot_frame_coordinate(self.T0_2)##var
        # plot_frame_coordinate(self.T0_3)
        # plot_frame_coordinate(self.T0_4)##var
        # plot_frame_coordinate(self.T0_5)
        # plot_frame_coordinate(self.T0_6)##var
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
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_xlim(-200, 200)#ax.set_xlim(0, 600)
        ax.set_ylim(0, 500)#ax.set_ylim(-300, 300)
        ax.set_zlim(0, 700)
        ## set panes color
        ax.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        ax.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        ax.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        
        ## print the position of the end effector
        # print("Joint ", self.get_joint_variable())
        # print("Pose  ", self.forward_kinematics(rep = 'quaternion'))
        # print(get_euler_convention(self.T0_11))

        ## show the plot
        # fig.canvas.draw()
        # fig.canvas.flush_events()

    def plot_workspace_point(self, ax=None, res=5, size=5, color=None, axes='xyz', alpha=1):
        q1 = np.linspace(self.joint_limits[0][0], self.joint_limits[0][1], res)
        q2 = np.linspace(self.joint_limits[1][0], self.joint_limits[1][1], res)
        q3 = np.linspace(self.joint_limits[2][0], self.joint_limits[2][1], res)
        # q4 = np.linspace(self.joint_limits[3][0], self.joint_limits[3][1], res)
        # q5 = np.linspace(self.joint_limits[4][0], self.joint_limits[4][1], res)
        # q6 = np.linspace(self.joint_limits[5][0], self.joint_limits[5][1], res)

        x = []
        y = []
        z = []
        for i in range(len(q1)):
            for j in range(len(q2)):
                for k in range(len(q3)):
                    # for l in range(len(q4)):
                    #     for m in range(len(q5)):
                    #         for n in range(len(q6)):
                    T = mdh2tf(self.alpha1, self.a1, self.d1, self.theta1) \
                        @ mdh2tf(self.alpha2, self.a2, self.d2, q1[i]) \
                        @ mdh2tf(self.alpha3, self.a3, self.d3, self.theta3) \
                        @ mdh2tf(self.alpha4, self.a4, self.d4, q2[j]) \
                        @ mdh2tf(self.alpha5, self.a5, self.d5, self.theta5) \
                        @ mdh2tf(self.alpha6, self.a6, q3[k], self.theta6) \
                        @ mdh2tf(self.alpha7, self.a7, self.d7, self.theta7) \
                        @ mdh2tf(self.alpha8, self.a8, self.d8, 90) \
                        @ mdh2tf(self.alpha9, self.a9, self.d9, 90) \
                        @ mdh2tf(self.alpha10, self.a10, self.d10, 0) \
                        @ mdh2tf(self.alpha11, self.a11, self.d11, self.theta11)
                    x.append(T[0, 3])
                    y.append(T[1, 3])
                    z.append(T[2, 3])
        # rescale the workspace
        x = ( self.t[0] + self.s[0]*(np.array(x) - 28.1005) + 28.1005 ).tolist()
        y = ( self.t[1] + self.s[1]*(np.array(y) - 281.944) + 281.944 ).tolist()
        z = ( self.t[2] + self.s[2]*(np.array(z) - 65.0495) + 65.0495 ).tolist()

        if ax == None: fig, ax_r = plt.subplots(subplot_kw={'projection': '3d'})
        else: ax_r = ax

        if color==None: ax_r.scatter(x, y, z, c=z, cmap='viridis', s=5, alpha=alpha)
        else: ax_r.scatter(x, y, z, c=color, s=5, alpha=alpha)
        
        # (plane, (elev, azim, roll))
        views = [(90, -90, 0),  # XY
                 (0, -90, 0),   # XZ
                 (0,   0, 0),   # YZ
                 (-90,  90, 0), # -XY
                 (0,  90, 0),   # -XZ
                 (0, 180, 0)]   # -YZ
        if axes == 'xy': ax_r.view_init(elev=views[0][0], azim=views[0][1], roll=views[0][2])
        elif axes == 'xz': ax_r.view_init(elev=views[1][0], azim=views[1][1], roll=views[1][2])
        elif axes == 'yz': ax_r.view_init(elev=views[2][0], azim=views[2][1], roll=views[2][2])
        ax_r.set_xlabel('X')
        ax_r.set_ylabel('Y')
        ax_r.set_zlabel('Z')
        ax_r.set_proj_type('ortho')
        ax_r.set_box_aspect(None, zoom=1.25)
        ax_r.grid(True)
        ## set panes color
        ax_r.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        ax_r.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        ax_r.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))

    def plot_outer_workspace_point(self, ax=None, res=5, size=5, axes='xyz'):
        q1 = np.linspace(self.joint_limits[0][0], self.joint_limits[0][1], res)
        q2 = np.linspace(self.joint_limits[1][0], self.joint_limits[1][1], res)
        q3 = np.linspace(self.joint_limits[2][0], self.joint_limits[2][1], res)
        
        x = []
        y = []
        z = []
        for i in range(len(q2)):
            for j in range(len(q3)):
                T = mdh2tf(self.alpha1, self.a1, self.d1, self.theta1) \
                    @ mdh2tf(self.alpha2, self.a2, self.d2, self.joint_limits[0][0]) \
                    @ mdh2tf(self.alpha3, self.a3, self.d3, self.theta3) \
                    @ mdh2tf(self.alpha4, self.a4, self.d4, q2[i]) \
                    @ mdh2tf(self.alpha5, self.a5, self.d5, self.theta5) \
                    @ mdh2tf(self.alpha6, self.a6, q3[j], self.theta6) \
                    @ mdh2tf(self.alpha7, self.a7, self.d7, self.theta7) \
                    @ mdh2tf(self.alpha8, self.a8, self.d8, 90) \
                    @ mdh2tf(self.alpha9, self.a9, self.d9, 90) \
                    @ mdh2tf(self.alpha10, self.a10, self.d10, 0) \
                    @ mdh2tf(self.alpha11, self.a11, self.d11, self.theta11)
                x.append(T[0, 3])
                y.append(T[1, 3])
                z.append(T[2, 3])
        for i in range(len(q2)):
            for j in range(len(q3)):
                T = mdh2tf(self.alpha1, self.a1, self.d1, self.theta1) \
                    @ mdh2tf(self.alpha2, self.a2, self.d2, self.joint_limits[0][1]) \
                    @ mdh2tf(self.alpha3, self.a3, self.d3, self.theta3) \
                    @ mdh2tf(self.alpha4, self.a4, self.d4, q2[i]) \
                    @ mdh2tf(self.alpha5, self.a5, self.d5, self.theta5) \
                    @ mdh2tf(self.alpha6, self.a6, q3[j], self.theta6) \
                    @ mdh2tf(self.alpha7, self.a7, self.d7, self.theta7) \
                    @ mdh2tf(self.alpha8, self.a8, self.d8, 90) \
                    @ mdh2tf(self.alpha9, self.a9, self.d9, 90) \
                    @ mdh2tf(self.alpha10, self.a10, self.d10, 0) \
                    @ mdh2tf(self.alpha11, self.a11, self.d11, self.theta11)
                x.append(T[0, 3])
                y.append(T[1, 3])
                z.append(T[2, 3])
                
        for i in range(len(q1)):
            for j in range(len(q3)):
                T = mdh2tf(self.alpha1, self.a1, self.d1, self.theta1) \
                    @ mdh2tf(self.alpha2, self.a2, self.d2, q1[i]) \
                    @ mdh2tf(self.alpha3, self.a3, self.d3, self.theta3) \
                    @ mdh2tf(self.alpha4, self.a4, self.d4, self.joint_limits[1][0]) \
                    @ mdh2tf(self.alpha5, self.a5, self.d5, self.theta5) \
                    @ mdh2tf(self.alpha6, self.a6, q3[j], self.theta6) \
                    @ mdh2tf(self.alpha7, self.a7, self.d7, self.theta7) \
                    @ mdh2tf(self.alpha8, self.a8, self.d8, 90) \
                    @ mdh2tf(self.alpha9, self.a9, self.d9, 90) \
                    @ mdh2tf(self.alpha10, self.a10, self.d10, 0) \
                    @ mdh2tf(self.alpha11, self.a11, self.d11, self.theta11)
                x.append(T[0, 3])
                y.append(T[1, 3])
                z.append(T[2, 3])
        for i in range(len(q1)):
            for j in range(len(q3)):
                T = mdh2tf(self.alpha1, self.a1, self.d1, self.theta1) \
                    @ mdh2tf(self.alpha2, self.a2, self.d2, q1[i]) \
                    @ mdh2tf(self.alpha3, self.a3, self.d3, self.theta3) \
                    @ mdh2tf(self.alpha4, self.a4, self.d4, self.joint_limits[1][1]) \
                    @ mdh2tf(self.alpha5, self.a5, self.d5, self.theta5) \
                    @ mdh2tf(self.alpha6, self.a6, q3[j], self.theta6) \
                    @ mdh2tf(self.alpha7, self.a7, self.d7, self.theta7) \
                    @ mdh2tf(self.alpha8, self.a8, self.d8, 90) \
                    @ mdh2tf(self.alpha9, self.a9, self.d9, 90) \
                    @ mdh2tf(self.alpha10, self.a10, self.d10, 0) \
                    @ mdh2tf(self.alpha11, self.a11, self.d11, self.theta11)
                x.append(T[0, 3])
                y.append(T[1, 3])
                z.append(T[2, 3])
                
        for i in range(len(q1)):
            for j in range(len(q2)):
                T = mdh2tf(self.alpha1, self.a1, self.d1, self.theta1) \
                    @ mdh2tf(self.alpha2, self.a2, self.d2, q1[i]) \
                    @ mdh2tf(self.alpha3, self.a3, self.d3, self.theta3) \
                    @ mdh2tf(self.alpha4, self.a4, self.d4, q2[j]) \
                    @ mdh2tf(self.alpha5, self.a5, self.d5, self.theta5) \
                    @ mdh2tf(self.alpha6, self.a6, self.joint_limits[2][0], self.theta6) \
                    @ mdh2tf(self.alpha7, self.a7, self.d7, self.theta7) \
                    @ mdh2tf(self.alpha8, self.a8, self.d8, 90) \
                    @ mdh2tf(self.alpha9, self.a9, self.d9, 90) \
                    @ mdh2tf(self.alpha10, self.a10, self.d10, 0) \
                    @ mdh2tf(self.alpha11, self.a11, self.d11, self.theta11)
                x.append(T[0, 3])
                y.append(T[1, 3])
                z.append(T[2, 3])
        for i in range(len(q1)):
            for j in range(len(q2)):
                T = mdh2tf(self.alpha1, self.a1, self.d1, self.theta1) \
                    @ mdh2tf(self.alpha2, self.a2, self.d2, q1[i]) \
                    @ mdh2tf(self.alpha3, self.a3, self.d3, self.theta3) \
                    @ mdh2tf(self.alpha4, self.a4, self.d4, q2[j]) \
                    @ mdh2tf(self.alpha5, self.a5, self.d5, self.theta5) \
                    @ mdh2tf(self.alpha6, self.a6, self.joint_limits[2][1], self.theta6) \
                    @ mdh2tf(self.alpha7, self.a7, self.d7, self.theta7) \
                    @ mdh2tf(self.alpha8, self.a8, self.d8, 90) \
                    @ mdh2tf(self.alpha9, self.a9, self.d9, 90) \
                    @ mdh2tf(self.alpha10, self.a10, self.d10, 0) \
                    @ mdh2tf(self.alpha11, self.a11, self.d11, self.theta11)
                x.append(T[0, 3])
                y.append(T[1, 3])
                z.append(T[2, 3])
        # rescale the workspace
        x = self.t[0] + self.s[0]*(x - 28.1005) + 28.1005
        y = self.t[1] + self.s[1]*(y - 281.944) + 281.944
        z = self.t[2] + self.s[2]*(z - 65.0495) + 65.0495

        # print(np.round(x,3).tolist())
        # print(np.round(y,3).tolist())
        # print(np.round(z,3).tolist())
        if ax == None: fig, ax_r_p = plt.subplots(subplot_kw={'projection': '3d'})
        else: ax_r_p = ax
        ax_r_p.scatter(x, y, z, c=z, cmap='viridis', s=5)
        
        # (plane, (elev, azim, roll))
        views = [(90, -90, 0),  # XY
                 (0, -90, 0),   # XZ
                 (0,   0, 0),   # YZ
                 (-90,  90, 0), # -XY
                 (0,  90, 0),   # -XZ
                 (0, 180, 0)]   # -YZ
        if axes == 'xy': ax_r_p.view_init(elev=views[0][0], azim=views[0][1], roll=views[0][2])
        elif axes == 'xz': ax_r_p.view_init(elev=views[1][0], azim=views[1][1], roll=views[1][2])
        elif axes == 'yz': ax_r_p.view_init(elev=views[2][0], azim=views[2][1], roll=views[2][2])
        ax_r_p.set_xlabel('X')
        ax_r_p.set_ylabel('Y')
        ax_r_p.set_zlabel('Z')
        ax_r_p.set_proj_type('ortho')
        ax_r_p.set_box_aspect(None, zoom=1.25)
        ax_r_p.grid(True)
        ## set panes color
        ax_r_p.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        ax_r_p.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        ax_r_p.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))

    def plot_outer_workspace_wireframe(self, ax=None, res=5, color='b', alpha=0.5, axes='xyz'):
        # Create a 3D plot
        if ax == None:
            fig, ax_r_w = plt.subplots(subplot_kw={'projection': '3d'})
        else:
            ax_r_w = ax
        # Generate the grid of joint angles
        q1 = np.linspace(self.joint_limits[0][0], self.joint_limits[0][1], res)
        q2 = np.linspace(self.joint_limits[1][0], self.joint_limits[1][1], res)
        q3 = np.linspace(self.joint_limits[2][0], self.joint_limits[2][1], res)
        q1, q2, q3 = np.meshgrid(q1, q2, q3)
        # Compute the workspace coordinates
        x = (378724173*np.cos(np.deg2rad(q1)))/2500000 + (77*np.sin(np.deg2rad(q1)))/2 - (197*q3*((259*np.cos(np.deg2rad(q1))*np.sin(np.deg2rad(q2)))/1000 - np.cos(np.deg2rad(q2))*np.sin(np.deg2rad(q1))))/250 - (37191*q3*np.cos(np.deg2rad(q1)))/62500 + (110082511*np.cos(np.deg2rad(q1))*np.sin(np.deg2rad(q2)))/2500000 - (425029*np.cos(np.deg2rad(q2))*np.sin(np.deg2rad(q1)))/2500
        y = (95151*q3*np.sin(np.deg2rad(q2)))/125000 - (205289007*np.sin(np.deg2rad(q2)))/1250000 - (19943*q3)/125000 + 1421754629/5000000
        z = (77*np.cos(np.deg2rad(q1)))/2 - (378724173*np.sin(np.deg2rad(q1)))/2500000 + (197*q3*(np.cos(np.deg2rad(q1))*np.cos(np.deg2rad(q2)) + (259*np.sin(np.deg2rad(q1))*np.sin(np.deg2rad(q2)))/1000))/250 + (37191*q3*np.sin(np.deg2rad(q1)))/62500 - (425029*np.cos(np.deg2rad(q1))*np.cos(np.deg2rad(q2)))/2500 - (110082511*np.sin(np.deg2rad(q1))*np.sin(np.deg2rad(q2)))/2500000 + 148
        # rescale the workspace
        x = self.t[0] + self.s[0]*(x - 28.1005) + 28.1005
        y = self.t[1] + self.s[1]*(y - 281.944) + 281.944
        z = self.t[2] + self.s[2]*(z - 65.0495) + 65.0495

        ax_r_w.plot_wireframe(x[res-1,:,:], y[res-1,:,:], z[res-1,:,:], color=color, alpha=alpha)
        ax_r_w.plot_wireframe(x[0,:,:], y[0,:,:], z[0,:,:], color=color, alpha=alpha)

        ax_r_w.plot_wireframe(x[:,res-1,:], y[:,res-1,:], z[:,res-1,:], color=color, alpha=alpha)
        ax_r_w.plot_wireframe(x[:,0,:], y[:,0,:], z[:,0,:], color=color, alpha=alpha)

        ax_r_w.plot_wireframe(x[:,:,res-1], y[:,:,res-1], z[:,:,res-1], color=color, alpha=alpha)
        ax_r_w.plot_wireframe(x[:,:,0], y[:,:,0], z[:,:,0], color=color, alpha=alpha)
        # Set plot labels and limits
        ax_r_w.set_xlabel('X')
        ax_r_w.set_ylabel('Y')
        ax_r_w.set_zlabel('Z')
        ## set panes color
        ax_r_w.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        ax_r_w.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        ax_r_w.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))

        # (plane, (elev, azim, roll))
        views = [(90, -90, 0),  # XY
                 (0, -90, 0),   # XZ
                 (0,   0, 0),   # YZ
                 (-90,  90, 0), # -XY
                 (0,  90, 0),   # -XZ
                 (0, 180, 0)]   # -YZ
        if axes == 'xy': ax_r_w.view_init(elev=views[0][0], azim=views[0][1], roll=views[0][2])
        elif axes == 'xz': ax_r_w.view_init(elev=views[1][0], azim=views[1][1], roll=views[1][2])
        elif axes == 'yz': ax_r_w.view_init(elev=views[2][0], azim=views[2][1], roll=views[2][2])
        ax_r_w.set_xlabel('X')
        ax_r_w.set_ylabel('Y')
        ax_r_w.set_zlabel('Z')
        ax_r_w.set_proj_type('ortho')
        ax_r_w.set_box_aspect(None, zoom=1.25)
        ax_r_w.grid(True)
        ## set panes color
        ax_r_w.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        ax_r_w.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        ax_r_w.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))

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
        # Jp[:, 3] = np.cross(A[7], p - self.T0_8[:3, 3]) # revolute
        # Jp[:, 4] = np.cross(A[8], p - self.T0_9[:3, 3]) # revolute
        # Jp[:, 5] = np.cross(A[9], p - self.T0_10[:3, 3]) # revolute

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

        # Jo[:, 0] = A[1] # revolute
        # Jo[:, 1] = A[3] # revolute
        # Jo[:, 2] = A[5] # prismatic
        Jo[:, 3] = A[7] # revolute
        Jo[:, 4] = A[8] # revolute
        Jo[:, 5] = A[9] # revolute

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
                q[2] = q[2]
                q[3] = np.rad2deg(q[3])
                q[4] = np.rad2deg(q[4])
                q[5] = np.rad2deg(q[5])
            ## normalize if unit in degree
            q[0] = angle_normalize(q[0])
            q[1] = angle_normalize(q[1])
            q[2] = q[2]
            q[3] = angle_normalize(q[3])
            q[4] = angle_normalize(q[4])
            q[5] = angle_normalize(q[5])
            ## apply joint limit
            for i in range(len(q)):
                if q[i] < self.joint_limits[i][0]:
                    q[i] = self.joint_limits[i][0]
                elif q[i] > self.joint_limits[i][1]:
                    q[i] = self.joint_limits[i][1]
            ## apply angle to joint
            self.theta2 = q[0]
            self.theta4 = q[1]
            self.d6     = q[2]
            self.theta8 = q[3]
            self.theta9 = q[4]
            self.theta10 = q[5]

        H = self.update_tf_matrix()
        ## extract the position of the end effector
        x = np.round(H[0,3], 4)
        y = np.round(H[1,3], 4)
        z = np.round(H[2,3], 4)
        if rep == 'euler':
            roll, pitch, yaw = np.round(get_euler_angle(H), 4)
            return [x, y, z, roll, pitch, yaw]
        elif rep == 'quaternion':
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

            q = self.forward_kinematics(q = q_d)
            return q_d
