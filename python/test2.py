# import serial, struct
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


## confgure serial communication
# try:
#     ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
#     ser.reset_input_buffer()
# except:
#     print("Serial not connected!")
#     exit()
## turn on interactive plot
plt.ion()
## create a 3D figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')


## joint angle (x_i to xi over z_i)
theta1 = 0 #var
theta2 = 0 #var
theta3 = 0
## joint offset (O_i to xi over z_i)
d1 = 0
d2 = 0
d3 = 0 #var
## link lengths (Oi to z_i over xi)
a1 = 0
a2 = 0
a3 = 0
## link twist (z_i to zi over xi)
alpha1 = 0
alpha2 = 0
alpha3 = 0

def Rx(phi):
    phi = np.deg2rad(phi)
    return np.array([
            [1, 0, 0, 0],
            [0, np.cos(phi), -np.sin(phi), 0],
            [0, np.sin(phi), np.cos(phi), 0],
            [0, 0, 0, 1]
            ])
def Ry(psi):
    psi = np.deg2rad(psi)
    return np.array([
            [np.cos(psi), 0, np.sin(psi), 0],
            [0, 1, 0, 0],
            [-np.sin(psi), 0, np.cos(psi), 0],
            [0, 0, 0, 1]
            ])
def Rz(theta):
    theta = np.deg2rad(theta)
    return np.array([
            [np.cos(theta), -np.sin(theta), 0, 0],
            [np.sin(theta), np.cos(theta), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
            ])
def Txyz(x, y, z):
    return np.array([
            [1, 0, 0, x], 
            [0, 1, 0, y], 
            [0, 0, 1, z],
            [0, 0, 0, 1 ]
            ])
def homogeneous_transformation_matrix(phi, psi, theta, x, y, z):
    phi = np.deg2rad(phi)
    psi = np.deg2rad(psi)
    theta = np.deg2rad(theta)
    R = Rz(theta) @ Ry(psi) @ Rx(phi)
    T = Txyz(x,y,z)
    return T @ R



while True:
    theta1+=1
    theta2+=1
    d3+=1

    # ## read joint angle from serial
    # if ser.read(1) == b'@':
    #     data = ser.read_until(b'$')
    #     data = data[1:len(data)-1]
    #     try:
    #         value = struct.unpack('fff', data)
    #         theta1 = value[0]
    #         theta2 = value[1]
    #         theta3 = value[2]
    #     except:
    #         pass


    '''
    # Define the end-effector position and orientation (forward kinematics)
    end_effector_pos = np.array([0.5, 0.5, 0.5])
    end_effector_orientation = np.array([0.707, 0, 0, 0.707])  # Quaternion representation

    # Define the joint angles of the arm (sensor)
    joint_angles = np.array([0.1, 0.2, 0.3])

    # Define the Jacobian matrix
    J = np.array([[0.2, 0.3, 0.1],
                [-0.1, 0.4, 0.2],
                [0.1, -0.1, 0.3],
                [0, 0, 0.1],
                [0, 0, 0],
                [1, 1, 1]])

    # Calculate the current end-effector velocity
    current_velocities = J @ joint_angles

    # Define the desired end-effector velocity
    # v = J(q) * q_dot
    desired_velocities = np.array([0.1, -0.2, 0.3, 0.1, 0, 0])

    # Calculate the change in joint angles required to achieve the desired end-effector velocity
    # q_dot = J^-1(q) * v
    delta_joint_angles = np.linalg.pinv(J) @ desired_velocities

    # Calculate the new joint angles by adding the change in joint angles to the current joint angles
    new_joint_angles = joint_angles + delta_joint_angles

    print("Current Joint Angles:", joint_angles)
    print("New Joint Angles:", new_joint_angles)
    '''


    # Homogeneous transformation matrix from frame 0 to frame 1
    T01 = Txyz(0,0,100) @ Rx(90) @ Ry(90)           #homogeneous_transformation_matrix(0, 90, 0, 0, 0, 100)
    T12 = Rz(theta1) @ Txyz(180,0,240) @ Ry(-75)    #homogeneous_transformation_matrix(0, -75, 0, 180, 0, 240)
    T23 = Rz(theta2) @ Txyz(300,0,0) @ Ry(128)      #homogeneous_transformation_matrix(0, 128, 0, 300, 0, 0)
    T34 = Txyz(0,0,-d3-200)                         #homogeneous_transformation_matrix(0, 0, 0, 0, 0, -200)
    ## calculate the transformation matrix for the end effector
    T02 = T01 @ T12
    T03 = T02 @ T23
    T04 = T03 @ T34
    ## extract the position of the end effector
    x = T04[0,3]
    y = T04[1,3]
    z = T04[2,3]

    ## clear axes plot
    ax.cla()
    ## redraw plot
    ax.plot([0, T01[0,3]],          [0, T01[1,3]],          [0, T01[2,3]],          'y')
    ax.plot([T01[0,3], T02[0,3]],   [T01[1,3], T02[1,3]],   [T01[2,3], T02[2,3]],   'r')
    ax.plot([T02[0,3], T03[0,3]],   [T02[1,3], T03[1,3]],   [T02[2,3], T03[2,3]],   'g')
    ax.plot([T03[0,3], x],          [T03[1,3], y],          [T03[2,3], z],          'b')
    ax.scatter(x, y, z)

    ## add relative coordinate frames for each link
    O = np.array([[0, 0, 0]]).T
    X1 = T01[:3, :3] @ np.array([[1, 0, 0]]).T
    Y1 = T01[:3, :3] @ np.array([[0, 1, 0]]).T
    Z1 = T01[:3, :3] @ np.array([[0, 0, 1]]).T

    X2 = T02[:3, :3] @ np.array([[1, 0, 0]]).T
    Y2 = T02[:3, :3] @ np.array([[0, 1, 0]]).T
    Z2 = T02[:3, :3] @ np.array([[0, 0, 1]]).T

    X3 = T03[:3, :3] @ np.array([[1, 0, 0]]).T
    Y3 = T03[:3, :3] @ np.array([[0, 1, 0]]).T
    Z3 = T03[:3, :3] @ np.array([[0, 0, 1]]).T

    X4 = T04[:3, :3] @ np.array([[1, 0, 0]]).T
    Y4 = T04[:3, :3] @ np.array([[0, 1, 0]]).T
    Z4 = T04[:3, :3] @ np.array([[0, 0, 1]]).T

    ax.quiver(T01[0, 3], T01[1, 3], T01[2, 3], X1[0, 0], X1[1, 0], X1[2, 0], color='r', length=50)
    ax.quiver(T01[0, 3], T01[1, 3], T01[2, 3], Y1[0, 0], Y1[1, 0], Y1[2, 0], color='g', length=50)
    ax.quiver(T01[0, 3], T01[1, 3], T01[2, 3], Z1[0, 0], Z1[1, 0], Z1[2, 0], color='b', length=50) 
    
    ax.quiver(T02[0, 3], T02[1, 3], T02[2, 3], X2[0, 0], X2[1, 0], X2[2, 0], color='r', length=50)
    ax.quiver(T02[0, 3], T02[1, 3], T02[2, 3], Y2[0, 0], Y2[1, 0], Y2[2, 0], color='g', length=50)
    ax.quiver(T02[0, 3], T02[1, 3], T02[2, 3], Z2[0, 0], Z2[1, 0], Z2[2, 0], color='b', length=50) 
    
    ax.quiver(T03[0, 3], T03[1, 3], T03[2, 3], X3[0, 0], X3[1, 0], X3[2, 0], color='r', length=50)
    ax.quiver(T03[0, 3], T03[1, 3], T03[2, 3], Y3[0, 0], Y3[1, 0], Y3[2, 0], color='g', length=50)
    ax.quiver(T03[0, 3], T03[1, 3], T03[2, 3], Z3[0, 0], Z3[1, 0], Z3[2, 0], color='b', length=50) 

    ax.quiver(T04[0, 3], T04[1, 3], T04[2, 3], X3[0, 0], X3[1, 0], X3[2, 0], color='r', length=50)
    ax.quiver(T04[0, 3], T04[1, 3], T04[2, 3], Y3[0, 0], Y3[1, 0], Y3[2, 0], color='g', length=50)
    ax.quiver(T04[0, 3], T04[1, 3], T04[2, 3], Z3[0, 0], Z3[1, 0], Z3[2, 0], color='b', length=50) 
    
    ## set figure configuration
    # ax.set_title('3D Simulation')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim(0, 600)
    ax.set_ylim(-300, 300)
    ax.set_zlim(0, 600)
    
    ## print the position of the end effector
    print("Angle: ({:.2f}, {:.2f}, {:.2f})   EE-pos: ({:.2f}, {:.2f}, {:.2f})".format(theta1,theta2,d3,x,y,z))

    ## show the plot
    fig.canvas.draw()
    fig.canvas.flush_events()
    plt.pause(0.001)
