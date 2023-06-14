import numpy as np
from DH_Parameter import *
from Jacobian_Matrix import *

def clamp(num, min_value, max_value):
    return max(min(num, max_value), min_value)

def Update_d_h_table(joint_2, joint_4, joint_6, joint_8, joint_9, joint_10):
    # Update d_h_table
    global d_h_table
    
    d_h_table  = np.matrix([[np.deg2rad(90),           np.deg2rad(0),        0,      d1      ],     # Joint 1
                            [joint_2,                  np.deg2rad(90),       0,      0       ],     # Joint 2
                            [np.deg2rad(90),           np.deg2rad(0),        a3,     d3      ],     # Joint 3
                            [joint_4,                  np.deg2rad(105),      a4,     0       ],     # Joint 4
                            [np.deg2rad(90),           np.deg2rad(0),        a5,     0       ],     # Joint 5
                            [np.deg2rad(0),            np.deg2rad(52),       0,      joint_6 ],     # Joint 6
                            [np.deg2rad(0),            np.deg2rad(90),       0,      d7      ],     # Joint 7
                            [joint_8,                  np.deg2rad(90),       0,      d8      ],     # Joint 8
                            [joint_9,                  np.deg2rad(90),       0,      0       ],     # Joint 9
                            [joint_10,                 np.deg2rad(90),       a10,    0       ],     # Joint 10
                            [np.deg2rad(0),            np.deg2rad(0),        a11,    0       ]])    # Joint 11 (End Effector)

def Update_Homogenous():
    # Update H0_n
    global H0_1
    global H0_2
    global H0_3
    global H0_4
    global H0_5
    global H0_6
    global H0_7
    global H0_8
    global H0_9
    global H0_10
    global H0_11
    
    # Homogeneous transformation matrix from frame 0 to frame 1
    i = 0
    H0_1 = np.matrix([[np.cos(d_h_table[i,0]), -np.sin(d_h_table[i,0]), 0, d_h_table[i,2]],
                    [np.sin(d_h_table[i,0]) * np.cos(d_h_table[i,1]), np.cos(d_h_table[i,0]) * np.cos(d_h_table[i,1]), -np.sin(d_h_table[i,1]), -d_h_table[i,3] * np.sin(d_h_table[i,1])],
                    [np.sin(d_h_table[i,0]) * np.sin(d_h_table[i,1]), np.cos(d_h_table[i,0]) * np.sin(d_h_table[i,1]), np.cos(d_h_table[i,1]), d_h_table[i,3] * np.cos(d_h_table[i,1])],
                    [0, 0, 0, 1]])  
    
    # Homogeneous transformation matrix from frame 1 to frame 2
    i = 1
    H1_2 = np.matrix([[np.cos(d_h_table[i,0]), -np.sin(d_h_table[i,0]), 0, d_h_table[i,2]],
                    [np.sin(d_h_table[i,0]) * np.cos(d_h_table[i,1]), np.cos(d_h_table[i,0]) * np.cos(d_h_table[i,1]), -np.sin(d_h_table[i,1]), -d_h_table[i,3] * np.sin(d_h_table[i,1])],
                    [np.sin(d_h_table[i,0]) * np.sin(d_h_table[i,1]), np.cos(d_h_table[i,0]) * np.sin(d_h_table[i,1]), np.cos(d_h_table[i,1]), d_h_table[i,3] * np.cos(d_h_table[i,1])],
                    [0, 0, 0, 1]])  
    
    # Homogeneous transformation matrix from frame 2 to frame 3
    i = 2
    H2_3 = np.matrix([[np.cos(d_h_table[i,0]), -np.sin(d_h_table[i,0]), 0, d_h_table[i,2]],
                    [np.sin(d_h_table[i,0]) * np.cos(d_h_table[i,1]), np.cos(d_h_table[i,0]) * np.cos(d_h_table[i,1]), -np.sin(d_h_table[i,1]), -d_h_table[i,3] * np.sin(d_h_table[i,1])],
                    [np.sin(d_h_table[i,0]) * np.sin(d_h_table[i,1]), np.cos(d_h_table[i,0]) * np.sin(d_h_table[i,1]), np.cos(d_h_table[i,1]), d_h_table[i,3] * np.cos(d_h_table[i,1])],
                    [0, 0, 0, 1]])  
    
    # Homogeneous transformation matrix from frame 3 to frame 4
    i = 3
    H3_4 = np.matrix([[np.cos(d_h_table[i,0]), -np.sin(d_h_table[i,0]), 0, d_h_table[i,2]],
                    [np.sin(d_h_table[i,0]) * np.cos(d_h_table[i,1]), np.cos(d_h_table[i,0]) * np.cos(d_h_table[i,1]), -np.sin(d_h_table[i,1]), -d_h_table[i,3] * np.sin(d_h_table[i,1])],
                    [np.sin(d_h_table[i,0]) * np.sin(d_h_table[i,1]), np.cos(d_h_table[i,0]) * np.sin(d_h_table[i,1]), np.cos(d_h_table[i,1]), d_h_table[i,3] * np.cos(d_h_table[i,1])],
                    [0, 0, 0, 1]])  
    
    # Homogeneous transformation matrix from frame 4 to frame 5
    i = 4
    H4_5 = np.matrix([[np.cos(d_h_table[i,0]), -np.sin(d_h_table[i,0]), 0, d_h_table[i,2]],
                    [np.sin(d_h_table[i,0]) * np.cos(d_h_table[i,1]), np.cos(d_h_table[i,0]) * np.cos(d_h_table[i,1]), -np.sin(d_h_table[i,1]), -d_h_table[i,3] * np.sin(d_h_table[i,1])],
                    [np.sin(d_h_table[i,0]) * np.sin(d_h_table[i,1]), np.cos(d_h_table[i,0]) * np.sin(d_h_table[i,1]), np.cos(d_h_table[i,1]), d_h_table[i,3] * np.cos(d_h_table[i,1])],
                    [0, 0, 0, 1]])  

    # Homogeneous transformation matrix from frame 5 to frame 6
    i = 5
    H5_6 = np.matrix([[np.cos(d_h_table[i,0]), -np.sin(d_h_table[i,0]), 0, d_h_table[i,2]],
                    [np.sin(d_h_table[i,0]) * np.cos(d_h_table[i,1]), np.cos(d_h_table[i,0]) * np.cos(d_h_table[i,1]), -np.sin(d_h_table[i,1]), -d_h_table[i,3] * np.sin(d_h_table[i,1])],
                    [np.sin(d_h_table[i,0]) * np.sin(d_h_table[i,1]), np.cos(d_h_table[i,0]) * np.sin(d_h_table[i,1]), np.cos(d_h_table[i,1]), d_h_table[i,3] * np.cos(d_h_table[i,1])],
                    [0, 0, 0, 1]]) 

    # Homogeneous transformation matrix from frame 6 to frame 7
    i = 6
    H6_7 = np.matrix([[np.cos(d_h_table[i,0]), -np.sin(d_h_table[i,0]), 0, d_h_table[i,2]],
                    [np.sin(d_h_table[i,0]) * np.cos(d_h_table[i,1]), np.cos(d_h_table[i,0]) * np.cos(d_h_table[i,1]), -np.sin(d_h_table[i,1]), -d_h_table[i,3] * np.sin(d_h_table[i,1])],
                    [np.sin(d_h_table[i,0]) * np.sin(d_h_table[i,1]), np.cos(d_h_table[i,0]) * np.sin(d_h_table[i,1]), np.cos(d_h_table[i,1]), d_h_table[i,3] * np.cos(d_h_table[i,1])],
                    [0, 0, 0, 1]]) 

    # Homogeneous transformation matrix from frame 7 to frame 8
    i = 7
    H7_8 = np.matrix([[np.cos(d_h_table[i,0]), -np.sin(d_h_table[i,0]), 0, d_h_table[i,2]],
                    [np.sin(d_h_table[i,0]) * np.cos(d_h_table[i,1]), np.cos(d_h_table[i,0]) * np.cos(d_h_table[i,1]), -np.sin(d_h_table[i,1]), -d_h_table[i,3] * np.sin(d_h_table[i,1])],
                    [np.sin(d_h_table[i,0]) * np.sin(d_h_table[i,1]), np.cos(d_h_table[i,0]) * np.sin(d_h_table[i,1]), np.cos(d_h_table[i,1]), d_h_table[i,3] * np.cos(d_h_table[i,1])],
                    [0, 0, 0, 1]]) 

    # Homogeneous transformation matrix from frame 8 to frame 9
    i = 8
    H8_9 = np.matrix([[np.cos(d_h_table[i,0]), -np.sin(d_h_table[i,0]), 0, d_h_table[i,2]],
                    [np.sin(d_h_table[i,0]) * np.cos(d_h_table[i,1]), np.cos(d_h_table[i,0]) * np.cos(d_h_table[i,1]), -np.sin(d_h_table[i,1]), -d_h_table[i,3] * np.sin(d_h_table[i,1])],
                    [np.sin(d_h_table[i,0]) * np.sin(d_h_table[i,1]), np.cos(d_h_table[i,0]) * np.sin(d_h_table[i,1]), np.cos(d_h_table[i,1]), d_h_table[i,3] * np.cos(d_h_table[i,1])],
                    [0, 0, 0, 1]])

    # Homogeneous transformation matrix from frame 9 to frame 10
    i = 9
    H9_10 = np.matrix([[np.cos(d_h_table[i,0]), -np.sin(d_h_table[i,0]), 0, d_h_table[i,2]],
                    [np.sin(d_h_table[i,0]) * np.cos(d_h_table[i,1]), np.cos(d_h_table[i,0]) * np.cos(d_h_table[i,1]), -np.sin(d_h_table[i,1]), -d_h_table[i,3] * np.sin(d_h_table[i,1])],
                    [np.sin(d_h_table[i,0]) * np.sin(d_h_table[i,1]), np.cos(d_h_table[i,0]) * np.sin(d_h_table[i,1]), np.cos(d_h_table[i,1]), d_h_table[i,3] * np.cos(d_h_table[i,1])],
                    [0, 0, 0, 1]]) 

    # Homogeneous transformation matrix from frame 9 to frame 10
    i = 10
    H10_11 = np.matrix([[np.cos(d_h_table[i,0]), -np.sin(d_h_table[i,0]), 0, d_h_table[i,2]],
                        [np.sin(d_h_table[i,0]) * np.cos(d_h_table[i,1]), np.cos(d_h_table[i,0]) * np.cos(d_h_table[i,1]), -np.sin(d_h_table[i,1]), -d_h_table[i,3] * np.sin(d_h_table[i,1])],
                        [np.sin(d_h_table[i,0]) * np.sin(d_h_table[i,1]), np.cos(d_h_table[i,0]) * np.sin(d_h_table[i,1]), np.cos(d_h_table[i,1]), d_h_table[i,3] * np.cos(d_h_table[i,1])],
                        [0, 0, 0, 1]]) 

    # Get Homogeneous transformation matrix from frame 0 to 1-11
    # H0_1 = H0_1
    H0_2 = H0_1 @ H1_2 
    H0_3 = H0_2 @ H2_3 
    H0_4 = H0_3 @ H3_4 
    H0_5 = H0_4 @ H4_5 
    H0_6 = H0_5 @ H5_6 
    H0_7 = H0_6 @ H6_7 
    H0_8 = H0_7 @ H7_8 
    H0_9 = H0_8 @ H8_9 
    H0_10 = H0_9 @ H9_10 
    H0_11 = H0_10 @ H10_11

def Update_J_Matrix():
    # Rotational Matrix
    R0_1 = np.matrix([[H0_1[0,0], H0_1[0,1], H0_1[0,2]],
                        [H0_1[1,0], H0_1[1,1], H0_1[1,2]],
                        [H0_1[2,0], H0_1[2,1], H0_1[2,2]]])
    R0_3 = np.matrix([[H0_3[0,0], H0_3[0,1], H0_3[0,2]],
                        [H0_3[1,0], H0_3[1,1], H0_3[1,2]],
                        [H0_3[2,0], H0_3[2,1], H0_3[2,2]]])
    R0_5 = np.matrix([[H0_5[0,0], H0_5[0,1], H0_5[0,2]],
                        [H0_5[1,0], H0_5[1,1], H0_5[1,2]],
                        [H0_5[2,0], H0_5[2,1], H0_5[2,2]]])
    R0_7 = np.matrix([[H0_7[0,0], H0_7[0,1], H0_7[0,2]],
                        [H0_7[1,0], H0_7[1,1], H0_7[1,2]],
                        [H0_7[2,0], H0_7[2,1], H0_7[2,2]]])
    R0_8 = np.matrix([[H0_8[0,0], H0_8[0,1], H0_8[0,2]],
                        [H0_8[1,0], H0_8[1,1], H0_8[1,2]],
                        [H0_8[2,0], H0_8[2,1], H0_8[2,2]]])
    R0_9 = np.matrix([[H0_9[0,0], H0_9[0,1], H0_9[0,2]],
                        [H0_9[1,0], H0_9[1,1], H0_9[1,2]],
                        [H0_9[2,0], H0_9[2,1], H0_9[2,2]]])

    # Linear Matrix
    d0_1 = np.matrix([[H0_1[0,3]],
                        [H0_1[1,3]],
                        [H0_1[2,3]]])
    d0_3 = np.matrix([[H0_3[0,3]],
                        [H0_3[1,3]],
                        [H0_3[2,3]]])
    d0_5 = np.matrix([[H0_5[0,3]],
                        [H0_5[1,3]],
                        [H0_5[2,3]]])
    d0_7 = np.matrix([[H0_7[0,3]],
                        [H0_7[1,3]],
                        [H0_7[2,3]]])
    d0_8 = np.matrix([[H0_8[0,3]],
                        [H0_8[1,3]],
                        [H0_8[2,3]]])
    d0_9 = np.matrix([[H0_9[0,3]],
                        [H0_9[1,3]],
                        [H0_9[2,3]]])
    d0_11 = np.matrix([[H0_11[0,3]],
                        [H0_11[1,3]],
                        [H0_11[2,3]]])

    # Jacobian Matrix Element
    # Column 2 Revolute Joint
    var42_62 = np.transpose(R0_1 @ np.matrix([[0],[0],[1]]))
    var12_32 = np.cross(var42_62, np.transpose(d0_11 - d0_1))
    # Column 4 Revolute Joint
    var44_64 = np.transpose(R0_3 @ np.matrix([[0],[0],[1]]))
    var14_34 = np.cross(var44_64, np.transpose(d0_11 - d0_3))
    # Column 6 Prismatic Joint
    var46_66 = np.matrix([0, 0, 0])
    var16_36 = np.transpose(R0_5 @ np.matrix([[0],[0],[1]]))
    # Column 8 Revolute Joint
    var48_68 = np.transpose(R0_7 @ np.matrix([[0],[0],[1]]))
    var18_38 = np.cross(var48_68, np.transpose(d0_11 - d0_7))
    # Column 9 Revolute Joint
    var49_69 = np.transpose(R0_8 @ np.matrix([[0],[0],[1]]))
    var19_39 = np.cross(var49_69, np.transpose(d0_11 - d0_8))
    # Column 10 Revolute Joint
    var410_610 = np.transpose(R0_9 @ np.matrix([[0],[0],[1]]))
    var110_310 = np.cross(var410_610, np.transpose(d0_11 - d0_9))

    #Update J_Matrix and J_Inverse
    global J_Matrix
    global J_Inverse
    
    # Jacobian Matrix
    J_Matrix = np.matrix([[0, var12_32[0,0], 0, var14_34[0,0], 0, var16_36[0,0], 0, var18_38[0,0], var19_39[0,0], var110_310[0,0], 0],
                            [0, var12_32[0,1], 0, var14_34[0,1], 0, var16_36[0,1], 0, var18_38[0,1], var19_39[0,1], var110_310[0,1], 0],
                            [0, var12_32[0,2], 0, var14_34[0,2], 0, var16_36[0,2], 0, var18_38[0,2], var19_39[0,2], var110_310[0,2], 0],
                            [0, var42_62[0,0], 0, var44_64[0,0], 0, var46_66[0,0], 0, var48_68[0,0], var49_69[0,0], var410_610[0,0], 0],
                            [0, var42_62[0,1], 0, var44_64[0,1], 0, var46_66[0,1], 0, var48_68[0,1], var49_69[0,1], var410_610[0,1], 0],
                            [0, var42_62[0,2], 0, var44_64[0,2], 0, var46_66[0,2], 0, var48_68[0,2], var49_69[0,2], var410_610[0,2], 0]])

    # Invert the Jacobian Matrix
    J_Inverse = np.linalg.pinv(J_Matrix, rcond=1e-15, hermitian=False)

def Jacobian_IK(x, y, z, roll, pitch, yaw):
    for i in range (10):
        
        # Joint Value Init
        j0 = np.matrix([[d_h_table[0, 0]],     # Joint 1
                        [d_h_table[1, 0]],     # Joint 2
                        [d_h_table[2, 0]],     # Joint 3
                        [d_h_table[3, 0]],     # Joint 4
                        [d_h_table[4, 0]],     # Joint 5
                        [d_h_table[5, 3]],     # Joint 6
                        [d_h_table[6, 0]],     # Joint 7
                        [d_h_table[7, 0]],     # Joint 8
                        [d_h_table[8, 0]],     # Joint 9
                        [d_h_table[9, 0]],     # Joint 10
                        [d_h_table[10, 0]]])   # Joint 11 (End Effector)
    
        # Pose Init
        pitch0 = -np.arcsin(H0_11[0, 2])
        p0 = np.matrix([[H0_11[0, 3]],   
                        [H0_11[1, 3]],
                        [H0_11[2, 3]],
                        [np.arctan((-H0_11[1, 2]/np.cos(pitch0))/(H0_11[2, 2]/np.cos(pitch0)))],
                        [pitch0],
                        [np.arctan((-H0_11[0, 1]/np.cos(pitch0))/(H0_11[0, 0]/np.cos(pitch0)))]])
    
        # Pose Value
        p  = np.matrix([[x],
                        [y],
                        [z],
                        [roll],
                        [pitch],
                        [yaw]])
        
        # Calculate delta_P
        delta_p = p - p0
        
        # Inverse Kinematics
        # delta_j = J_inverse @ delta_p
        # | j_1 |     |      |     |p_1|
        # | j_2 |     |      |     |p_2|
        # |  :  |  =  |MATRIX|  @  |p_3|
        # |  :  |     |11 x 6|     |p_4|
        # |  :  |     |      |     |p_5|
        # | j_11|     |      |     |p_6|
        delta_j = J_Inverse @ delta_p
        
        # Modulo delta_j value
        delta_j[1,0] = delta_j[1,0] % np.deg2rad(360)
        delta_j[3,0] = delta_j[3,0] % np.deg2rad(360)
        delta_j[7,0] = delta_j[7,0] % np.deg2rad(360)
        delta_j[8,0] = delta_j[8,0] % np.deg2rad(360)
        delta_j[9,0] = delta_j[9,0] % np.deg2rad(360)
        
        # Clamp delta_j value
        delta_j[1,0] = clamp(delta_j[1,0], np.deg2rad(0), np.deg2rad(90))       # Joint 2 (0 - 90)
        delta_j[3,0] = clamp(delta_j[3,0], np.deg2rad(0), np.deg2rad(110))      # Joint 4 (-55 to 55)
        delta_j[5,0] = clamp(delta_j[5,0], 0, 173.39)                           # Joint 6 (14.37 to 188.16)
        delta_j[7,0] = clamp(delta_j[7,0], np.deg2rad(0), np.deg2rad(270))      # Joint 8 (-45 to 225)
        delta_j[8,0] = clamp(delta_j[8,0], np.deg2rad(0), np.deg2rad(180))      # Joint 9 (0 to 180)
        delta_j[9,0] = clamp(delta_j[9,0], np.deg2rad(0), np.deg2rad(180))      # Joint 10 (-90 to 90)
        
        j = delta_j + j0
        
        print('i :', i+1, '\n')
        print('j0 :', j0, '\n')
        print('j :', j, '\n')
        print('delta_J :', delta_j, '\n')
        print('p0 :', p0, '\n')
        print('p :', p, '\n')
        print('delta_p :', delta_p, '\n')
        print('J_Matrix :', J_Matrix, '\n')
        print('J_Inverse :', J_Inverse, '\n')        
        
        
        # Update DH_Table
        Update_d_h_table(j[1,0], j[3,0], j[5,0], j[7,0], j[8,0], j[9,0])
        
        # Update Homogenous Matrix
        Update_Homogenous()
        
        # Update J_Matrix and J_Inverse
        Update_J_Matrix()
        

while(1):
    Jacobian_IK(281.9441210783462, -142.75111343841218, 27.765365429783316, np.rad2deg(90), np.rad2deg(0), np.rad2deg(-90))
    break