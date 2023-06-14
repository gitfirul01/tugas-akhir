import numpy as np

# Link lengths in millimeters
a3  = 240.21
a4  = 38.50
a5  = 150.32
a10  = 10
a11  = 7

d1  = 148
d3  = 179.33
d6  = 14.37 # (d6 + j_6 <= 188.16)
d7  = 122.15
d8  = 485

# Initialize values for the joints 
j_2 = 0
j_4 = 0
j_6 = 0
j_8 = 0
j_9 = 0 
j_10 = 0   

# Declare the Modified Denavit-Hartenberg table. 
# It will have four columns to represent:
# theta, alpha-1, a-1, and d
# the table will convert angles to radians.

d_h_table = np.matrix([[np.deg2rad(90),           np.deg2rad(0),        0,      d1      ],     # Joint 1
                       [np.deg2rad(0 + j_2),      np.deg2rad(90),       0,      0       ],     # Joint 2
                       [np.deg2rad(90),           np.deg2rad(0),        a3,     d3      ],     # Joint 3
                       [np.deg2rad(0 + j_4),      np.deg2rad(105),      a4,     0       ],     # Joint 4
                       [np.deg2rad(90),           np.deg2rad(0),        a5,     0       ],     # Joint 5
                       [np.deg2rad(0),            np.deg2rad(52),       0,      d6 + j_6],     # Joint 6
                       [np.deg2rad(0),            np.deg2rad(90),       0,      d7      ],     # Joint 7
                       [np.deg2rad(90 + j_8),     np.deg2rad(90),       0,      d8      ],     # Joint 8
                       [np.deg2rad(90 + j_9),     np.deg2rad(90),       0,      0       ],     # Joint 9
                       [np.deg2rad(0 + j_10),     np.deg2rad(90),       a10,    0       ],     # Joint 10
                       [np.deg2rad(0),            np.deg2rad(0),        a11,    0       ]])    # Joint 11 (End Effector)

# About Homogenous Matrix
# For Modified Denavit-Hartenberg table, we use these to calculate Homogenous Matrix:
#             |cos(theta_n),                -sin(theta_n),              0,              a              |
# (n-1)T(n) = |sin(theta_n)cos(alpha_n),    cos(theta_n)cos(alpha_n),   -sin(alpha_n),  -d_n sin(alpha)|
#             |sin(theta_n)sin(alpha_n),    cos(theta_n)sin(alpha_n),   cos(alpha_n),   d_n cos(alpha) |
#             |0,                           0,                          0,              1              |

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

'''
# Print the homogeneous transformation matrices
print("Homogeneous Matrix Frame 0 to Frame 11:")
print(H0_11, '\n')
'''