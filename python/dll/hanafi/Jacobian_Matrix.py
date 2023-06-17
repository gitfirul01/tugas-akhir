import numpy as np
from DH_Parameter import *

# About Jacobian Matrix
# Delta_p = J * (Delta_j)
# Jacobian Matrix is 6xn Matrix which mean 6x11 for this robot
# The first 3 row for position and the remaining for rotation of the end effector
# |  dotx |     | var1_1 var1_2 ... var1_11 |   |delta_j1 |
# |  doty |     | var2_1 var2_2 ... var2_11 |   |delta_j2 |
# |  dotz |  =  | var3_1 var3_2 ... var3_11 | . |    :    |
# |omega_x|     | var4_1 var4_2 ... var4_11 |   |deljad_6 |
# |omega_y|     | var5_1 var5_2 ... var5_11 |   |    :    |
# |omega_z|     | var6_1 var6_2 ... var6_11 |   |delta_j11|
# For more information how to calculate Jacobian Matrix, you can visit : https://youtu.be/SefTCXrpL8U

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

# Jacobian Matrix
J_Matrix = np.matrix([[0, var12_32[0,0], 0, var14_34[0,0], 0, var16_36[0,0], 0, var18_38[0,0], var19_39[0,0], var110_310[0,0], 0],
                      [0, var12_32[0,1], 0, var14_34[0,1], 0, var16_36[0,1], 0, var18_38[0,1], var19_39[0,1], var110_310[0,1], 0],
                      [0, var12_32[0,2], 0, var14_34[0,2], 0, var16_36[0,2], 0, var18_38[0,2], var19_39[0,2], var110_310[0,2], 0],
                      [0, var42_62[0,0], 0, var44_64[0,0], 0, var46_66[0,0], 0, var48_68[0,0], var49_69[0,0], var410_610[0,0], 0],
                      [0, var42_62[0,1], 0, var44_64[0,1], 0, var46_66[0,1], 0, var48_68[0,1], var49_69[0,1], var410_610[0,1], 0],
                      [0, var42_62[0,2], 0, var44_64[0,2], 0, var46_66[0,2], 0, var48_68[0,2], var49_69[0,2], var410_610[0,2], 0]])

# Invert the Jacobian Matrix
J_Inverse = np.linalg.pinv(J_Matrix, rcond=1e-15, hermitian=False)

'''
print("Jacobian Inverse:")
print(J_Inverse, '\n')
'''