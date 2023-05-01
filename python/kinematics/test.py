from manipulator import *
from function import *
import sympy as sp

# theta1, theta2, x = sp.symbols('theta1 theta2 x')
# print(Rz(theta2, True ) * Rx(theta1, True))
# print(Tx(100, True) * Rz(90))

# print(get_jacobian_form())

manipulator = Manipulator() 

# theta1 = 15
# theta2 = 30
# d3 = 100
# theta4 = 45
# theta5 = 60
# theta6 = 90
# q = [theta1, theta2, d3, theta4, theta5, theta6]
# manipulator.update_joint_variable(q)
# print(get_jacobian())
# print(manipulator.forward_kinematics())



x = 212.13
y = 0
z = 45.00
# w = 0.3827  
# i = 0
# j = 0.9239 
# k = 0
yaw = np.round(np.deg2rad(180), 4)
pitch = np.round(np.deg2rad(45), 4)
roll = np.round(np.deg2rad(-180), 4)

# p_d = [x, y, z, w, i, j, k]
p_d = [x, y, z, yaw, pitch, roll]
print("old joint: ", manipulator.get_joint_variable())
print("old pose: ", manipulator.forward_kinematics())
print("target pose: ", p_d)
for i in range(100):
    q_new = manipulator.inverse_kinematics(p_d)
    manipulator.update_joint_variable(q_new)
print("new joint: ", manipulator.get_joint_variable())
print("new pose: ", manipulator.forward_kinematics())

