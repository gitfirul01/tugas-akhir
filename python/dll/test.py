from manipulator import *
from controller import *
from function import *
import sympy as sp

# theta1, theta2, x = sp.symbols('theta1 theta2 x')
# print(Rz(theta2, True ) * Rx(theta1, True))
# print(Tx(100, True) * Rz(90))

manipulator = Manipulator() 
controller = Controller() 

# print(controller.get_jacobian_form())
print(controller.get_jacobian())

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


# x = 389.35
# y = 127.83
# z = 100
# # w = 0.3827  
# # i = 0
# # j = 0.9239 
# # k = 0
# yaw = -37
# pitch = 0
# roll = 90

# # p_d = [x, y, z, w, i, j, k]
# p_d = [x, y, z, yaw, pitch, roll]
# while True:
#     q_new = manipulator.inverse_kinematics(p_d)
#     manipulator.update_joint_variable(q_new)


