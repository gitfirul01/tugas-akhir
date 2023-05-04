import numpy as np
import sympy as sp

def Rx(phi, symbol = False):
    '''
    @Return rotation matrix about x-axis
    @Param:
        phi    - the rotational value in degree
        symbol - (True) return symbols matrix
               - (False) return valued matrix
    '''
    if not symbol:
        phi = np.deg2rad(phi)
        return np.array([
                [1, 0, 0, 0],
                [0, np.cos(phi), -np.sin(phi), 0],
                [0, np.sin(phi), np.cos(phi), 0],
                [0, 0, 0, 1]
                ])
    else:
        return sp.Matrix([
                [1, 0, 0, 0],
                [0, sp.cos(phi), -sp.sin(phi), 0],
                [0, sp.sin(phi), sp.cos(phi), 0],
                [0, 0, 0, 1]
                ])

def Ry(psi, symbol = False):
    '''
    @Return rotation matrix about y-axis
    @Param:
        psi    - the rotational value in degree
        symbol - (True) return symbols matrix
               - (False) return valued matrix
    '''
    if not symbol:
        psi = np.deg2rad(psi)
        return np.array([
                [np.cos(psi), 0, np.sin(psi), 0],
                [0, 1, 0, 0],
                [-np.sin(psi), 0, np.cos(psi), 0],
                [0, 0, 0, 1]
                ])
    else:
        return sp.Matrix([
                [sp.cos(psi), 0, sp.sin(psi), 0],
                [0, 1, 0, 0],
                [-sp.sin(psi), 0, sp.cos(psi), 0],
                [0, 0, 0, 1]
                ])

def Rz(theta, symbol = False):
    '''
    @Return rotation matrix about z-axis
    @Param:
        theta  - the rotational value in degree
        symbol - (True) return symbols matrix
               - (False) return valued matrix
    '''
    if not symbol:
        theta = np.deg2rad(theta)
        return np.array([
                [np.cos(theta), -np.sin(theta), 0, 0],
                [np.sin(theta), np.cos(theta), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
                ])
    else:
        return sp.Matrix([
                [sp.cos(theta), -sp.sin(theta), 0, 0],
                [sp.sin(theta), sp.cos(theta), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
                ])

def Tx(x, symbol = False):
    '''
    @Return transkation matrix over x-axis
    @Param:
        x - the translational value
    '''
    if not symbol:
        return np.array([
                [1, 0, 0, x], 
                [0, 1, 0, 0], 
                [0, 0, 1, 0],
                [0, 0, 0, 1]
                ])
    else:
        return sp.Matrix([
        [1, 0, 0, x], 
        [0, 1, 0, 0], 
        [0, 0, 1, 0],
        [0, 0, 0, 1]
        ])

def Ty(y, symbol = False):
    '''
    @Return transkation matrix over y-axis
    @Param:
        y - the translational value
    '''
    if not symbol:
        return np.array([
                [1, 0, 0, 0], 
                [0, 1, 0, y], 
                [0, 0, 1, 0],
                [0, 0, 0, 1]
                ])
    else:
        return sp.Matrix([
        [1, 0, 0, 0], 
        [0, 1, 0, y], 
        [0, 0, 1, 0],
        [0, 0, 0, 1]
        ])

def Tz(z, symbol = False):
    '''
    @Return transkation matrix over z-axis
    @Param:
        z - the translational value
    '''
    if not symbol:
        return np.array([
                [1, 0, 0, 0], 
                [0, 1, 0, 0], 
                [0, 0, 1, z],
                [0, 0, 0, 1]
                ])
    else:
        return sp.Matrix([
        [1, 0, 0, 0], 
        [0, 1, 0, 0], 
        [0, 0, 1, z],
        [0, 0, 0, 1]
        ])

def dh2tf(theta, d, a, alpha, symbol = False):
    '''
    @ Return 4x4 transformation matrix from DH-param:eters
    @ in sequence of Rz(theta) @ Tz(d) @ Tx(a) @ Rx(alpha)
    @ Param:
        theta - rotation about z-axis   | [radian] if symbol | [degree] if numpy
        d     - translation over z-axis | [radian] if symbol | [degree] if numpy
        a     - translation over x-axis | [radian] if symbol | [degree] if numpy
        alpha - rotation about x-axis   | [radian] if symbol | [degree] if numpy
    '''
    if not symbol:
        theta = np.deg2rad(theta)
        alpha = np.deg2rad(alpha)
        return np.array([
                [np.cos(theta),    -np.sin(theta)*np.cos(alpha),    np.sin(theta)*np.sin(alpha),    a*np.cos(theta)],
                [np.sin(theta),     np.cos(theta)*np.cos(alpha),   -np.cos(theta)*np.sin(alpha),    a*np.sin(theta)],
                [0,                 np.sin(alpha),                  np.cos(alpha),                  d],
                [0,                 0,                              0,                              1]
                ])
    else:
        return sp.Matrix([
                [sp.cos(theta),    -sp.sin(theta)*sp.cos(alpha),    sp.sin(theta)*sp.sin(alpha),    a*sp.cos(theta)],
                [sp.sin(theta),     sp.cos(theta)*sp.cos(alpha),   -sp.cos(theta)*sp.sin(alpha),    a*sp.sin(theta)],
                [0,                 sp.sin(alpha),                  sp.cos(alpha),                  d],
                [0,                 0,                              0,                              1]
                ])

def homogeneous_transformation_matrix(phi, psi, theta, x, y, z):
    '''
    @Return 4x4 transformation matrix
    @Param:
        phi   - rotation about x-axis
        psi   - rotation about y-axis
        theta - rotation about z-axis
        x     - translation over x-axis
        y     - translation over y-axis
        z     - translation over z-axis
    '''
    phi = np.deg2rad(phi)
    psi = np.deg2rad(psi)
    theta = np.deg2rad(theta)
    R = Rz(theta) @ Ry(psi) @ Rx(phi)
    T = Tx(x) @ Ty(y) @ Tz(z)

    return T @ R

def get_euler_convention(T):
    '''
    @Return euler convention used in transformation matrix
    @Param:
        T - 4x4 transformation matrix
    '''
    R = T[:3, :3]
    # Extract the individual rotation angles from the rotation matrix
    alpha = np.arctan2(R[2,1], R[2,2])
    beta = np.arctan2(-R[2,0], np.sqrt(R[2,1]**2 + R[2,2]**2))
    gamma = np.arctan2(R[1,0], R[0,0])
    
    # Determine the convention based on the order of rotations
    if np.isclose(beta, np.pi/2):
        convention = "ZXZ"
    elif np.isclose(beta, -np.pi/2):
        convention = "ZX'Z'"
    elif np.isclose(alpha, 0) and np.isclose(gamma, 0):
        convention = "ZYX"
    elif np.isclose(alpha, np.pi) and np.isclose(gamma, 0):
        convention = "YZX"
    elif np.isclose(alpha, 0) and np.isclose(gamma, np.pi):
        convention = "ZXY"
    elif np.isclose(alpha, np.pi) and np.isclose(gamma, np.pi):
        convention = "YXZ"
    else:
        convention = "Unknown convention"

    return convention

def get_euler_angle(T):
    '''
    @Return euler angle (yaw, pitch, roll) in ZYX convention
    @Param:
        T - 4x4 transformation matrix
    '''
    # Extract rotation matrix from transformation matrix
    R = T[:3, :3]
    # calculate pitch (around y-axis)
    pitch = np.arcsin(-R[2,0])
    # calculate roll (around x-axis) and yaw (around z-axis)
    if np.cos(pitch) == 0:
        roll = 0 # convention: roll is zero if pitch is +/- 90 degrees
        yaw = np.arctan2(-R[1,2], R[0,2])
    else:
        roll = np.arctan2(R[2,1], R[2,2])
        yaw = np.arctan2(R[1,0], R[0,0])
    # convert angles from radians to degrees
    yaw_deg = round(np.rad2deg(yaw), 2)
    pitch_deg = round(np.rad2deg(pitch), 2)
    roll_deg = round(np.rad2deg(roll), 2)

    return [yaw_deg, pitch_deg, roll_deg]

def get_quaternion(T):
    '''
    @Return quaternion variables (w, x, y, z)
    @Param:
        T - 4x4 transformation matrix
    '''
    R = T[:3, :3]  # Extract rotation matrix from transformation matrix
    tr = np.trace(R)
    if tr > 0:
        w = np.sqrt(1 + tr) / 2
        x = (R[2, 1] - R[1, 2]) / (4 * w)
        y = (R[0, 2] - R[2, 0]) / (4 * w)
        z = (R[1, 0] - R[0, 1]) / (4 * w)
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2 * np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2 * np.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2 * np.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
    
    w = np.round(w, 4)
    x = np.round(x, 4)
    y = np.round(y, 4)
    z = np.round(z, 4)

    return [w, x, y, z]
