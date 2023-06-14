import numpy as np

def Rx(phi):
    '''
    @Return rotation matrix about x-axis
    @Param:
        phi    - the rotational value in degree
    '''
    phi = np.deg2rad(phi)
    return np.array([
            [1, 0, 0, 0],
            [0, np.cos(phi), -np.sin(phi), 0],
            [0, np.sin(phi), np.cos(phi), 0],
            [0, 0, 0, 1]
            ])

def Ry(psi):
    '''
    @Return rotation matrix about y-axis
    @Param:
        psi    - the rotational value in degree
    '''
    psi = np.deg2rad(psi)
    return np.array([
            [np.cos(psi), 0, np.sin(psi), 0],
            [0, 1, 0, 0],
            [-np.sin(psi), 0, np.cos(psi), 0],
            [0, 0, 0, 1]
            ])
    
def Rz(theta):
    '''
    @Return rotation matrix about z-axis
    @Param:
        theta  - the rotational value in degree
    '''
    theta = np.deg2rad(theta)
    return np.array([
            [np.cos(theta), -np.sin(theta), 0, 0],
            [np.sin(theta), np.cos(theta), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
            ])
    
def Tx(x):
    '''
    @Return transkation matrix over x-axis
    @Param:
        x - the translational value
    '''
    return np.array([
            [1, 0, 0, x], 
            [0, 1, 0, 0], 
            [0, 0, 1, 0],
            [0, 0, 0, 1]
            ])

def Ty(y):
    '''
    @Return transkation matrix over y-axis
    @Param:
        y - the translational value
    '''
    return np.array([
            [1, 0, 0, 0], 
            [0, 1, 0, y], 
            [0, 0, 1, 0],
            [0, 0, 0, 1]
            ])

def Tz(z):
    '''
    @Return transkation matrix over z-axis
    @Param:
        z - the translational value
    '''
    return np.array([
            [1, 0, 0, 0], 
            [0, 1, 0, 0], 
            [0, 0, 1, z],
            [0, 0, 0, 1]
            ])

def dh2tf(theta, d, a, alpha):
    '''
    @ Return 4x4 transformation matrix from DH-param:eters
    @ in sequence of Rz(theta) @ Tz(d) @ Tx(a) @ Rx(alpha)
    @ Param:
        theta - rotation about z-axis   | [radian] if symbol | [degree] if numpy
        d     - translation over z-axis | [radian] if symbol | [degree] if numpy
        a     - translation over x-axis | [radian] if symbol | [degree] if numpy
        alpha - rotation about x-axis   | [radian] if symbol | [degree] if numpy
    '''
    theta = np.deg2rad(theta)
    alpha = np.deg2rad(alpha)
    return np.array([
            [np.cos(theta),    -np.sin(theta)*np.cos(alpha),    np.sin(theta)*np.sin(alpha),    a*np.cos(theta)],
            [np.sin(theta),     np.cos(theta)*np.cos(alpha),   -np.cos(theta)*np.sin(alpha),    a*np.sin(theta)],
            [0,                 np.sin(alpha),                  np.cos(alpha),                  d],
            [0,                 0,                              0,                              1]
            ])
    
def mdh2tf(alpha, a, d, theta):
    '''
    @ Return 4x4 transformation matrix from DH-param:eters
    @ in sequence of Rz(theta) @ Tz(d) @ Tx(a) @ Rx(alpha)
    @ Param:
        theta - rotation about z-axis   | [radian] if symbol | [degree] if numpy
        d     - translation over z-axis | [radian] if symbol | [degree] if numpy
        a     - translation over x-axis | [radian] if symbol | [degree] if numpy
        alpha - rotation about x-axis   | [radian] if symbol | [degree] if numpy
    '''
    theta = np.deg2rad(theta)
    alpha = np.deg2rad(alpha)
    return np.array([
            [np.cos(theta),                 -np.sin(theta),                 0,              a               ],
            [np.sin(theta)*np.cos(alpha),    np.cos(theta)*np.cos(alpha),  -np.sin(alpha),  -d*np.sin(alpha)],
            [np.sin(theta)*np.sin(alpha),    np.cos(theta)*np.sin(alpha),   np.cos(alpha),  d*np.cos(alpha) ],
            [0,                              0,                             0,              1               ]
            ])

def homogeneous_transformation_matrix(phi, psi, theta, x, y, z):
    '''
    Parameter:
        phi   - rotation about x-axis
        psi   - rotation about y-axis
        theta - rotation about z-axis
        x     - translation over x-axis
        y     - translation over y-axis
        z     - translation over z-axis
    Output:
        4x4 transformation matrix
    '''
    phi = np.deg2rad(phi)
    psi = np.deg2rad(psi)
    theta = np.deg2rad(theta)
    R = Rz(theta) @ Ry(psi) @ Rx(phi)
    T = Tx(x) @ Ty(y) @ Tz(z)

    return T @ R

def get_euler_convention(T):
    '''
    Parameter:
        T - 4x4 transformation matrix
    Output:
        Euler convention used in transformation matrix
    '''
    if T.shape == (4,4):
        R = T[:3, :3]  # Extract rotation matrix from transformation matrix
    elif T.shape == (3,3):
        R = T
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
    Parameter:
        T - 4x4 transformation matrix
    Output:
        Euler angle (roll, pitch, yaw) in ZYX convention
    '''
    if T.shape == (4,4):
        R = T[:3, :3]  # Extract rotation matrix from transformation matrix
    elif T.shape == (3,3):
        R = T
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

    return [roll_deg, pitch_deg, yaw_deg]

def get_quaternion(T):
    '''
    Parameter:
        T - 4x4 transformation matrix
    Output:
        Quaternion variables [w, x, y, z]
    '''
    if T.shape == (4,4):
        R = T[:3, :3]  # Extract rotation matrix from transformation matrix
    elif T.shape == (3,3):
        R = T
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

def quaternion_to_rotation_matrix(w, x, y, z):
    # Convert a quaternion to a 3x3 rotation matrix
    x2 = x * x
    y2 = y * y
    z2 = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return np.array([[1 - 2*y2 - 2*z2, 2*xy - 2*wz, 2*xz + 2*wy],
                    [2*xy + 2*wz, 1 - 2*x2 - 2*z2, 2*yz - 2*wx],
                    [2*xz - 2*wy, 2*yz + 2*wx, 1 - 2*x2 - 2*y2]])

def euler_to_rotation_matrix(roll, pitch, yaw):
    '''
    Parameter:
        Euler angles (roll, pitch, yaw) in radians
    Output:
        np.array of rotation matrix
    '''
    c1 = np.cos(roll)
    s1 = np.sin(roll)
    c2 = np.cos(pitch)
    s2 = np.sin(pitch)
    c3 = np.cos(yaw)
    s3 = np.sin(yaw)

    R = np.array([
        [c2 * c3, -c2 * s3, s2],
        [c1 * s3 + c3 * s1 * s2, c1 * c3 - s1 * s2 * s3, -c2 * s1],
        [s1 * s3 - c1 * c3 * s2, c3 * s1 + c1 * s2 * s3, c1 * c2]
    ])

    return R

def quaternion_to_angular_velocity(q, dqdt):
    '''
    Parameter:
        Vector of quaternion q = [w, x, y, z] 
        Differential of q over time dqdt 
    Output:
        Angular velocity vector
    '''
    # Normalize quaternion
    q /= np.linalg.norm(q)

    # Compute rotation matrix
    R = quaternion_to_rotation_matrix(q)
    
    # Compute skew-symmetric matrix
    dRdt = np.array([[0, -dqdt[2], dqdt[1]],
                     [dqdt[2], 0, -dqdt[0]],
                     [-dqdt[1], dqdt[0], 0]])
    S = np.dot(R.T, dRdt)

    # Extract angular velocity vector
    w = np.array([-S[1,2], -S[2,0], -S[0,1]])

    return w

def angle_normalize(angle):
    '''
    normalize angle to -180 < 0 < 180
    '''
    if angle > 180:
        return angle - 360
    elif angle < -180: 
        return angle + 360
    return angle

def angle_offset_compensate(offset, angle):
    '''
    compensate angle offset
    '''
    angle = angle + offset
    if ((offset < 0.0) and (angle < 0.0)):
        return 360.0 + angle
    elif ((offset > 0.0) and (angle > 360.0)):
        return 0.0 + angle - 360.0
    return angle


## -- CONTROLLER-RELATED FUNCTION --------------------------- ##
def fsr_read(raw):
	fsr = (raw-100)/(2000-100) # normalized (0 s.d. 1)
	if fsr > 1: return 1
	elif fsr < 0: return 0
	return fsr

def link1_read(raw):
	val = 0.087890625 * raw # 12-bit data to degree
	val = angle_offset_compensate(0.0, val)
	val = angle_normalize(val)
	return val

def link2_read(raw):
	val = 0.087890625 * raw # 12-bit data to degree
	val = angle_offset_compensate(-68.0, val)
	val = angle_normalize(val)
	return val

def link3_read(raw, ang_1):
	val = 0.087890625 * raw # 12-bit data to degree
	val = angle_offset_compensate(67.0, val)
	val = 90.0 + (val - ang_1) # link 2 angle measurement
	val = angle_normalize(val)
	return val

def link4_read(raw):
	val = -0.087890625 * raw # 12-bit data to degree
	val = angle_offset_compensate(90.0, val)
	val = angle_normalize(val)
	return val

def link5_read(raw):
	val = 0.087890625 * raw # 12-bit data to degree
	val = angle_offset_compensate(-170.0, val)
	val = angle_normalize(val)
	return val

def link6_read(raw):
	val = -0.087890625 * raw # 12-bit data to degree
	val = angle_offset_compensate(168.0, val)
	val = angle_normalize(val)
	return val