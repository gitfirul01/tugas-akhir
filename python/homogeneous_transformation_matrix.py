import numpy as np

def homogeneous_transformation_matrix(phi, psi, theta, Px, Py, Pz):
    # Rx(phi), Ry(psi), Rz(theta)
    phi = np.deg2rad(phi)
    psi = np.deg2rad(psi)
    theta = np.deg2rad(theta)
    Rx = np.array([
        [1, 0, 0, 0],
        [0, np.cos(phi), -np.sin(phi), 0],
        [0, np.sin(phi), np.cos(phi), 0],
        [0, 0, 0, 1]
        ])
    Ry = np.array([
        [np.cos(psi), 0, np.sin(psi), 0],
        [0, 1, 0, 0],
        [-np.sin(psi), 0, np.cos(psi), 0],
        [0, 0, 0, 1]
        ])
    Rz = np.array([
        [np.cos(theta), -np.sin(theta), 0, 0],
        [np.sin(theta), np.cos(theta), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
        ])
    R = Rz@Ry@Rx
    T = np.array([
        [1, 0, 0, Px], 
        [0, 1, 0, Py], 
        [0, 0, 1, Pz],
        [0, 0, 0, 1 ]
    ])
    return T @ R
