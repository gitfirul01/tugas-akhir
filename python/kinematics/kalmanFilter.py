import numpy as np

class KalmanFilter:
    def __init__(self, A = [[1]], B = [[1]], H = [[1]], Q = [[0.1]], R = [[0.1]], x0 = [[0]], P0 = [[1]]):
        ## Kalman filter parameters
        self.A = np.array(A)  # (n x n) state transition matrix
        self.B = np.array(B)  # (n x l) control input matrix
        self.H = np.array(H)  # (m x n) observation matrix
        self.Q = np.array(Q)  # Process noise covariance
        self.R = np.array(R)  # Measurement noise covariance

        ## Initial state
        self.x0 = np.array(x0)  # (n-vector) initial state estimate
        self.P0 = np.array(P0)  # Initial error covariance
        
        ## Initialize variables
        self.x_hat = self.x0  # (n-vector) updated state estimate
        self.x_hat_minus = None # (n-vector) predicted state estimate
        self.P = self.P0  # Updated error covariance
        self.P_minus = None # Predicted error covariance

    def step(self, z, mode = 'normal'):
        ## Predict step
        self.x_hat_minus = self.A.dot(self.x_hat) # no control variable
        self.P_minus = self.A.dot(self.P).dot(self.A.T) + self.Q
        ## Update step
        if mode == 'circural':
            residual = np.deg2rad(z) - self.H.dot(self.x_hat_minus)
            residual = (residual + np.pi) % (2 * np.pi) - np.pi
        else:
            residual = z - self.H.dot(self.x_hat_minus)

        S = self.H.dot(self.P_minus).dot(self.H.T) + self.R
        K = self.P_minus.dot(self.H.T).dot(np.linalg.inv(S))

        self.x_hat = self.x_hat_minus + K.dot(residual)
        self.P = (np.eye(self.x0.shape[0]) - K.dot(self.H)).dot(self.P_minus)

        if mode == 'circural':
            return np.rad2deg(self.x_hat[0][0])
        else:
            return self.x_hat[0][0]



# data1 = [1, 2, 3, 4, 5, 10, 7, 8, 9]
# data2 = [175, 176, 177, 178, 179, 180, -179, -178, -177, -176]
# kf = KalmanFilter()
# for z in data1:
#     print(kf.step(z))
# for z in data2:
#     print(kf.step(z, mode='circural'))
