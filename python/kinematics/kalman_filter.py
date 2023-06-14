import numpy as np

class kalman_filter():
    def __init__(self, A = [[1]], B = [[1]], H = [[1]], Q = [[0.01]], R = [[0.1]], x0 = [[0]], P0 = [[1]]):
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
        self.P = self.P0  # Updated error covariance
        self.x_hat_minus = None # (n-vector) predicted state estimate
        self.P_minus = None # Predicted error covariance
        self.K = None # (cons) kalman gain

    def run(self, z):
        ## Predictor step
        self.x_hat_minus = self.A.dot(self.x_hat) # no control variable
        self.P_minus = self.A.dot(self.P).dot(self.A.T) + self.Q
        ## Corrector step
        self.K = self.P_minus.dot(self.H.T).dot(np.linalg.inv(self.H.dot(self.P_minus).dot(self.H.T) + self.R))
        self.x_hat = self.x_hat_minus + self.K.dot(z - self.H.dot(self.x_hat_minus))
        self.P = (np.eye(self.x0.shape[0]) - self.K.dot(self.H)).dot(self.P_minus)

        return self.x_hat.flatten()

# ## Simulated measurement data
# measurements = [1, 2, 3, 4, 5, 10, 7, 8, 9]
# kf = kalman_filter()

# for z in measurements:
#     print(kf.run(z))
