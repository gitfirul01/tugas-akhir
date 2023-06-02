import numpy as np

# Kalman filter parameters
A = np.array([[1]])  # State transition matrix
B = np.array([[1]])  # Control input matrix
H = np.array([[1]])  # Observation matrix
Q = np.array([[0.01]])  # Process noise covariance
R = np.array([[0.1]])  # Measurement noise covariance

# Initial state
x0 = np.array([[0]])  # Initial state estimate
P0 = np.array([[1]])  # Initial error covariance

# Initialize variables
x_hat = x0  # Updated state estimate
P = P0  # Updated error covariance

# Simulated measurement data
measurements = [1, 2, 3, 4, 5, 10, 7, 8, 9]

# Kalman filter loop
for z in measurements:
    # Predict step
    x_hat_minus = A.dot(x_hat)  # Predicted state estimate
    P_minus = A.dot(P).dot(A.T) + Q  # Predicted error covariance

    # Update step
    K = P_minus.dot(H.T).dot(np.linalg.inv(H.dot(P_minus).dot(H.T) + R))  # Kalman gain
    x_hat = x_hat_minus + K.dot(z - H.dot(x_hat_minus))  # Updated state estimate
    P = (np.eye(x0.shape[0]) - K.dot(H)).dot(P_minus)  # Updated error covariance

    # Print the filtered estimate
    print("Filtered estimate:", x_hat.flatten())
