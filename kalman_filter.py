import numpy as np

KF_A = np.array([[1]])  # State transition matrix
KF_H = np.array([[1]])  # Measurement matrix
KF_Q = np.array([[0.0001]])  # Process noise covariance
KF_R = np.array([[0.1]])  # Measurement noise covariance
KF_P = np.array([[1]])  # Estimate covariance
KF_x = np.array([[0]])  # Initial state estimate

def kalman_filter(z, x, P):
    # Prediction Step
    x_pred = KF_A @ x
    P_pred = KF_A @ P @ KF_A.T + KF_Q
    
    # Update Step
    K = P_pred @ KF_H.T @ np.linalg.inv(KF_H @ P_pred @ KF_H.T + KF_R)  # Kalman Gain
    x_new = x_pred + K @ (z - KF_H @ x_pred)
    P_new = (np.eye(1) - K @ KF_H) @ P_pred
    
    return x_new, P_new