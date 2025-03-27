# import numpy as np

# KF_A = np.array([[1, 0.01], [0, 1]])  # State transition matrix
# KF_H = np.array([[0, 1]])  # Measurement matrix
# KF_Q = np.array([[0.0001, 0], [0, 0.01]])  # Process noise covariance
# KF_R = np.array([[0.1]])  # Measurement noise covariance
# KF_P = np.array([[1, 0], [0, 1]])  # Estimate covariance
# KF_x = np.array([[0, 0, 0, 0, 0, 0]])  # Initial state estimate

# def kalman_filter(z, x, P):
#     # Prediction Step
#     x_pred = KF_A @ x
#     P_pred = KF_A @ P @ KF_A.T + KF_Q
    
#     # Update Step
#     K = P_pred @ KF_H.T @ np.linalg.inv(KF_H @ P_pred @ KF_H.T + KF_R)  # Kalman Gain
#     x_new = x_pred + K @ (z - KF_H @ x_pred)
#     P_new = (np.eye(1) - K @ KF_H) @ P_pred
    
#     return x_new, P_new

import numpy as np

# Time step
dt = 0.01  # 100 Hz

# State transition matrix (6x6 for 3D tracking)
KF_A = np.array([
    [1, dt, 0,  0,  0,  0],
    [0,  1, 0,  0,  0,  0],
    [0,  0, 1, dt,  0,  0],
    [0,  0, 0,  1,  0,  0],
    [0,  0, 0,  0,  1, dt],
    [0,  0, 0,  0,  0,  1]
])

# Measurement matrix (maps velocity)
KF_H = np.array([
    [0, 1, 0, 0, 0, 0],
    [0, 0, 0, 1, 0, 0],
    [0, 0, 0, 0, 0, 1]
])

# Process noise covariance (6x6)
KF_Q = np.array([
    [0.0001, 0,      0,      0,      0,      0],
    [0,      0.01,   0,      0,      0,      0],
    [0,      0,      0.0001, 0,      0,      0],
    [0,      0,      0,      0.01,   0,      0],
    [0,      0,      0,      0,      0.0001, 0],
    [0,      0,      0,      0,      0,      0.01]
])

# Measurement noise covariance (3x3, assuming independent noise per axis)
KF_R = np.array([
    [0.1,  0,    0],
    [0,    0.1,  0],
    [0,    0,    0.1]
])

# Estimate covariance (6x6)
KF_P = np.eye(6)

# Initial state estimate (position=0, velocity=0 in x, y, z)
KF_x = np.zeros((6, 1))

def kalman_filter(z, x, P):
    """
    Kalman Filter update step for 3D position tracking.
    :param z: 3x1 measurement vector (acceleration in x, y, z)
    :param x: 6x1 state vector [x_pos, x_vel, y_pos, y_vel, z_pos, z_vel]
    :param P: 6x6 covariance matrix
    :return: Updated state (x) and covariance (P)
    """
    
    # Prediction Step
    x_pred = KF_A @ x
    P_pred = KF_A @ P @ KF_A.T + KF_Q

    # Kalman Gain
    S = KF_H @ P_pred @ KF_H.T + KF_R
    K = P_pred @ KF_H.T @ np.linalg.inv(S)

    # Measurement Update
    x_new = x_pred + K @ (z - KF_H @ x_pred)
    P_new = (np.eye(6) - K @ KF_H) @ P_pred

    return x_new, P_new
