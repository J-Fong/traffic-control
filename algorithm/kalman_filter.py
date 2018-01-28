import math
import numpy as np

def kalman_predict(x0, y0, x, y, state, P):
    # Four dimensional state: position_x, position_y, velocity_x, velocity_y
    # Covariance parameters
    Sigma_M = np.matrix([[0.5, 0, 0, 0], [0, 0.5, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    Sigma_O = np.matrix([[0.5, 0], [0, 0.5]])

    """
    # Check if the first time running this function
    if previous_t < 0
        state = [x, y, 0, 0];
        P = 0.1 * eye(4);
        predictx = x;
        predicty = y;
        return;
    end
    """

    # Kalman filter updates
    dt = 0.04 # Webcam FPS
    # System model
    A = np.matrix([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]])
    # Measurement model
    C = np.matrix([[1, 0, 0, 0], [0, 1, 0, 0]])
    # Read measurement
    z = np.matrix([x], [y]])
    # Predicted covariance estimate 
    P = np.dot(np.dot(A, P), np.transpose(A)) + Sigma_M
    # Residual variance
    R = np.dot(np.dot(C, P), np.transpose(C)) + Sigma_O
    # Kalman gain
    K = np.dot(np.dot(P, np.transpose(C)), numpy.linalg.inv(R))
    # Update state estimate
    state = np.dot(A, np.transpose(state)) + np.dot(K, (z - np.dot(np.dot(C, A), np.transpose(state)))';
    # Update covariance estimate
    P = P - K * C * P;
    # Predict 330ms into the future
    predictx = state(1) + state(3) * 0.330;
    predicty = state(2) + state(4) * 0.330;
end
