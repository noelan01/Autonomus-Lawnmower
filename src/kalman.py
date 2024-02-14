# Kalman filter to estimate position and velocity

import numpy as np
np.set_printoptions(precision=3,suppress=True)

########################################
"""
REGULAR KALMAN FILTER
not adjusted to our model
"""
########################################

class Kalman_filter():
    def __init__(self) -> None:
        pass

    def kf_predict(X, P, A, Q, B, U):
        X = np.dot(A, X) + np.dot(B, U)
        P = np.dot(A, np.dot(P, A.T)) + Q
        return(X,P)


    def kf_update(X, P, Y, H, R):
        IM = np.dot(H, X)
        IS = R + np.dot(H, np.dot(P, H.T))
        K = np.dot(P, np.dot(H.T, np.linalg.inv(IS)))
        X = X + np.dot(K, (Y-IM))
        P = P - np.dot(K, np.dot(IS, K.T))
        LH = gauss_pdf(Y, IM, IS)
        return (X,P,K,IM,IS,LH)


    def gauss_pdf(X, M, S):
        if M.shape()[1] == 1:
            DX = X - np.tile(M, X.shape()[1])
            E = 0.5 * sum(DX * (np.dot(np.linalg.inv(S), DX)), axis=0)
            E = E + 0.5 * M.shape()[0] * np.log(2 * np.pi) + 0.5 * np.log(np.linalg.det(S))
            P = np.exp(-E)

        elif X.shape()[1] == 1:
            DX = np.tile(X, M.shape()[1])- M
            E = 0.5 * sum(DX * (np.dot(np.linalg.inv(S), DX)), axis=0)
            E = E + 0.5 * M.shape()[0] * np.log(2 * np.pi) + 0.5 * np.log(np.linalg.det(S))
            P = np.exp(-E)

        else:
            DX = X-M
            E = 0.5 * np.dot(DX.T, np.dot(np.linalg.inv(S), DX))
            E = E + 0.5 * M.shape()[0] * np.log(2 * np.pi) + 0.5 * np.log(np.linalg.det(S))
            P = np.exp(-E)

        return (P[0],E[0])
    


###################################################################
"""
EXTENDED KALMAN FILTER
Förra årets
"""
###################################################################

class EKF:
    def __init__(self, initial_state, initial_input, initial_covariance, process_noise):
        self.state = initial_state
        self.input = initial_input
        self.P = initial_covariance
        self.Q = process_noise

    def get_B(self, theta, dt):
        B = np.array([[dt * np.cos(theta), 0],
                      [dt * np.sin(theta), 0],
                      [0, dt]])
        return B

    def predict(self, delta_x, delta_y, dt):
        theta = self.state[2]
        B = self.get_B(theta, dt)
        self.state += B @ self.input
        F = np.eye(3)
        F[0, 2] = -dt * self.input[0] * np.sin(theta)
        F[1, 2] = dt * self.input[0] * np.cos(theta)
        self.P = F @ self.P @ F.T + self.Q

    def get_state(self):
        return self.state

    def update_gps(self, gps_x, gps_y, measurement_noise):
        z = np.array([gps_x, gps_y])
        H = np.array([[1, 0, 0],
                      [0, 1, 0]])
        R = measurement_noise
        self.update(z, H, R)

    def update(self, z, H, R):
        y = z - H @ self.state
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.state += K @ y
        self.P = (np.eye(3) - K @ H) @ self.P