# Kalman filter to estimate position and velocity

import numpy as np

###################################################################
"""
EXTENDED KALMAN FILTER
"""
###################################################################

class EKF():
    def __init__(self, init_state, init_input, init_P, init_noise, init_pos_reading):
        self._state = init_state               # init_state = [x_prev, y_prev, theta_prev]
        self._input = init_input               # init_input = [v_prev, yaw_rate_prev]
        self._P = init_P                       # init_P = [[0.1, 0, 0],[0, 0.1, 0],[0, 0, 0.1]] från exempel
        self._noise = init_noise
        self._Z_k = init_pos_reading
    
    def predict_state(self, input_prev):
        B = self.get_B()
        A = self.get_A()
        state_prev = self.get_state()
        input_prev = self.get_input()
        noise = self.get_noise()

        self.state = A @ state_prev + B @ input_prev + noise
        return self.state
    
    def predict_cov(self):
        F_k = np.eye(3)
        Q_k = np.eye(3)             # kovariansmatris, se anteckningar JUSTERA VID BEHOV

        P = F_k @ self._P @ np.transpose(F_k) + Q_k
        return P
    
    def gain(self):         # steg 4 till 6 
        H_k = np.eye(3)             # justera vid behov
        R_k = np.eye(3)             # justera vid behov
        P_k = self.predict_cov()
        Z_k = self.get_pos_reading()                   # mätvärden [x_k, y_k, theta_k]
        X_k = self.predict_state()
        sensor_error = self.get_sensor_error()

        y_k = Z_k - H_k @ X_k + sensor_error

        S_k = H_k @ P_k @ np.transpose(H_k) + R_k

        K_k = P_k @ np.transpose(H_k) @ np.linalg.inv(S_k)
        return K_k

    def get_B(self, theta_prev, dk):
        return np.array([[np.cos(theta_prev) * dk, 0],
                        [np.sin(theta_prev) * dk, 0],
                        [0,                      dk]])
    
    def update(self):
        # TODO
        # steg 7-8
        # uppdatera state och cov
        return None

    def pos_update(self):
        # TODO
        # Hämta positionsmätningar och uppdatera Z_k
        self._Z_k = ...

    def get_A(self):
        return np.eye(3)
    
    def get_state(self):
        return self._state
    
    def get_input(self):
        return self._input
    
    def get_noise(self):
        return self._noise
    
    def get_pos_reading(self):
        return self._Z_k
    
    def get_sensor_error(self):
        # TODO
        # hämta sensorerror fån ROS2
        error = ...
        return error


###################################################################
"""
EXTENDED KALMAN FILTER
Förra årets
"""
###################################################################

class old_EKF:
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
        A = np.eye(3)
        A[0, 2] = -dt * self.input[0] * np.sin(theta)
        A[1, 2] = dt * self.input[0] * np.cos(theta)
        self.P = A @ self.P @ A.T + self.Q

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
    
