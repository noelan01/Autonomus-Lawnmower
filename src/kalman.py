import numpy as np
#import node_lawnmower_control

#drive_node = node_lawnmower_control.Lawnmower_Control()

"""# Endast för simulering
import sys
sys.path.append('/home/noelan/chalmers/kandidatarbete/Autonomus-Lawnmower/tests')
import sub_data
sim_data = sub_data.Get_data()
#
"""
###################################################################
"""
EXTENDED sKALMAN FILTER

Nu görs state estimationen med GPS data enbart samt predictions baserat på
control inputs. Skapa sensor fusion function?

https://www.youtube.com/watch?v=whSw42XddsU&ab_channel=BrianDouglas
"""
###################################################################

class EKF():
    def __init__(self, sim):
        # init
        self._state = np.array([[0],[0],[0]])   
        self._input = np.array([[0],[0]])
        self._noise = 0
        self._time = 0
        self._Z_k = np.array([[0],[0],[0]])
        self._sensor_error = np.array([[0],[0],[0]])

        # previous
        self._Z_k_prev = np.array([[0],[0],[0]])
        self._input_prev = np.array([[0],[0]])
        self._prev_time = 0
        
        # other
        self._A = np.eye(3)
        self._B = None
        self._K_k = None
        self._y_k = None        # diff mellan mätvärden och predikterade mätvärden
        
        self._P = np.array([[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]])     # Predikterad kovarians matris av state estimering  JUSTERA
        self._H_k = np.eye(3)   # Mätningsmatris
        self._R_k = np.eye(3)   # Sensorbrus kovarians matris       justera vid behov
        self._F_k = np.eye(3)   # Funkar som A matrisen
        self._Q_k = np.eye(3)   # kovariansmatris INIT, sätt till covarianser med setter om de behövs

        # 1 if simulation. 0
        self._sim = sim
    

    """
    Update funtionen uppdaterar alla delar i kalmanfiltret.
    Kalla på denna för att få den uppdaterade state estimationen.
    """    
    def update(self, Z_k, control_input, sensor_error):
        self._Z_k_prev = self._Z_k
        self._Z_k = Z_k
        self._input_prev = self._input
        self._input = control_input
        self._sensor_error = sensor_error
        
        self.predict_state()            # 1. predict state
        self.predict_cov()              # 2. predict cov
        self.set_gain()                 # 3. - 5. set optimal gain


        state_prev = self._state   # 6. & 7. Updatera state och cov
        K_k = self._K_k
        y_k = self._y_k
        P_prev = self._P
        H_k = self._H_k

        self._state = state_prev + K_k @ y_k        # uppdatera state
        
        self._P = (np.eye(3) - K_k @ H_k) @ P_prev  # uppdatera cov

        return self._state                     # returna uppdaterat state
    

    ######   ######   #######   #######    ######    #######    ######
    #        #           #         #       #         #     #    #
    ######   ###         #         #       ###       #######    ######
         #   #           #         #       #         # ##            #
    ######   ######      #         #       ######    #   ##     ######
        

    def predict_state(self):
        self.set_B()
        B = self._B
        A = self._A
        state_prev = self._state

        input_prev = self._input_prev

        noise = self._noise

        self._state = A @ state_prev + B @ input_prev + noise
    

    def predict_cov(self):
        F_k = self._F_k
        self.set_Q_k()
        Q_k = self._Q_k
        P_prev = self._P

        self._P = F_k @ P_prev @ np.transpose(F_k) + Q_k

    
    def set_gain(self): 
        H_k = self._H_k
        R_k = self._R_k

        P_k = self._P

        Z_k = self._Z_k                     # mätvärden [[x_k], [y_k], [theta_k]]
        
        X_k = self._state

        
        sensor_error = self._sensor_error

        self._y_k = Z_k - H_k @ X_k + sensor_error

        S_k = H_k @ P_k @ np.transpose(H_k) + R_k

        self._K_k = P_k @ np.transpose(H_k) @ np.linalg.inv(S_k)
    

    def set_B(self):
        theta_prev = self._Z_k[2][0]
        #dk = self.get_dk()
        dk = 0.025

        self._B = np.array([[np.cos(theta_prev) * dk, 0],
                        [np.sin(theta_prev) * dk, 0],
                        [0,                      dk]])
        
    def set_Q_k(self):
        pos = self._Z_k
        prev_pos = self._Z_k_prev

        x = pos[0][0]
        y = pos[1][0]
        theta = pos[2][0]
        
        x_prev = prev_pos[0][0]
        y_prev = prev_pos[1][0]
        theta_prev = prev_pos[2][0]
        
        """
        Q_k = np.array([[np.cov([x,x]).item(), np.cov([x,y]).item(), np.cov([x,theta]).item()],
                        [np.cov([y,x]).item(), np.cov([y,y]).item(), np.cov([y,theta]).item()],
                        [np.cov([theta,x]).item(), np.cov([theta,y]).item(), np.cov([theta,theta]).item()]])
        """
        Q_k = np.array([[np.cov([x_prev,x]).item(),     10e-6,                          0],
                        [10e-6,                             np.cov([y_prev,y]).item(),  0],
                        [0,                             0,                          np.cov([theta_prev,theta]).item()]])
        self._Q_k = Q_k
        
    """
    def update_time(self):
        self._prev_time = self.get_time()
        if self._sim:
            self._time = sim_data.update_time()
        else:
            pass    # GET FROM ROS
    """

    """
    def update_sensor_error(self):
        if self._sim:
            self._sensor_error = sim_data.get_sensor_error()
        else:
            pass"""
        


    ######   ######   #######   #######    ######    #######    ######
    #        #           #         #       #         #     #    #
    #  ###   ###         #         #       ###       #######    ######
    #    #   #           #         #       #         # ##            #
    ######   ######      #         #       ######    #   ##     ######
        
    def get_state(self):
        return self._state
    
    """
    def get_dk(self):
        prev_time = self._prev_time
        time = self._time

        dk = abs(prev_time - time)
        return dk
    """

    










    
    




###################################################################
"""
EXTENDED KALMAN FILTER
Förra årets
"""
###################################################################
"""
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

"""