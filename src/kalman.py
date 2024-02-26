import numpy as np

# Endast för simulering
import sys
sys.path.append('/home/noelan/chalmers/kandidatarbete/Autonomus-Lawnmower/tests')
import sub_data
sim_data = sub_data.Get_data()
#

###################################################################
"""
EXTENDED sKALMAN FILTER

Nu görs state estimationen med GPS data enbart samt predictions baserat på
control inputs. Skapa sensor fusion function?

https://www.youtube.com/watch?v=whSw42XddsU&ab_channel=BrianDouglas
"""
###################################################################

class EKF():
    def __init__(self, init_state, init_input, init_noise, init_pos_reading, sim):
        self._state = init_state               # init_state = [x_prev, y_prev, theta_prev]
        self._input = init_input               # init_input = [v_prev, yaw_rate_prev]
        self._noise = init_noise
        self._Z_k = init_pos_reading
        self._time = 0
        self._prev_time = 0
        self._sensor_error = np.array([[0],[0],[0]])

        self._A = np.eye(3)
        self._B = None
        self._K_k = None
        self._y_k = None        # diff mellan mätvärden och predikterade mätvärden
        
        self._P = np.array([[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]])     # Predikterad kovarians matris av state estimering  JUSTERA
        self._H_k = np.eye(3)   # Mätningsmatris
        self._R_k = np.eye(3)   # Sensorbrus kovarians matris       justera vid behov
        self._F_k = np.eye(3)   # Funkar som A matrisen
        self._Q_k = np.eye(3)   # kovariansmatris INIT, sätt till covarianser med setter om de behövs

        self._sim = sim
    

    """
    Update funtionen uppdaterar alla delar i kalmanfiltret.
    Kalla på denna för att få den uppdaterade state estimationen.
    """    
    def update(self):
        self.predict_state()            # 1. predict state
        self.predict_cov()              # 2. predict cov
        self.set_gain()                 # 3. - 5. set optimal gain


        state_prev = self.get_state()   # 6. & 7. Updatera state och cov
        K_k = self.get_K_k()
        y_k = self.get_y_k()
        P_prev = self.get_cov()
        H_k = self.get_H_k()

        #print("K_k = ", K_k)
        #print("y_k = ", y_k)
        #print("K_k * y_k = ",K_k @ y_k)

        self._state = state_prev + K_k @ y_k        # uppdatera state
        
        self._P = (np.eye(3) - K_k @ H_k) @ P_prev  # uppdatera cov

        return self.get_state()                     # returna uppdaterat state
    

    ######   ######   #######   #######    ######    #######    ######
    #        #           #         #       #         #     #    #
    ######   ###         #         #       ###       #######    ######
         #   #           #         #       #         # ##            #
    ######   ######      #         #       ######    #   ##     ######
        

    def predict_state(self):
        self.set_B()
        B = self.get_B()
        A = self.get_A()
        state_prev = self.get_state()

        self.update_input()
        input_prev = self.get_input()

        noise = self.get_noise()

        #print("A = ", A)
        #print("state_prev = ", state_prev)
        #print("B = ", B)
        #print("input_prev = ", input_prev)
        #print("noise = ", noise)

        self._state = A @ state_prev + B @ input_prev + noise
    

    def predict_cov(self):
        F_k = self.get_F_k()
        self.set_Q_k()
        Q_k = self.get_Q_k()
        P_prev = self.get_P()

        self._P = F_k @ P_prev @ np.transpose(F_k) + Q_k

    
    def set_gain(self): 
        H_k = self.get_H_k()
        R_k = self.get_R_k()

        P_k = self.get_cov()

        self.set_pos_update()
        Z_k = self.get_pos_reading()                   # mätvärden [[x_k], [y_k], [theta_k]]
        
        X_k = self.get_state()

        self.update_sensor_error()
        sensor_error = self.get_sensor_error()

        #print("Z_k =", Z_k)
        #print("H_k =", H_k)
        #print("X_k =", X_k)
        #print("sensor_error =", sensor_error)
        self._y_k = Z_k - H_k @ X_k + sensor_error

        S_k = H_k @ P_k @ np.transpose(H_k) + R_k

        self._K_k = P_k @ np.transpose(H_k) @ np.linalg.inv(S_k)
    

    def set_pos_update(self):
        # TODO
        # Hämta positionsmätningar och uppdatera Z_k (yaw från imu?)
        # ENDAST GPS? ELLER KOMBINERA SENSORER??

        rtk_available = False        # checka om RTK tillgänglig

        if rtk_available:           # väljer RTK om tillgänglig
            self._Z_k = ...
        else:                               # annars GNSS
            if self._sim:
                self._time, self._Z_k = sim_data.get_pos_reading()
            else:
                pass    # GET FROM ROS
        

    def set_B(self):
        theta_prev = self.get_theta()
        dk = self.get_dk()

        self._B = np.array([[np.cos(theta_prev) * dk, 0],
                        [np.sin(theta_prev) * dk, 0],
                        [0,                      dk]])
        
    def set_Q_k(self):
        self.set_pos_update()
        pos = self.get_pos_reading()

        x = pos[0][0]
        y = pos[1][0]
        theta = pos[2][0]

        self._Q_k = np.array([[np.cov([x,x]).item(), np.cov([x,y]).item(), np.cov([x,theta]).item()],
                        [np.cov([y,x]).item(), np.cov([y,y]).item(), np.cov([y,theta]).item()],
                        [np.cov([theta,x]).item(), np.cov([theta,y]).item(), np.cov([theta,theta]).item()]])
        #print("Q_k = ",self._Q_k)
        
    """
    def update_time(self):
        self._prev_time = self.get_time()
        if self._sim:
            self._time = sim_data.update_time()
        else:
            pass    # GET FROM ROS
    """
    def update_input(self):
        if self._sim:
            self._input = sim_data.get_control_input()
        else:
            pass    # GET FROM ROS

    def update_sensor_error(self):
        if self._sim:
            self._sensor_error = sim_data.get_sensor_error()
        else:
            pass    # GET FROM ROS
        


    ######   ######   #######   #######    ######    #######    ######
    #        #           #         #       #         #     #    #
    #  ###   ###         #         #       ###       #######    ######
    #    #   #           #         #       #         # ##            #
    ######   ######      #         #       ######    #   ##     ######
        
    def get_A(self):
        return self._A
    
    def get_B(self):
        return self._B
    
    def get_P(self):
        return self._P
    
    def get_K_k(self):
        return self._K_k
    
    def get_state(self):
        return self._state
    
    def get_cov(self):
        return self._P
    
    def get_input(self):
        return self._input
    
    def get_y_k(self):
        return self._y_k
    
    def get_noise(self):
        return self._noise
    
    def get_pos_reading(self):
        return self._Z_k
    
    def get_H_k(self):
        return self._H_k
    
    def get_R_k(self):
        return self._R_k
    
    def get_F_k(self):
        return self._F_k
    
    def get_Q_k(self):
        return self._Q_k
    
    def get_dk(self):
        prev_time = self.get_prev_time()
        time = self.get_time()

        dk = abs(prev_time - time)
        return dk
    
    def get_theta(self):
        self.set_pos_update()
        return self._Z_k[2][0]
    
    def get_sensor_error(self):
        return self._sensor_error
    
    def get_time(self):
        return self._time
    
    def get_prev_time(self):
        return self._prev_time
    
    




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