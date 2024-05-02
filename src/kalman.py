import numpy as np

###################################################################
"""
EXTENDED sKALMAN FILTER
"""
###################################################################

class EKF():
    def __init__(self, sim, theta_init):
        # init
        self._state = np.array([[0],[0],[theta_init]]) # x, y, theta   
        self._input = np.array([[0],[0]])   # control input
        self._noise = 0
        self._time = 0
        self._Z_k = np.array([[0],[0],[0]]) # measured state
        self._sensor_error = np.array([[0],[0],[0]])    # sensor error in measurements

        # previous
        self._Z_k_prev = np.array([[0],[0],[0]])
        self._input_prev = np.array([[0],[0]])
        self._prev_time = 0
        
        # other
        self._A = np.eye(3)
        self._B = None
        self._K_k = None
        self._y_k = None        # diff between measurements and predicted
        
        self._P = np.array([[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]])     # Predicted cov matrix
        self._H_k = np.eye(3)   # Measurement matrix
        self._R_k = np.eye(3) * 0.9  # Sensor error cov matrix
        self._F_k = np.eye(3)  
        self._Q_k = np.eye(3)   # cov matrix init
        self._dk = 0.025

        # 1 if simulation. 0
        self._sim = sim
    

    """
    The update function updates makes all steps to update the state.
    Call this function in the code to get the updated state.
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


        state_prev = self._state   # 6. & 7. Update state and cov
        K_k = self._K_k
        y_k = self._y_k
        P_prev = self._P
        H_k = self._H_k

        self._state = state_prev + K_k @ y_k        # update state
        
        self._P = (np.eye(3) - K_k @ H_k) @ P_prev  # update cov

        return self._state                     
    

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

        Z_k = self._Z_k                     # measured [[x_k], [y_k], [theta_k]]
        
        X_k = self._state

        
        sensor_error = self._sensor_error

        self._y_k = Z_k - H_k @ X_k + sensor_error

        S_k = H_k @ P_k @ np.transpose(H_k) + R_k

        self._K_k = P_k @ np.transpose(H_k) @ np.linalg.inv(S_k)
    

    def set_B(self):
        theta_prev = self._Z_k[2][0]
        
        dk = self._dk

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
        Q_k = np.array([[np.cov([x_prev,x]).item(),     10e-6,                          0],
                        [10e-6,                             np.cov([y_prev,y]).item(),  0],
                        [0,                             0,                          np.cov([theta_prev,theta]).item()]])
        """
        Q_k = np.array([[1, 0, 0],
                        [0, 1,  0],
                        [0, 0, 1]]) * 0.05

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
