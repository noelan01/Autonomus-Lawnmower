# Modellering av system. Står för regleringen

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
import numpy as np
import matplotlib.pyplot as plt
import cmath

class System_model():
    def __init__(self, x_init, y_init):
        
        # init values
        self._x_init = x_init
        self._y_init = y_init
        self._d = 0.4           # distance between track and kritningsmekanism
        self._r = 0.751/(2*np.pi)
        self._l = (43/2+3.2/2)/100
        
        # prev timestep
        self._x_prev = 0
        self._y_prev = 0
        self._theta_prev = 0
        
        # current timestep
        self._x = 0
        self._y = 0
        self._theta = 0
        
        # reference
        self._x_ref = 0
        self._y_ref = 0
        
        self._error = [0,0]
    
    # 
    def update(self):
        # 1. Update error
        self.error_update()
        
        # 2. PID gain
        x_reg = PID.controler(self._error[0])
        y_reg = PID.controler(self._error[1])
        
        # 3. prepare control input
        self.control_input_prep(x_reg, y_reg)
        
    
    def error_update(self):
        x_error = self._x_ref - self._x_prev
        y_error = self._y_ref - self._y_prev
        self._error = [x_error, y_error]
    
        
    def control_input_prep(self, x_reg, y_reg):
        theta_prev = self._theta_prev
        d = self._d
        r = self._r
        l = self._l
        
        # 1
        delta_omega = cmath.asin((x_reg * cmath.sin(theta_prev) - y_reg * cmath.cos(theta_prev)) / d).real  #asin((u(1)*sin(u(3))-u(2)*cos(u(3)))/D)
        # 2
        delta_S_k = d * cmath.cos(delta_omega) - d + x_reg * cmath.cos(theta_prev) + y_reg * cmath.sin(theta_prev)   # D*cos(u(1))-D+u(2)*cos(u(4))+u(3)*sin(u(4))

        wheelspeed1 = (1/r) * (delta_S_k + l * delta_omega) # 1/r*(u(2)+L*u(1))
        wheelspeed2 = (1/r) * (delta_S_k - l * delta_omega) # 1/r*(u(2)-L*u(1))

class PID():
    def __init__(self, Kp, Ki, Kd, setpoint, measurement):
        self._Kp= Kp
        self._Ki = Ki
        self._Kd = Kd
        self._setpoint = setpoint
        self._measurement = measurement
    
    def controler(self, error):
        # Sriv PID kontroller här
        pass
    