#This is the main program for controlling the odometry of the robot
import math
import numpy as np
#import matplotlib.pyplot as plt
import cmath
import random

import kalman
state_estimation = kalman.EKF(0)


class Differential_Drive():
    def __init__(self, drive_node):
        self.drive_node = drive_node

        # Lawnmower parameters
        self._dc = 0.4                   # Distance to chalking mech [m]
        self._r = 0.752/(2*math.pi)      # Wheelradius [m]
        self._track = (43 + 3.2) / 100   # Track width [m]                
        self._L = self._track / 2        # Distance from wheel to centre [m]
        self._PPR = 349

        # Init state
        self._x_init = 0
        self._y_init = 0
        self._theta_init = 0

        # Current state
        self._x = self._x_init
        self._y = self._y_init
        self._theta = self._theta_init

        self._x_chalk = self._x_init - self._dc*math.cos(self._theta_init)
        self._y_chalk = self._y_init - self._dc*math.cos(self._theta_init)

        self._x_error = 0
        self._y_error = 0

        # Previous state
        self._x_prev = 0
        self._y_prev = 0
        self._theta_prev = 0

        self.steering_prev = 0

        self._theta_0_meas_prev = 0
        self._theta_1_meas_prev  = 0

        # Controller
        self._K_theta = 10
        self._K_long = 10


    def update(self, x_ref, y_ref):
        self.state_estimation(x_ref, y_ref)
        long_speed, rot_speed, theta_diff = self.vel_calc()

        wheelspeed0, wheelspeed1 = self.wheelspeeds_calc(long_speed, rot_speed)
        
        speed, steering = self.convert_speed(wheelspeed0, wheelspeed1)

        clamped_speed, clamped_steering = self.clamping(speed, steering, theta_diff)

        self.send_drive(clamped_speed, clamped_steering, self._x_error, self._y_error)


    def state_estimation(self, x_ref, y_ref):

        wheel_0_counter, wheel_1_counter = self.drive_node.get_wheelcounters()
        wheel_0_counter_init, wheel_1_counter_init = self.drive_node.get_wheelcounters_init()

        # Get diff between initial and current wheelcounters
        wheel_0_counter -= wheel_0_counter_init
        wheel_1_counter -= wheel_1_counter_init

        theta_0_meas = wheel_0_counter*2*math.pi/self._PPR*-1
        theta_1_meas = wheel_1_counter*2*math.pi/self._PPR*-1

        delta_theta_0 = theta_0_meas - self._theta_0_meas_prev
        delta_theta_1 = theta_1_meas - self._theta_1_meas_prev

        delta_s = self._r/2*(delta_theta_0+delta_theta_1)
        delta_theta = self._r/(2*self._L)*(delta_theta_0-delta_theta_1)

        self._x += delta_s*math.cos(self._theta_prev)
        self._y += delta_s*math.sin(self._theta_prev)

        self._theta += delta_theta

        self._x_chalk = self._x - self._dc*math.cos(self._theta)
        self._y_chalk = self._y - self._dc*math.sin(self._theta)

        self._x_error = self._x_chalk - x_ref
        self._y_error = self._y_chalk - y_ref

        # Update previous timestep
        self._x_prev = self._x
        self._y_prev = self._y
        self._theta_prev = self._theta

        self._theta_0_meas_prev = theta_0_meas
        self._theta_1_meas_prev = theta_1_meas

        self._x_ref = x_ref
        self._y_ref = y_ref


    def vel_calc(self):
        x = self._x_chalk
        y = self._y_chalk
        theta = self._theta

        x_ref = self._x_ref
        y_ref = self._y_ref

        # Rotational velocity calc
        theta_ref = np.arctan2(y_ref-y, x_ref-x)

        K_theta = self._K_theta

        theta_diff = theta_ref - theta
        rotational_speed = K_theta * theta_diff

        # Longitudonal velocity magnitude calc
        K_long = self._K_long

        long_speed = K_long * np.sqrt((x_ref-x)**2 + (y_ref-y)**2)

        return long_speed, rotational_speed, theta_diff


    def wheelspeeds_calc(self, longitudonal, rotational):
        r = self._r
        track = self._track

        vel = np.array([[longitudonal], [rotational]])
        A = np.array([r/2,      r/2],
                     [-r/track, r/track])
        
        phi_dot = np.linalg.inv(A) @ vel

        wheelspeed0 = phi_dot[0].item()
        wheelspeed1 = phi_dot[1].item()

        return wheelspeed0, wheelspeed1
    

    def convert_speed(self, wheel0, wheel1):
        if wheel1 > wheel0:                         # Left turn
            steering = (wheel1 - wheel0)/wheel1
        else:                                       # Right turn
            steering = (wheel0 - wheel1)/wheel0

            speed = 0.5     # change this to dynamic

            return speed, steering


    def send_drive(self, speed, steering, x_error, y_error):
        time_prev, time = self.drive_node.get_time()

        rate = self.drive_node.get_rate()
        self.drive_node.drive(speed, steering)
        rate.sleep()

        self.steering_prev = steering
       

        return x_error, y_error, self._x_chalk, self._y_chalk, self._theta, time


    def clamping(self, speed, steering, theta_diff):
        speed = round(speed, 2)
        steering = round(steering, 2)

        # Speed magnitude clamping
        if abs(speed) < 0.2:
            speed = 0.2
        elif speed > 1:
            speed = 1
        elif speed < -1:
            speed = -1.0

        diff = 0.1

        # Preventing big jumps in steering input
        if abs(steering - self.steering_prev) > diff:
            if steering < self.steering_prev:
                steering = self.steering_prev - diff
            else:
                steering = self.steering_prev + diff

        # Don't stop and rotate if angle angle error isn't high
        if np.abs(theta_diff) < np.pi/4:
            if steering > 1:
                steering = 0.9
            elif steering < -1:
                steering = -0.9        
        else:
            if steering > 2:
                steering = 2.0
            elif steering < -2:
                steering = -2.0 

        return float(speed), float(steering)