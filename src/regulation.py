#This is the main program for controlling the odometry of the robot
import math
import numpy as np
#import matplotlib.pyplot as plt
import cmath
import random
import rclpy

import kalman
skalman = kalman.EKF(0)

import node_lawnmower_control 
import coord_transformation
#import calibration_offset

class Regulation():
    def __init__(self, drive_node):
        self.drive_node = drive_node

        #Starting with defining variables for the robot
        self.D = 0.4
        self.r = 0.752/(2*math.pi)
        self.L = (43/2+3.2/2)/100
        
        #Other variables
        self.x = 0
        self.y = 0
        self.x_kalman = 0
        self.y_kalman = 0

        self.theta = - np.pi

        self.x_base = 0
        self.y_base = 0
        self.x_base_kalman = 0
        self.y_base_kalman = 0
        self.theta_kalman = 0

        # PID PARAMETERS
        # x
        self.Kp_x = 5
        self.Ki_x = 0
        self.Kd_x = 0

        # y
        self.Kp_y = 10
        self.Ki_y = 5
        self.Kd_y = 0.001

        #Put the sample time to the same as the update time of the drive publish node?
        self.Ts = 1/self.drive_node.get_updaterate()
        self.acc_sum_delta_omega_1 = 0
        self.acc_sum_delta_omega_2 = 0    

        self.lin_vel = 0
        self.ang_vel = 0
        self.err_sum_x = 0
        self.err_sum_y = 0

        self.delta_xe_old = 0
        self.delta_ye_old = 0

        #This angle should be set based on which coordinate we are moving towards
        self.theta_old = self.theta
        self.theta_0_meas_old = 0
        self.theta_1_meas_old = 0


        self.PPR = 349

        self.steering_prev = 0


    def update(self, x_ref, y_ref):

        x_ref = x_ref
        y_ref = y_ref
        
        #Implementing the kinematic model of the robot
        delta_xe = x_ref - self.x
        delta_ye = y_ref - self.y

        self.err_sum_x = self.err_sum_x + delta_xe
        self.err_sum_y = self.err_sum_y + delta_ye 

        #PID regulator

        delta_x = self.PID(delta_xe, self.Kp_x, self.Ki_x, self.Kd_x)
        delta_y = self.PID(delta_ye, self.Kp_y, self.Ki_y, self.Kd_y)

        #delta_x = delta_xe*self.Kp+self.Ki*self.Ts*self.err_sum_x+self.Kd*(delta_xe-self.delta_xe_old)/self.Ts
        #delta_y = delta_ye*self.Kp+self.Ki*self.Ts*self.err_sum_y+self.Kd*(delta_ye-self.delta_ye_old)/self.Ts
        
        #print("PID: X: ", delta_x, "Y: ", delta_y)
        print("error sum x: ", self.err_sum_x, "error sum y: ", self.err_sum_y)
        
        #Calculating delta_omega(k) and delta_S(k)
        delta_omega = cmath.asin((delta_x*math.sin(self.theta_old)-delta_y*math.cos(self.theta_old))/self.D).real
        delta_S = self.D*math.cos(delta_omega)+self.D+delta_x*math.cos(self.theta_old)+delta_y*math.sin(self.theta_old)

        #Calculating delta_omega0(k) and delta_omega1(k)
        delta_omega0 = 1/self.r*(delta_S + self.L*delta_omega)
        delta_omega1 = 1/self.r*(delta_S - self.L*delta_omega)

        #Calculating the needed angular velocity of each wheel
        #Multiplying with -1 to get the same sign on the rotational velocity as the lawnmower. 
        #The model has defined the opposite sign of positive angular velocity, 
        #which means we have to change it so that the model is the same as the lawnmower
        dtheta0_dt = -1*delta_omega0
        dtheta1_dt = -1*delta_omega1

        print("DTHETA 0: ", dtheta0_dt, "DTHETA 1: ", dtheta1_dt)
        print("")

        dtheta_sum = dtheta0_dt + dtheta1_dt
        l_ratio = dtheta0_dt/dtheta_sum
        r_ratio = dtheta1_dt/dtheta_sum

        print("l_ratio: ", l_ratio, "r_ratio: ", r_ratio)

        #Converting the linear and angular velocity to the signals

        jonas_steering = False

        if jonas_steering == True:
            if dtheta0_dt == -dtheta1_dt:
                if dtheta0_dt > dtheta1_dt:
                    steering = 2
                else:
                    steering = -2
            else:
                self.steering = 2*(dtheta1_dt - dtheta0_dt)/(dtheta0_dt + dtheta1_dt)
        else:
            max_steering = 1
            if l_ratio > r_ratio:                                   # left turn
                self.steering = max_steering * (l_ratio-r_ratio)
            else:                                                   # right turn
                self.steering = - max_steering * (r_ratio-l_ratio)
            


        speed = (dtheta0_dt + dtheta1_dt)*self.r/2

        print("calculated speed: ", speed)
        
        speed, steering = self.clamping(speed, self.steering)

        time_prev, time = self.drive_node.get_time()
        print("Drive commands:", "SPEED = ", speed, "STEERING = ", steering)
        print("")

        #Publish angular and linear velocity to the lawnmower node
        rate = self.drive_node.get_rate()
        self.drive_node.drive(speed, steering)
        rate.sleep()

        self.steering_prev = self.steering

        #Have to take the current counter and subtract the initial value to get the correct counter from the start
        wheel_0_counter, wheel_1_counter = self.drive_node.get_wheelcounters()
        wheel_0_counter_init, wheel_1_counter_init = self.drive_node.get_wheelcounters_init()

        wheelspeed0, wheelspeed1 = self.drive_node.get_wheelspeeds()

        wheel_0_counter = wheel_0_counter-wheel_0_counter_init
        wheel_1_counter = wheel_1_counter-wheel_1_counter_init
        
        print("WHEELCOUNTER 0: ", wheel_0_counter, "   WHEELCOUNTER 1: ", wheel_1_counter)
        print("WHEELCOUNTER 0 init: ", wheel_0_counter_init, "   WHEELCOUNTER 1 init: ", wheel_1_counter_init)
        print("")

        #Convert to angular displacement
        #Here I multiply with -1 again to make sure that the sign of rotation is the same in the model. Since the total step command is the same in absolute terms, we can take the negatie sign and convert it back so that the model can still be used
        theta_0_meas = wheel_0_counter*2*math.pi/self.PPR*-1
        theta_1_meas = wheel_1_counter*2*math.pi/self.PPR*-1

        print("ANGLULAR DIST 0: ", theta_0_meas, "ANGLULAR DIST 1: ", theta_1_meas)
        print("")

        #The random noise was calculated by finding the resolution of the lawnmower (360/PPR) and estimating that a reasonable error would be if the robot misses a step or reports back a too high or low step
        rand1 = random.uniform(-360/self.PPR,360/self.PPR) + random.uniform(-0.001,0.001)
        rand2 = random.uniform(-360/self.PPR,360/self.PPR) + random.uniform(-0.001,0.001)
        rand3 = random.uniform(-360/self.PPR,360/self.PPR) + random.uniform(-0.001,0.001)

        #Calculating the angular difference between the two samples
        delta_theta_0 = theta_0_meas-self.theta_0_meas_old
        delta_theta_1 = theta_1_meas-self.theta_1_meas_old     

        delta_s = self.r/2*(delta_theta_0+delta_theta_1)
        delta_theta = self.r/(2*self.L)*(delta_theta_0-delta_theta_1)

        #Updating the robots base position
        self.x_base = self.x_base + delta_s*math.cos(self.theta_old)
        self.y_base = self.y_base + delta_s*math.sin(self.theta_old)
        self.theta = self.theta + delta_theta

        print("THETA : ", self.theta)

        # get coord inits
        x_rtk, y_rtk = self.drive_node.get_rtk()
        x_init_rtk, y_init_rtk = self.drive_node.get_coord_init_pos1()
        offset_angle = self.drive_node.get_rtk_angle_offset()

        print("rkt init: x:", x_init_rtk, "y: ", y_init_rtk)
        print("rkt: x:", x_rtk, "y: ", y_rtk)
        print("offset angle: ", offset_angle)

        x_rotated, y_rotated = pos_global_to_local(x_rtk, y_rtk, x_init_rtk, y_init_rtk, offset_angle)

        # EKF
        use_kalman = False

        if use_kalman == True:
            yaw_angle = self.theta

            v = (wheelspeed0 + wheelspeed1)/2
            yaw_rate = (wheelspeed1 - wheelspeed0)/self.L

            rtk_error_x, rtk_error_y = self.drive_node.get_rtk_accuracy()

            z_k = np.array([[x_rotated],
                            [y_rotated],
                            [yaw_angle]])
            
            control_input = np.array([[v],
                                      [yaw_rate]])
            
            sensor_error = np.array([[rtk_error_x],
                                     [rtk_error_y],
                                     [0]])

            skalman_state = skalman.update(z_k, control_input, sensor_error)

            state_x = skalman_state[0].item()
            state_y = skalman_state[1].item()

        else:
            state_x = x_rotated
            state_y = y_rotated


        #Updating the chalking mechanism position
        x = self.x_base - self.D*math.cos(self.theta)
        y = self.y_base - self.D*math.sin(self.theta)

        self.x = state_x - self.D*math.cos(self.theta)
        self.y = state_y - self.D*math.sin(self.theta)

        print("RTK ROTATED X: ", self.x, "  Y: ", self.y)
        print("ODOMETRY X: ", x, "  Y: ", y)
        print("")

        x_error = self.x-x_ref
        y_error = self.y-y_ref
        
        self.theta_old = self.theta
        self.delta_xe_old = delta_xe
        self.delta_ye_old = delta_ye
        self.theta_0_meas_old = theta_0_meas
        self.theta_1_meas_old = theta_1_meas

        return x_error, y_error, self.x, self.y, self.theta, time
    

    def PID(self, error, kp, ki, kd):
        delta = error*kp+ki*self.Ts*self.err_sum_x+kd*(error-self.delta_xe_old)/self.Ts
        return delta

    

    def reset_error_sum(self):
        self.err_sum_x = 0
        self.err_sum_y = 0


    def clamping(self, speed, steering):
        speed = round(speed, 2)
        steering = round(steering, 2)

        if abs(speed) < 0.2:
            speed = 0.2
        elif speed > 1:
            speed = 1
        elif speed < -1:
            speed = -1.0

        diff = 0.1

        if abs(steering - self.steering_prev) > diff:
            if steering < self.steering_prev:
                steering = self.steering_prev - diff
            else:
                steering = self.steering_prev + diff

        if steering > 2:
            steering = 2.0
        elif steering < -2:
            steering = -2.0

        return float(speed), float(steering)
    




"""
    HELPERS
"""    
    
def rotation_matrix(angle):
    return np.array([
        [np.cos(angle), -np.sin(angle)],
        [np.sin(angle),  np.cos(angle)],
    ])
    

def pos_global_to_local(x_rtk,y_rtk, x_init_rtk,y_init_rtk,offset_angle):

    translated = np.array([x_rtk,y_rtk]) - np.array([x_init_rtk,y_init_rtk])
    print(translated)
    rotated = np.dot(rotation_matrix(offset_angle),translated)  
    pos_xy_local = rotated
    return pos_xy_local[0],pos_xy_local[1]



def control_conversion(speed, steering):
    longitudonal = ...
    yaw_rate = ...

    return longitudonal, yaw_rate