#This is the main program for controlling the odometry of the robot
import math
import numpy as np
#import matplotlib.pyplot as plt
import cmath
import random
import rclpy

import kalman
state_estimation = kalman.EKF(0)

import node_lawnmower_control 
import coord_transformation
import calibration_offset


"""
def regulation():

    #Starting with defining variables for the robot
    D = 0.4
    r = 0.752/(2*math.pi)
    L = (43/2+3.2/2)/100
    
    #Other variables
    x = 0
    y = 0
    x_kalman = 0
    y_kalman = 0

    theta = 0
    delta_x = 0
    delta_y = 0
    delta_xe = 0
    delta_ye = 0

    x_ref = 0
    y_ref = 0

    x_base = 0
    y_base = 0
    x_base_kalman = 0
    y_base_kalman = 0
    x_error = 0
    y_error = 0
    theta_kalman = 0
    Kp = 15
    Ki = 2
    Kd = 0.08
    Ts = 0.025
    acc_sum_delta_omega_1 = 0
    acc_sum_delta_omega_2 = 0    
    theta_1_meas = 0
    theta_2_meas = 0
    theta_1_increment = 0
    theta_2_increment = 0
    delta_theta_1 = 0
    delta_theta_2 = 0
    dtheta1_dt = 0
    dtheta2_dt = 0
    lin_vel = 0
    ang_vel = 0
    err_sum_x = 0
    err_sum_y = 0

    delta_omega = 0
    delta_S = 0
    delta_s = 0
    delta_theta = 0
    delta_omega1 = 0
    delta_omega2 = 0
    delta_xe_old = 0
    delta_ye_old = 0
    theta_old = 0
    theta_1_meas_old = 0
    theta_2_meas_old = 0

    PPR = 349
    max_speed = 0.6
    v = 0.5

    #Updating x_ref & y_ref
    x_ref = x_ref - 0.025
    y_ref = y_ref
    
    #Implementing the kinematic model of the robot
    delta_xe = x_ref - x
    delta_ye = y_ref - y

    err_sum_x = err_sum_x + delta_xe
    err_sum_y = err_sum_y + delta_ye 

    #Increasing the error with the proportional gain
    delta_x = delta_xe*Kp+Ki*Ts*err_sum_x+Kd*(delta_xe-delta_xe_old)/Ts
    delta_y = delta_ye*Kp+Ki*Ts*err_sum_y+Kd*(delta_ye-delta_ye_old)/Ts

    #Calculating delta_omega(k) and delta_S(k)
    delta_omega = cmath.asin((delta_x*math.sin(theta_old)-delta_y*math.cos(theta_old))/D).real
    delta_S = D*math.cos(delta_omega)+D+delta_x*math.cos(theta_old)+delta_y*math.sin(theta_old)

    #Calculating delta_omega1(k) and delta_omega2(k)
    delta_omega1 = 1/r*(delta_S+L*delta_omega)
    delta_omega2 = 1/r*(delta_S-L*delta_omega)

    #Discrete time integration using backwards Euler
    acc_sum_delta_omega_1 = acc_sum_delta_omega_1 + Ts*delta_omega1
    acc_sum_delta_omega_2 = acc_sum_delta_omega_2 + Ts*delta_omega2

    #Measure the difference between the old wheel counter and the accumulated sum to find angular displacement
    theta_1_increment = acc_sum_delta_omega_1-theta_1_meas
    theta_2_increment = acc_sum_delta_omega_2-theta_2_meas

    #Calculating the needed angular velocity of each wheel
    dtheta1_dt = theta_1_increment/Ts
    dtheta2_dt = theta_2_increment/Ts

    steering = (dtheta1_dt-dtheta2_dt)/(v*max_speed)

    #Publish angular and linear velocity to the lawnmower node
    rate = drive_node.get_rate()
    drive_node.drive(0.5,steering)
    rate.sleep()

    wheel_1_counter = drive_node.get_wheelcounter0()
    wheel_2_counter = drive_node.get_wheelcounter1()

    #Convert to angular displacement
    theta_1_meas = wheel_1_counter*2*math.pi/PPR
    theta_2_meas = wheel_2_counter*2*math.pi/PPR

    #The random noise was calculated by finding the resolution of the lawnmower (360/PPR) and estimating that a reasonable error would be if the robot misses a step or reports back a too high or low step
    rand1 = random.uniform(-360/PPR,360/PPR) + random.uniform(-0.001,0.001)
    rand2 = random.uniform(-360/PPR,360/PPR) + random.uniform(-0.001,0.001)
    rand3 = random.uniform(-360/PPR,360/PPR) + random.uniform(-0.001,0.001)

    #Calculating the angular difference between the two samples
    delta_theta_1 = theta_1_meas-theta_1_meas_old
    delta_theta_2 = theta_2_meas-theta_2_meas_old     

    delta_s = r/2*(delta_theta_1+delta_theta_2)
    delta_theta = r/(2*L)*(delta_theta_1-delta_theta_2)

    #Updating the robots base position
    x_base = x_base +delta_s*math.cos(theta_old)
    y_base = y_base + delta_s*math.sin(theta_old)
    theta = theta + delta_theta

        #Variables to the Kalman function
        #Z_k = np.array([[x_base[k]],[y_base[k]],[theta[k]]])
        #control_inputs = np.array([[lin_vel[k]],[ang_vel[k]]])
        #sensor_error = np.array([[rand1],[rand2],[rand3]])
        #no_sensor_error = np.array([[0],[0],[0]])
        #Filtering with the Kalman filter to get a better estimation of the position
        #state = state_estimation.update(Z_k,control_inputs,sensor_error)
        
        #x_base_kalman.append(state[0].item())
        #y_base_kalman.append(state[1].item())
        #theta_kalman.append(state[2].item())

    #Updating the chalking mechanism position
    x = x_base - D*math.cos(theta) 
    y = y_base - D*math.sin(theta) 

        #x_kalman.append(x_base_kalman[k] - D*math.cos(theta_kalman[k]))
        #y_kalman.append(y_base_kalman[k]-D*math.sin(theta[k]))
        #print(x[k]-x_kalman[k])
        #print(y-y_kalman)

    x_error = x-x_ref
    y_error = y-y_ref
    
    theta_old = theta
    delta_xe_old = delta_xe
    delta_ye_old = delta_ye
    theta_1_meas_old = theta_1_meas
    theta_2_meas_old = theta_2_meas
        
#regulation()
"""


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

        self.theta = - np.pi/2
        self.delta_x = 0
        self.delta_y = 0
        self.delta_xe = 0
        self.delta_ye = 0

        self.x_ref = 0
        self.y_ref = 0

        self.x_base = 0
        self.y_base = 0
        self.x_base_kalman = 0
        self.y_base_kalman = 0
        self.x_error = 0
        self.y_error = 0
        self.theta_kalman = 0

        # PID PARAMETERS
        self.Kp = 20
        self.Ki = 0
        self.Kd = 0

        #Put the sample time to the same as the update time of the drive publish node?
        self.Ts = 1/self.drive_node.get_updaterate()
        self.acc_sum_delta_omega_1 = 0
        self.acc_sum_delta_omega_2 = 0    

        #Change these to the initial values of the counters
        self.wheel_1_counter_init, self.wheel_2_counter_init = self.drive_node.get_wheelcounters_init()

        self.wheel_1_counter, self.wheel_2_counter = self.drive_node.get_wheelcounters()

        self.theta_1_meas = 0
        self.theta_2_meas = 0

        self.theta_1_increment = 0
        self.theta_2_increment = 0
        self.delta_theta_1 = 0
        self.delta_theta_2 = 0
        self.dtheta1_dt = 0
        self.dtheta2_dt = 0
        self.lin_vel = 0
        self.ang_vel = 0
        self.err_sum_x = 0
        self.err_sum_y = 0

        self.delta_omega = 0
        self.delta_S = 0
        self.delta_s = 0
        self.delta_theta = 0
        self.delta_omega1 = 0
        self.delta_omega2 = 0
        self.delta_xe_old = 0
        self.delta_ye_old = 0

        #This angle should be set based on which coordinate we are moving towards
        self.theta_old = self.theta
        self.theta_1_meas_old = 0
        self.theta_2_meas_old = 0

        #RTK info
        self.x_init_rtk, self.y_init_rtk = 0 #Get the first point from Noels node 
        self.offset_angle = calibration_offset.get_offset(0,0,0,0)   #input points from Noels node
        self.x_rtk, self.y_rtk = self.drive_node.get_rtk()

        self.PPR = 349

        self.steering_prev = 0

    def update(self, x_ref, y_ref):

        #if abs(self.x) > 2 or abs(self.y) > 2:
        #    self.drive_node.stop_drive()
        #    rclpy.shutdown()

        #Updating x_ref & y_ref
        #self.x_ref = self.x_ref -0.025
        #self.y_ref = self.y_ref

        self.x_ref = x_ref
        self.y_ref = y_ref
        
        #Implementing the kinematic model of the robot
        self.delta_xe = self.x_ref - self.x
        self.delta_ye = self.y_ref - self.y

        self.err_sum_x = self.err_sum_x + self.delta_xe
        self.err_sum_y = self.err_sum_y + self.delta_ye 

        #PID regulator
        self.delta_x = self.delta_xe*self.Kp+self.Ki*self.Ts*self.err_sum_x+self.Kd*(self.delta_xe-self.delta_xe_old)/self.Ts
        self.delta_y = self.delta_ye*self.Kp+self.Ki*self.Ts*self.err_sum_y+self.Kd*(self.delta_ye-self.delta_ye_old)/self.Ts
        
        #print("PID: X: ", self.delta_x, "Y: ", self.delta_y)
        #print("")
        
        #Calculating delta_omega(k) and delta_S(k)
        self.delta_omega = cmath.asin((self.delta_x*math.sin(self.theta_old)-self.delta_y*math.cos(self.theta_old))/self.D).real
        self.delta_S = self.D*math.cos(self.delta_omega)+self.D+self.delta_x*math.cos(self.theta_old)+self.delta_y*math.sin(self.theta_old)

        #Calculating delta_omega1(k) and delta_omega2(k)
        self.delta_omega1 = 1/self.r*(self.delta_S + self.L*self.delta_omega)
        self.delta_omega2 = 1/self.r*(self.delta_S - self.L*self.delta_omega)

        #Discrete time integration using backwards Euler
        #self.acc_sum_delta_omega_1 = self.acc_sum_delta_omega_1 + self.Ts*self.delta_omega1
        #self.acc_sum_delta_omega_2 = self.acc_sum_delta_omega_2 + self.Ts*self.delta_omega2

        #Measure the difference between the old wheel counter and the accumulated sum to find angular displacement
        #self.theta_1_increment = self.acc_sum_delta_omega_1-self.theta_1_meas
        #self.theta_2_increment = self.acc_sum_delta_omega_2-self.theta_2_meas

        #Base the incremental position on the command instead of the wheel measure
        #self.theta_1_increment = self.Ts*self.delta_omega1
        #self.theta_2_increment = self.Ts*self.delta_omega2

        #Calculating the needed angular velocity of each wheel
        #Multiplying with -1 to get the same sign on the rotational velocity as the lawnmower. The model has defined the opposite sign of positive angular velocity, which means we have to change it so that the model is the same as the lawnmower
        self.dtheta1_dt = -1*self.delta_omega1
        self.dtheta2_dt = -1*self.delta_omega2

        print("DTHETA 0: ", self.dtheta1_dt, "DTHETA 0: ", self.dtheta1_dt)
        print("")

        #Converting the linear and angular velocity to the signals
        self.steering = (self.dtheta1_dt-self.dtheta2_dt)/self.dtheta1_dt
        #steering_scale = abs(self.steering - 2) / (2 * 2)

        speed = 0.2
        
        #Publish angular and linear velocity to the lawnmower node
        speed, steering = self.clamping(speed, self.steering)

        time_prev, time = self.drive_node.get_time()
        print("Drive commands:", "SPEED = ", speed, "STEERING = ", steering)
        print("")

        rate = self.drive_node.get_rate()
        self.drive_node.drive(speed, steering)
        rate.sleep()

        self.steering_prev = self.steering
        #Have to take the current counter and subtract the initial value to get the correct counter from the start
        self.wheel_1_counter, self.wheel_2_counter = self.drive_node.get_wheelcounters()
        self.wheel_1_counter_init, self.wheel_2_counter_init = self.drive_node.get_wheelcounters_init()

        self.wheel_1_counter = self.wheel_1_counter-self.wheel_1_counter_init
        self.wheel_2_counter = self.wheel_2_counter-self.wheel_2_counter_init
        
        print("WHEELCOUNTER 0: ", self.wheel_1_counter, "   WHEELCOUNTER 1: ", self.wheel_2_counter)
        print("")
        print("WHEELCOUNTER 0 init: ", self.wheel_1_counter_init, "   WHEELCOUNTER 1 init: ", self.wheel_2_counter_init)
        print("")

        #Convert to angular displacement
        #Here I multiply with -1 again to make sure that the sign of rotation is the same in the model. Since the total step command is the same in absolute terms, we can take the negatie sign and convert it back so that the model can still be used
        self.theta_1_meas = self.wheel_1_counter*2*math.pi/self.PPR*-1
        self.theta_2_meas = self.wheel_2_counter*2*math.pi/self.PPR*-1

        print("ANGLULAR DIST 0: ", self.theta_1_meas, "ANGLULAR DIST 1: ", self.theta_2_meas)
        print("")

        #The random noise was calculated by finding the resolution of the lawnmower (360/PPR) and estimating that a reasonable error would be if the robot misses a step or reports back a too high or low step
        self.rand1 = random.uniform(-360/self.PPR,360/self.PPR) + random.uniform(-0.001,0.001)
        self.rand2 = random.uniform(-360/self.PPR,360/self.PPR) + random.uniform(-0.001,0.001)
        self.rand3 = random.uniform(-360/self.PPR,360/self.PPR) + random.uniform(-0.001,0.001)

        #Calculating the angular difference between the two samples
        self.delta_theta_1 = self.theta_1_meas-self.theta_1_meas_old
        self.delta_theta_2 = self.theta_2_meas-self.theta_2_meas_old     

        self.delta_s = self.r/2*(self.delta_theta_1+self.delta_theta_2)
        self.delta_theta = self.r/(2*self.L)*(self.delta_theta_1-self.delta_theta_2)

        #Updating the robots base position
        self.x_base = self.x_base + self.delta_s*math.cos(self.theta_old)
        self.y_base = self.y_base + self.delta_s*math.sin(self.theta_old)
        self.theta = self.theta + self.delta_theta

        print("THETA : ", self.theta)

        self.x_rotated_rtk, self.y_rotated_rtk = coord_transformation.pos_global_to_local(self.x_rtk,self.y_rtk,self.x_init_rtk,self.y_init_rtk,self.offset_angle)



            #Variables to the Kalman function
            #Z_k = np.array([[x_base[k]],[y_base[k]],[theta[k]]])
            #control_inputs = np.array([[lin_vel[k]],[ang_vel[k]]])
            #sensor_error = np.array([[rand1],[rand2],[rand3]])
            #no_sensor_error = np.array([[0],[0],[0]])
            #Filtering with the Kalman filter to get a better estimation of the position
            #state = state_estimation.update(Z_k,control_inputs,sensor_error)
            
            #x_base_kalman.append(state[0].item())
            #y_base_kalman.append(state[1].item())
            #theta_kalman.append(state[2].item())

        #Updating the chalking mechanism position
        self.x = self.x_base - self.D*math.cos(self.theta)
        self.y = self.y_base - self.D*math.sin(self.theta) 

        print("X: ", self.x, "  Y: ", self.y)
        print("")
        print("")
        print("")

            #x_kalman.append(x_base_kalman[k] - D*math.cos(theta_kalman[k]))
            #y_kalman.append(y_base_kalman[k]-D*math.sin(theta[k]))
            #print(x[k]-x_kalman[k])
            #print(y-y_kalman)

        self.x_error = self.x-self.x_ref
        self.y_error = self.y-self.y_ref
        
        self.theta_old = self.theta
        self.delta_xe_old = self.delta_xe
        self.delta_ye_old = self.delta_ye
        self.theta_1_meas_old = self.theta_1_meas
        self.theta_2_meas_old = self.theta_2_meas

        return self.x_error, self.y_error, self.x, self.y, self.theta, time


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