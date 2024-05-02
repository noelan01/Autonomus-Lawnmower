#This is the main program for controlling the odometry of the robot
import math
import numpy as np
#import matplotlib.pyplot as plt
import cmath
import random
import rclpy

import kalman

import node_lawnmower_control 
import coord_transformation
#import calibration_offset

class Regulation():
    def __init__(self, drive_node):
        self.drive_node = drive_node

        #Starting with defining variables for the robot
        self.D = 0.4   # distance between RTK module and track.
        self.r = 0.752/(2*math.pi)
        self.L = (43/2+3.2/2)/100
        self.chalk_offset = 0.4     # distance between 
        
        #Other variables
        self.x = 0
        self.y = 0
        self.y_old = 0
        self.x_old = 0
        self.x_kalman = 0
        self.y_kalman = 0

        self.rtk_x, self.rtk_y = self.drive_node.get_rtk_init()

        self.theta = -np.pi/2

        self.x_base = 0
        self.y_base = 0
        self.x_base_kalman = 0
        self.y_base_kalman = 0
        self.theta_kalman = 0
        self.reset_integral = False

        # PID PARAMETERS
        self.Kp_x = 20
        self.Ki_x = 20
        self.Kd_x = 0.5
        
        # y
        self.Kp_y = 20
        self.Ki_y = 20
        self.Kd_y = 0.5
    

        #Put the sample time to the same as the update time of the drive publish node
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

        self.delta_xe_old = 0
        self.delta_ye_old = 0

        #This angle should be set based on which coordinate we are moving towards
        self.theta_old = self.theta
        self.theta_0_meas_old = 0
        self.theta_1_meas_old = 0

        self.PPR = 349

        self.steering_prev = 0
        self.x_error = 0
        self.y_error = 0
        self.x_error_old = 0
        self.y_error_old = 0
        self.theta_odo=-math.pi
        self.theta_kalman=0

        self.skalman = kalman.EKF(0, self.theta + np.pi)


    def update(self, x_ref, y_ref,dir, rotate):
        
        
        # keep up with regular shit
        x_ref = x_ref
        y_ref = y_ref
        dir = dir
        
        #Implementing the kinematic model of the robot
        #Drive with RTK data
        delta_xe = x_ref - self.rtk_x
        delta_ye = y_ref - self.rtk_y
            
        if rotate == False:
            self.err_sum_x = self.err_sum_x + delta_xe*self.Ts
            self.err_sum_y = self.err_sum_y + delta_ye*self.Ts

            #PID regulator
            delta_x = self.PID_x(delta_xe, self.Kp_x, self.Ki_x, self.Kd_x)
            delta_y = self.PID_y(delta_ye, self.Kp_y, self.Ki_y, self.Kd_y)
            
            print("PID: X: ", delta_x, "Y: ", delta_y)
            #print("error sum x: ", self.err_sum_x, "error sum y: ", self.err_sum_y)
            
            #Calculating delta_omega(k) and delta_S(k)
            delta_omega = cmath.asin((delta_x*math.sin(self.theta_old)-delta_y*math.cos(self.theta_old))/self.D).real
            delta_S = self.D*math.cos(delta_omega)-self.D+delta_x*math.cos(self.theta_old)+delta_y*math.sin(self.theta_old)

            print("Delta S: ",delta_S)
            #Old version
            #Calculating delta_omega0(k) and delta_omega1(k)
            #delta_omega0 = 1/self.r*(delta_S + self.L*delta_omega)
            #delta_omega1 = 1/self.r*(delta_S - self.L*delta_omega)

            #Calculating delta_omega0(k) and delta_omega1(k)
            delta_omega1 = 1/self.r*(delta_S + self.L*delta_omega)
            delta_omega0 = 1/self.r*(delta_S - self.L*delta_omega)

            #Calculating the needed angular velocity of each wheel
            #Multiplying with -1 to get the same sign on the rotational velocity as the lawnmower. 
            #The model has defined the opposite sign of positive angular velocity, 
            #which means we have to change it so that the model is the same as the lawnmower
            dtheta1_dt = -1*delta_omega1
            dtheta0_dt = -1*delta_omega0

            print("DTHETA 1 (Left): ", dtheta1_dt, "DTHETA 0 (Right): ", dtheta0_dt)
            print("")
            

            #New version
            dtheta_sum = dtheta1_dt + dtheta0_dt

            if not dtheta_sum == 0:
                l_ratio = dtheta1_dt/dtheta_sum
                r_ratio = dtheta0_dt/dtheta_sum
            else:
                l_ratio = 0.5
                r_ratio = 0.5

            #print("l_ratio: ", l_ratio, "r_ratio: ", r_ratio)

            #Converting the linear and angular velocity to the signals

            jonas_steering = False
            noel_steering = False

            if jonas_steering == True:
                if dtheta0_dt == -dtheta1_dt:
                    if dtheta0_dt > dtheta1_dt:
                        steering = 2
                    else:
                        steering = -2
                else:
                    self.steering = 2*(dtheta1_dt - dtheta0_dt)/(dtheta0_dt + dtheta1_dt)
            elif noel_steering == True:
                if dtheta0_dt > dtheta1_dt:
                    self.steering = (dtheta0_dt-dtheta1_dt)/dtheta0_dt
                else:
                    self.steering = -(dtheta1_dt-dtheta0_dt)/dtheta1_dt
            else:
                max_steering = 1
                if l_ratio > r_ratio:                                   # right turn, since if l_ratio>r_ratio we want to turn right as the left wheel will rotate faster
                    self.steering = -max_steering * (l_ratio-r_ratio)
                else:                                                   # left turn
                    self.steering = max_steering * (r_ratio-l_ratio)
                

            speed_clamp = 16
            speed = (dtheta1_dt + dtheta0_dt)*self.r/(speed_clamp)
            # speed = 0.2

            print("calculated speed: ", speed)
            
            speed, steering = self.clamping(speed, self.steering)

            time_prev, time = self.drive_node.get_time()
            print("Drive commands:", "SPEED = ", speed, "STEERING = ", self.steering)
            print("")

            #Publish angular and linear velocity to the lawnmower node
            rate = self.drive_node.get_rate()
            self.drive_node.drive(speed, steering)
            print("RATE:", rate.sleep())
            rate.sleep()

            self.steering_prev = self.steering
        else:
            # Rotate in place
            time_prev, time = self.drive_node.get_time()
            
            rate = self.drive_node.get_rate()
            self.drive_node.drive(0.1, -2.0)
            rate.sleep()
            print("ROTATING!!!!!")

        #Have to take the current counter and subtract the initial value to get the correct counter from the start
        wheel_0_counter, wheel_1_counter = self.drive_node.get_wheelcounters()
        wheel_0_counter_init, wheel_1_counter_init = self.drive_node.get_wheelcounters_init()
        yaw_offset = self.drive_node.get_init_yaw_offset()

        wheelspeed0, wheelspeed1 = self.drive_node.get_wheelspeeds()
        yaw_angle = self.drive_node.get_yaw()

        wheel_0_counter = wheel_0_counter-wheel_0_counter_init
        wheel_1_counter = wheel_1_counter-wheel_1_counter_init
        yaw_angle = yaw_angle - yaw_offset -np.pi
        
        print("WHEELCOUNTER 1 (left): ", wheel_1_counter, "   WHEELCOUNTER 0 (right): ", wheel_0_counter)
        print("WHEELCOUNTER 1 init (left): ", wheel_1_counter_init, "   WHEELCOUNTER 0 init (right): ", wheel_0_counter_init)
        print("")

        #Convert to angular displacement
        #Here I multiply with -1 again to make sure that the sign of rotation is the same in the model. Since the total step command is the same in absolute terms, we can take the negative sign and convert it back so that the model can still be used
        theta_1_meas = wheel_1_counter*2*math.pi/self.PPR*-1
        theta_0_meas = wheel_0_counter*2*math.pi/self.PPR*-1

        print("ANGLULAR DIST 0: ", theta_0_meas, "ANGLULAR DIST 1: ", theta_1_meas)
        print("")

        #Calculating the angular difference between the two samples
        delta_theta_1 = theta_1_meas-self.theta_1_meas_old
        delta_theta_0 = theta_0_meas-self.theta_0_meas_old     

        delta_s = self.r/2*(delta_theta_1+delta_theta_0)
        delta_theta = self.r/(2*self.L)*(delta_theta_1-delta_theta_0)

        #Updating the robots base position
        self.x_base = self.x_base + delta_s*math.cos(self.theta_old)
        self.y_base = self.y_base + delta_s*math.sin(self.theta_old)

        self.theta = self.theta + delta_theta

        print("THETA odometry: ", self.theta)

        # get coord inits
        x_rtk, y_rtk = self.drive_node.get_rtk()
        x_init_rtk, y_init_rtk = self.drive_node.get_coord_init_pos1()
        offset_angle = self.drive_node.get_rtk_angle_offset()

        print("rkt init: x:", x_init_rtk, "y: ", y_init_rtk)
        print("rkt: x:", x_rtk, "y: ", y_rtk)

        x_rotated, y_rotated = pos_global_to_local(x_rtk, y_rtk, x_init_rtk, y_init_rtk, offset_angle)
        y_rotated = y_rotated*(-1)
        # EKF
        use_kalman = True

        if use_kalman == True:
            v = (wheelspeed0 + wheelspeed1)/2
            yaw_rate = (wheelspeed1 - wheelspeed0)/self.L

            rtk_error_x, rtk_error_y = self.drive_node.get_rtk_accuracy()

            z_k = np.array([[x_rotated],
                            [y_rotated],
                            [self.theta + np.pi]])
            
            control_input = np.array([[v],
                                      [yaw_rate]])
            
            sensor_error = np.array([[rtk_error_x],
                                     [rtk_error_y],
                                     [0]])

            skalman_state = self.skalman.update(z_k, control_input, sensor_error)

            state_x = skalman_state[0].item()
            state_y = skalman_state[1].item()
            #self.theta = skalman_state[2].item()
            print("yaw angle ", self.theta)

        else:
            state_x = x_rotated
            state_y = y_rotated


        #Updating the chalking mechanism position
        self.x_odometry = self.x_base -  self.D*math.cos(self.theta)
        self.y_odometry = self.y_base -  self.D*math.sin(self.theta)

        self.x_base_kalman = state_x + 0.2*math.cos(self.theta)
        self.y_base_kalman = state_y + 0.2*math.sin(self.theta)

        print("X base Kalman: ",self.x_base_kalman, "Y base Kalman: ", self.y_base_kalman)

        #Updating based on Kalman
        self.x_kalman = self.x_base_kalman - self.D*math.cos(self.theta)
        self.y_kalman = self.y_base_kalman- self.D*math.sin(self.theta)

        #Updating based on rtk
        self.rtk_x = x_rotated + 0.1*math.cos(self.theta) -  self.D*math.cos(self.theta)
        self.rtk_y = y_rotated + 0.1*math.sin(self.theta) - self.D*math.sin(self.theta)

        print("Kalman X: ", self.x_kalman, "  Y: ", self.y_kalman)
        print("ODOMETRY X: ", self.x_odometry, "  Y: ", self.y_odometry)
        print("RTK X: ",self.rtk_x, "Y: ", self.rtk_y)
        print("Direction",dir)

        # these decide what measurements we base the control on
        self.x_error_old = self.x_error
        self.x_error_old = self.x_error
        self.x_error = x_ref-self.x_kalman
        self.y_error = y_ref-self.y_kalman
        
        if dir == "x":
            signs_to_check = [self.y_kalman,self.y_old]
            signs = np.sign(signs_to_check)
        elif dir == "y":
            signs_to_check = [self.x_kalman,self.x_old]
            signs = np.sign(signs_to_check)
        else:
            signs_to_check = [self.x_kalman,self.x_old]
            signs = np.sign(signs_to_check)    
        
        if signs[0]!=signs[1]:
            reset_integral = True
            print("Crossed line")
        else:
            reset_integral = False
            print("Signs: ",signs)
        
        print("X error: ",self.x_error, "Y error: ",self.y_error)
        print("")

        #Updating the old variables to the new ones
        self.theta_old = self.theta
        self.delta_xe_old = delta_xe
        self.delta_ye_old = delta_ye
        self.theta_1_meas_old = theta_1_meas
        self.theta_0_meas_old = theta_0_meas
        self.y_old = self.y_kalman
        self.x_old = self.x_kalman


        # log data
        self.drive_node.pub_total_error(self.x_error, self.y_error)
        self.drive_node.pub_state_estimation(self.x_kalman, self.y_kalman, self.theta)
        self.drive_node.pub_odometry(self.x_odometry, self.y_odometry, self.theta)
        self.drive_node.pub_rtk(self.rtk_x, self.rtk_y)
        

        return self.x_error, self.y_error, self.x_error_old, self.y_error_old, self.x_kalman, self.y_kalman, self.theta, time, self.x_odometry, self.y_odometry, dir, self.rtk_x, self.rtk_y, reset_integral,x_ref,y_ref
    

    def PID_x(self, error, kp, ki, kd):
        delta = error*kp+ki*self.err_sum_x+kd*(error-self.delta_xe_old)/self.Ts
        return delta

    def PID_y(self, error, kp, ki, kd):
        delta = error*kp+ki*self.err_sum_y+kd*(error-self.delta_ye_old)/self.Ts
        return delta

    def reset_error_sum_dir(self,dir):
        if dir == "x":
            self.err_sum_x = 0
        elif dir == "y":
            self.err_sum_y = 0
        else:
            self.err_sum_x=0
            self.err_sum_y=0

    def reset_error_sum_crossed_line(self,dir):
        if dir == "x":
            self.err_sum_y = 0
        elif dir == "y":
            self.err_sum_x = 0        

    def reset_error_sum_rotation(self):
        self.err_sum_y = 0
        self.err_sum_x = 0   

    def clamping(self, speed, steering):
        speed = round(speed, 2)
        steering = round(steering, 2)

        #if speed >= 0 and speed < 0.2:
        #    speed = 0.2
        #elif speed < 0 and speed >-0.2:
        #    speed = -0.2
        #elif speed > 1:
        #    speed = 1
        #elif speed < -1:
        #    speed = -1.0

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
        #elif steering > 1 and steering <= 2:
        #    speed = 0.3
        ##elif steering > 0.5 and steering <= 1:
        #    speed = 0.5


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

