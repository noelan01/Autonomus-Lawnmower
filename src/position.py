
#Import packages
import math
import numpy as np
import matplotlib.pyplot as plt
import cmath
import random
import path_planner
import RouteSequencePlanner
#import main_simulation

#Create variables so the classes can be reached
path = path_planner.Path()
route = RouteSequencePlanner.sequencePlanner()


    
#Importing the Kalman filter code
import kalman
state_estimation = kalman.EKF(0)

#This is the code that simulates the lawnmower
def simulation(delta_xe,delta_ye,x_ref,y_ref,x_error,y_error,theta,Ts,delta_x,delta_y,delta_omega,delta_S,delta_omega1,delta_omega2,dtheta1_dt,dtheta2_dt,s,lin_vel,ang_vel,dtheta1_out_dt,dtheta2_out_dt,r,L,x,y,D,PPR,theta_1_meas,theta_2_meas,T,delta_theta_1,delta_theta_2,delta_s,delta_theta,x_base,y_base,x_base_kalman,y_base_kalman,theta_kalman,x_kalman,y_kalman,tot_error,next_point,rotate,delta_xe_old,delta_ye_old,theta_old,dtheta1_out_dt_old,dtheta2_out_dt_old,theta_1_meas_old,theta_2_meas_old,x_base_old,y_base_old,Kalman,y_old,x_old,err_sum_x,err_sum_y,acc_sum_delta_omega_1,acc_sum_delta_omega_2):

    x_ref = next_point[0]
    y_ref = next_point[1]
    dir = next_point[2]

    Kp_x = 20
    Ki_x = 20
    Kd_x = 0.5
    Kp_y = 20
    Ki_y = 20
    Kd_y = 0.5

    #Implementing the kinematic model of the robot
    delta_xe = x_ref - x
    delta_ye = y_ref - y

    err_sum_x = err_sum_x + delta_xe*Ts
    err_sum_y = err_sum_y + delta_ye*Ts
    
       
    if rotate == False:

        delta_x = PID(delta_xe, Kp_x, Ki_x, Kd_x,err_sum_x,delta_xe_old,Ts)
        delta_y = PID(delta_ye, Kp_y, Ki_y, Kd_y,err_sum_y,delta_ye_old,Ts)

        #Increasing the error with the PID-controller
        #delta_x = delta_xe*Kp_x+Ki_x*err_sum_x+Kd_x*(delta_xe-delta_xe_old)/Ts
        #delta_y = delta_ye*Kp_y+Ki_y*err_sum_y+Kd_y*(delta_ye-delta_ye_old)/Ts

        #Calculating delta_omega(k) and delta_S(k)
        delta_omega = cmath.asin((delta_x*math.sin(theta_old)-delta_y*math.cos(theta_old))/D).real
        delta_S = D*math.cos(delta_omega)-D+delta_x*math.cos(theta_old)+delta_y*math.sin(theta_old)

        #Calculating delta_omega1(k) and delta_omega2(k)
        delta_omega1 = 1/r*(delta_S+L*delta_omega)
        delta_omega2 = 1/r*(delta_S-L*delta_omega)    

        #Calculating the accumulated signal
        acc_sum_delta_omega_1 += delta_omega1*Ts
        acc_sum_delta_omega_2 += delta_omega2*Ts


        #Calculating the needed angular velocity of each wheel
        clamping = 1
        dtheta1_dt = (delta_omega1)/clamping
        dtheta2_dt = (delta_omega2)/clamping

        #Clamping the angular velocity of the wheels to be more representable of the real lawnmower
        if dtheta1_dt>6.5:
            dtheta1_dt = 6.5
        elif dtheta1_dt<-6.5:
            dtheta1_dt = -6.5
        if dtheta2_dt>6.5:
            dtheta2_dt =6.5
        elif dtheta2_dt<-6.5:
            dtheta2_dt = -6.5

        #Calculating the steering variable to see the output
        #s = (dtheta1_dt-dtheta2_dt)/dtheta1_dt

        #Converting to linear and angular movement of the robot to use in the Kalman filter
        lin_vel = (r/2*(dtheta1_dt+dtheta2_dt))
        ang_vel = (r/(2*L)*(dtheta1_dt-dtheta2_dt))

    elif rotate == True:
        dtheta1_dt = 2
        dtheta2_dt = -2
        print("ROTATING!!!!")

    #The random noise was calculated by finding the resolution of the lawnmower (360/PPR) and estimating that a reasonable error would be if the robot misses a step or reports back a too high or low step
    rand1 = random.uniform(-2*math.pi/PPR,2*math.pi/PPR) + random.uniform(-0.01,0.01)
    rand2 = random.uniform(-2*math.pi/PPR,2*math.pi/PPR) + random.uniform(-0.01,0.01)
    rand3 = random.uniform(-360/PPR,360/PPR) + random.uniform(-0.001,0.001)

    dtheta1_out_dt = dtheta1_dt*Ts/(T+Ts)+dtheta1_out_dt_old*T/(T+Ts)+rand1
    dtheta2_out_dt = dtheta2_dt*Ts/(T+Ts)+dtheta2_out_dt_old*T/(T+Ts)+rand2

    theta_1_meas = theta_1_meas_old+(dtheta1_out_dt*Ts)
    theta_2_meas = theta_2_meas_old+(dtheta2_out_dt*Ts)

    #Calculating the angular difference between the two samples
    delta_theta_1 = theta_1_meas-theta_1_meas_old
    delta_theta_2 = theta_2_meas-theta_2_meas_old

    delta_s = r/2*(delta_theta_1+delta_theta_2)
    delta_theta = r/(2*L)*(delta_theta_1-delta_theta_2)

    #Updating the robots base position
    x_base = (x_base_old +delta_s*math.cos(theta_old))
    y_base = (y_base_old + delta_s*math.sin(theta_old))
    theta = (theta_old + delta_theta)

    if Kalman == True:
        #Variables to the Kalman function
        Z_k = np.array([[x_base],[y_base],[theta]])
        control_inputs = np.array([[lin_vel],[ang_vel]])
        sensor_error = np.array([[rand1],[rand2],[rand3]])
        no_sensor_error = np.array([[0],[0],[0]])
        #Filtering with the Kalman filter to get a better estimation of the position
        state = state_estimation.update(Z_k,control_inputs,sensor_error)
        
        x_base_kalman.append(state[0].item())
        y_base_kalman.append(state[1].item())
        theta_kalman.append(state[2].item())
        x_kalman.append(x_base_kalman - D*math.cos(theta_kalman))
        y_kalman.append(y_base_kalman-D*math.sin(theta))

    #Updating the chalking mechanism position
    x = x_base - D*math.cos(theta)
    y = y_base - D*math.sin(theta) 

    delta_xe_old = delta_xe
    delta_ye_old = delta_ye        

    x_error = x_ref-x
    y_error = y_ref-y

    if dir == "x":
        signs_to_check = [y,y_old]
        signs = np.sign(signs_to_check)
    elif dir == "y":
        signs_to_check = [x,x_old]
        signs = np.sign(signs_to_check)

    if dir == "x" or dir  == "y": 
        if signs[0]!=signs[1]:
            reset_integral = True
            print("Crossed line")
        else:
            reset_integral = False
            print("Signs: ",signs)
    else:
        reset_integral = False

    print("THETA: ",theta)

    #Updating old values
    dtheta1_out_dt_old = dtheta1_out_dt
    dtheta2_out_dt_old = dtheta2_out_dt
    theta_old = theta
    delta_xe_old = delta_xe
    delta_ye_old = delta_ye
    theta_2_meas_old = theta_2_meas
    theta_1_meas_old = theta_1_meas
    y_base_old = y_base
    x_base_old = x_base
    y_old = y
    x_old = x


    return delta_xe,delta_ye,x_ref,y_ref,x_error,y_error,theta,Ts,delta_x,delta_y,delta_omega,delta_S,delta_omega1,delta_omega2,dtheta1_dt,dtheta2_dt,s,lin_vel,ang_vel,dtheta1_out_dt,dtheta2_out_dt,r,L,x,y,D,PPR,theta_1_meas,theta_2_meas,T,delta_theta_1,delta_theta_2,delta_s,delta_theta,x_base,y_base,x_base_kalman,y_base_kalman,theta_kalman,x_kalman,y_kalman,tot_error,next_point,rotate,delta_xe_old,delta_ye_old,theta_old,dtheta1_out_dt_old,dtheta2_out_dt_old,theta_1_meas_old,theta_2_meas_old,x_base_old,y_base_old,Kalman,y_old,x_old,err_sum_x,err_sum_y,dir,reset_integral,acc_sum_delta_omega_1,acc_sum_delta_omega_2


def reset_error_sum_dir(dir,err_sum_x,err_sum_y):
        if dir == "x" or dir == "-x":
            err_sum_x = 0
            err_sum_y = err_sum_y
        elif dir == "y" or dir == "-y":
            err_sum_x = err_sum_x
            err_sum_y = 0
        return err_sum_x,err_sum_y


def reset_error_sum_crossed_line(dir,err_sum_x,err_sum_y):
    if dir == "x" or dir == "-x":
        err_sum_x = err_sum_x
        err_sum_y = 0
    elif dir == "y" or dir == "-y":
        err_sum_x = 0  
        err_sum_y = err_sum_y
    return err_sum_x,err_sum_y

def reset_error_sum_rotation():
    err_sum_x = 0
    err_sum_y = 0
    return err_sum_x,err_sum_y


def PID(error, kp, ki, kd,err_sum_x,delta_xe_old,Ts):
        delta = error*kp+ki*err_sum_x+kd*(error-delta_xe_old)/Ts
        return delta