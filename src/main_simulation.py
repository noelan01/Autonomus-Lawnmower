
#Import packages
import math
import numpy as np
import matplotlib.pyplot as plt
import cmath
import random
import path_planner
import RouteSequencePlanner

#Create variables so the classes can be reached
path = path_planner.Path()
route = RouteSequencePlanner.sequencePlanner()

path = path_planner.Path()

import position
simulation  = position.simulation

def main():

    r = 0.752/(2*math.pi)
    L = (43/2+3.2/2)/100
    D = 0.2
    
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

    Ts = 0.1

    #Time constant for inner system
    T = 0.4

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
    dtheta1_out_dt = 0
    dtheta2_out_dt = 0
    lin_vel = 0
    ang_vel = 0
    err_sum_x = 0
    err_sum_y = 0
    s = 0
    tot_error = 0
    dir = 0

    delta_omega = 0
    delta_S = 0
    delta_s = 0
    delta_theta = 0
    delta_omega1 = 0
    delta_omega2 = 0
    reset_integral = False
    x_to_plot = 0
    y_to_plot = 0
    PPR = 349

    #Old variables
    delta_xe_old = 0
    delta_ye_old = 0
    theta_old = theta
    dtheta1_out_dt_old = 0
    dtheta2_out_dt_old = 0
    theta_1_meas_old = 0
    theta_2_meas_old = 0
    x_base_old = 0
    y_base_old = 0
    Kalman = False
    rotate = False
    y_old = 0
    x_old = 0
    index_end_point = 0
    x_to_plot = [0]
    y_to_plot = [0]
    y_to_plot_1 = [0]
    y_to_plot_2 = [0]
    y_to_plot_3 = [0]
    err_to_plot = [0]
    y_err = [0]
    x_err = [0]
    dir = [0]
    threshold = 0.01
    
    #Defining simulation time
    simTime = 120
    nrOfSteps = int(simTime/Ts)
    
    #load pitch data from json file
    route.load_pitch_data()

    # path.set_circle_path(9.15,(9.15,0),8000,"None")
    #Outer lines
    route.outerLines(path)
    
    # Lower penalty area
    route.lowerPenaltyArea(path)
    
    route.lowerArc(path)
    
    #Lower goal area
    route.lowerGoalArea(path)

    # #Transport path to reach mid line
    route.transportMidLine(path)

    # #Midline and mid circle
    route.midLine(path)
    
    #Transport from midLine to upper goal area
    route.driveToUpperLineFromMid(path)

    #Upper penalty area
    route.upperPenaltyArea(path)
    route.upperArc(path)

    #upper goal area

    route.upperGoalArea(path)

    #Corner flags
    route.upperLeftCorner(path)
    route.lowerLeftCorner(path)
    route.bottomRightCorner(path)
    route.upperRightCorner(path)

    #Defining the reached goal variable to false to begin the simulation
    reached_goal = False

    next_point = path.get_point()
    k = 0

    while reached_goal == False:

        delta_xe,delta_ye,x_ref,y_ref,x_error,y_error,theta,Ts,delta_x,delta_y,delta_omega,delta_S,delta_omega1,delta_omega2,dtheta1_dt,dtheta2_dt,s,lin_vel,ang_vel,dtheta1_out_dt,dtheta2_out_dt,r,L,x,y,D,PPR,theta_1_meas,theta_2_meas,T,delta_theta_1,delta_theta_2,delta_s,delta_theta,x_base,y_base,x_base_kalman,y_base_kalman,theta_kalman,x_kalman,y_kalman,tot_error,next_point,rotate,delta_xe_old,delta_ye_old,theta_old,dtheta1_out_dt_old,dtheta2_out_dt_old,theta_1_meas_old,theta_2_meas_old,x_base_old,y_base_old,Kalman,y_old,x_old,err_sum_x,err_sum_y,dir,reset_integral,acc_sum_delta_omega_1,acc_sum_delta_omega_2 = simulation(delta_xe,delta_ye,x_ref,y_ref,x_error,y_error,theta,Ts,delta_x,delta_y,delta_omega,delta_S,delta_omega1,delta_omega2,dtheta1_dt,dtheta2_dt,s,lin_vel,ang_vel,dtheta1_out_dt,dtheta2_out_dt,r,L,x,y,D,PPR,theta_1_meas,theta_2_meas,T,delta_theta_1,delta_theta_2,delta_s,delta_theta,x_base,y_base,x_base_kalman,y_base_kalman,theta_kalman,x_kalman,y_kalman,tot_error,next_point,rotate,delta_xe_old,delta_ye_old,theta_old,dtheta1_out_dt_old,dtheta2_out_dt_old,theta_1_meas_old,theta_2_meas_old,x_base_old,y_base_old,Kalman,y_old,x_old,err_sum_x,err_sum_y,acc_sum_delta_omega_1,acc_sum_delta_omega_2)

        theta_ref = theta
        
        next_point,err_sum_x,err_sum_y,index_end_point = goal(x_error, y_error, dir, reset_integral, theta_ref,delta_xe,delta_ye,x_ref,y_ref,theta,Ts,delta_x,delta_y,delta_omega,delta_S,delta_omega1,delta_omega2,dtheta1_dt,dtheta2_dt,s,lin_vel,ang_vel,dtheta1_out_dt,dtheta2_out_dt,r,L,x,y,D,PPR,theta_1_meas,theta_2_meas,T,delta_theta_1,delta_theta_2,delta_s,delta_theta,x_base,y_base,x_base_kalman,y_base_kalman,theta_kalman,x_kalman,y_kalman,tot_error,next_point,rotate,delta_xe_old,delta_ye_old,theta_old,dtheta1_out_dt_old,dtheta2_out_dt_old,theta_1_meas_old,theta_2_meas_old,x_base_old,y_base_old,Kalman,y_old,x_old,err_sum_x,err_sum_y,index_end_point,acc_sum_delta_omega_1,acc_sum_delta_omega_2,threshold)

        if next_point[0] == None or next_point[1]==None:
            reached_goal = True


        if dir == "x" or dir == "-x":
            y_to_plot_1.append(delta_ye)
            err_to_plot.append(delta_ye)
        elif dir == "y" or dir == "-y":
            y_to_plot_2.append(delta_xe)
            err_to_plot.append(delta_xe)
        elif dir == "None":
            y_to_plot_3.append(np.sqrt(delta_xe**2+delta_ye**2))
            err_to_plot.append(np.sqrt(delta_xe**2+delta_ye**2))
        else:
            err_to_plot.append(np.sqrt(delta_xe**2+delta_ye**2))
        
        x_to_plot.append(x)
        y_to_plot.append(y)

        k +=1
        
    plot(x_to_plot,y_to_plot,Kp_x=20,Ki_x=20,Kd_x=0.5,y_to_plot_1=y_to_plot_1,y_to_plot_2=y_to_plot_2,y_to_plot_3=y_to_plot_3,err_to_plot=err_to_plot,y_to_plot=y_to_plot)
        



def goal(x_error, y_error, dir, reset_integral, theta_ref,delta_xe,delta_ye,x_ref,y_ref,theta,Ts,delta_x,delta_y,delta_omega,delta_S,delta_omega1,delta_omega2,dtheta1_dt,dtheta2_dt,s,lin_vel,ang_vel,dtheta1_out_dt,dtheta2_out_dt,r,L,x,y,D,PPR,theta_1_meas,theta_2_meas,T,delta_theta_1,delta_theta_2,delta_s,delta_theta,x_base,y_base,x_base_kalman,y_base_kalman,theta_kalman,x_kalman,y_kalman,tot_error,next_point,rotate,delta_xe_old,delta_ye_old,theta_old,dtheta1_out_dt_old,dtheta2_out_dt_old,theta_1_meas_old,theta_2_meas_old,x_base_old,y_base_old,Kalman,y_old,x_old,err_sum_x,err_sum_y,index_end_point,acc_sum_delta_omega_1,acc_sum_delta_omega_2,threshold):
    total_error = np.sqrt(x_error**2 +  y_error**2)

    point = path.get_point()
    end_points = path.get_endpoints()
    rotate = False

    if point in end_points and index_end_point == end_points.index(point):
        rotate = True

        while rotate == True:    
            delta_xe,delta_ye,x_ref,y_ref,x_error,y_error,theta,Ts,delta_x,delta_y,delta_omega,delta_S,delta_omega1,delta_omega2,dtheta1_dt,dtheta2_dt,s,lin_vel,ang_vel,dtheta1_out_dt,dtheta2_out_dt,r,L,x,y,D,PPR,theta_1_meas,theta_2_meas,T,delta_theta_1,delta_theta_2,delta_s,delta_theta,x_base,y_base,x_base_kalman,y_base_kalman,theta_kalman,x_kalman,y_kalman,tot_error,next_point,rotate,delta_xe_old,delta_ye_old,theta_old,dtheta1_out_dt_old,dtheta2_out_dt_old,theta_1_meas_old,theta_2_meas_old,x_base_old,y_base_old,Kalman,y_old,x_old,err_sum_x,err_sum_y,dir,reset_integral,acc_sum_delta_omega_1,acc_sum_delta_omega_2 = simulation(delta_xe,delta_ye,x_ref,y_ref,x_error,y_error,theta,Ts,delta_x,delta_y,delta_omega,delta_S,delta_omega1,delta_omega2,dtheta1_dt,dtheta2_dt,s,lin_vel,ang_vel,dtheta1_out_dt,dtheta2_out_dt,r,L,x,y,D,PPR,theta_1_meas,theta_2_meas,T,delta_theta_1,delta_theta_2,delta_s,delta_theta,x_base,y_base,x_base_kalman,y_base_kalman,theta_kalman,x_kalman,y_kalman,tot_error,next_point,rotate,delta_xe_old,delta_ye_old,theta_old,dtheta1_out_dt_old,dtheta2_out_dt_old,theta_1_meas_old,theta_2_meas_old,x_base_old,y_base_old,Kalman,y_old,x_old,err_sum_x,err_sum_y,acc_sum_delta_omega_1,acc_sum_delta_omega_2)

            if abs(theta_ref - theta) >= np.pi/2:
                rotate = False
                next_dir = path._path[path._next_point][2]
                path._path[path.update_point()] = (x,y,next_dir)
                path.interpolate_points(path._path,threshold)

        err_sum_x,err_sum_y = position.reset_error_sum_rotation()
        index_end_point += 1

    seperate = True
    
    if seperate == True:
        if dir =="x":
            if x_error<0.3:
                path.update_point()
                err_sum_x,err_sum_y = position.reset_error_sum_dir(dir,err_sum_x,err_sum_y)
        elif dir == "y":
            if y_error<0.3:
                path.update_point()
                err_sum_x,err_sum_y = position.reset_error_sum_dir(dir,err_sum_x,err_sum_y)
        elif dir == "-x":
            if x_error >-0.3:
                path.update_point()
                err_sum_x,err_sum_y = position.reset_error_sum_dir(dir,err_sum_x,err_sum_y)
        elif dir == "-y":
            if y_error > -0.3:
                path.update_point()
                err_sum_x,err_sum_y = position.reset_error_sum_dir(dir,err_sum_x,err_sum_y)

        elif total_error<0.3:
            path.update_point()
    else:

        if total_error < 0.3:
            path.update_point()
            position.reset_error_sum_dir(dir,err_sum_x,err_sum_y)
    
    if reset_integral == True:
        position.reset_error_sum_crossed_line(dir,err_sum_x,err_sum_y)
    
    point = path.get_point()
    
    return point,err_sum_x,err_sum_y,index_end_point

def plot(x,y,Kp_x,Ki_x,Kd_x, y_to_plot_1, y_to_plot_2,y_to_plot_3,err_to_plot,y_to_plot):
        #Vector of simulation time used for plots
    t = np.linspace(0,120,len(y_to_plot))
    print(len(path._path))
        
    plt.figure()
    plt.plot(x,y,label = "Gräsklipparens simulerade trajektorie",c="orange")
    #plt.plot(x_ref,y_ref, label = "Önskad trajektorie")
    plt.plot([],[],' ',label="Kp = %i, Ki = %i, Kd = %.2f" %(Kp_x, Ki_x, Kd_x))
    plt.title("Trajektorieföljning")
    plt.legend(loc="upper left")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.show()
    plt.figure()

    min_error_x = min(y_to_plot_1)
    min_error_y = min(y_to_plot_2)
    min_error_circle = min(y_to_plot_3)
    max_error_x = max(y_to_plot_1)
    max_error_y = max(y_to_plot_2)
    max_error_circle = max(y_to_plot_3)

    print(max_error_x,max_error_y,max_error_circle,min_error_x,min_error_y,min_error_circle)
    plt.plot(t,err_to_plot)
    plt.title("Avvikelse från rutt")
    plt.xlabel("Simuleringstid [s]")
    plt.ylabel("Total avvikelse [m]")
    plt.legend(loc="upper left")
    plt.show()

main()