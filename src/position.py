
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


    
#Importing the Kalman filter code
import kalman
state_estimation = kalman.EKF(0)

#This is the code that simulates the lawnmower
def simulation():
    #Starting with defining variables of the robot

    r = 0.752/(2*math.pi)
    L = (43/2+3.2/2)/100
    D = 0.4
    
    #Other variables
    x = [0]
    y = [0]
    x_kalman = [0]
    y_kalman = [0]

    theta =[-math.pi]
    delta_x = [0]
    delta_y = [0]
    delta_xe = [0]
    delta_ye = [0]

    x_ref = [0]
    y_ref = [0]

    x_base = [0]
    y_base = [0]
    x_base_kalman = [0]
    y_base_kalman = [0]
    x_error = [0]
    y_error = [0]
    theta_kalman =[0]
    Kp = 15
    Ki = 10
    Kd = 0.01
    Ts = 0.1
    acc_sum_delta_omega_1 = [0]
    acc_sum_delta_omega_2 = [0]    
    theta_1_meas = [0]
    theta_2_meas = [0]
    theta_1_increment = [0]
    theta_2_increment = [0]
    delta_theta_1 = [0]
    delta_theta_2 = [0]
    dtheta1_dt = [0]
    dtheta2_dt = [0]
    lin_vel = [0]
    ang_vel = [0]
    err_sum_x = 0
    err_sum_y = 0
    s = [0]
    tot_error = 0

    delta_omega = [0]
    delta_S = [0]
    delta_s = [0]
    delta_theta = [0]
    delta_omega1 = [0]
    delta_omega2 = [0]
    PPR = 349
    
    
    #Defining simulation time
    simTime = 100
    nrOfSteps = int(simTime/Ts)
    
    #load pitch data from json file
    route.load_pitch_data()

    #Outer lines
    route.outerLines(path)

    #Lower penalty area
    route.lowerPenaltyArea(path)
    route.lowerArc(path)
    
    #Lower goal area
    route.lowerGoalArea(path)

    #Transport path to reach mid line
    route.transportMidLine(path)

    #Midline and mid circle
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

    #The first iteration of the while loop, k needs to equal 1
    k = 1
    #Vector of simulation time used for plots
    #t = np.linspace(0,simTime,len(path._path))

    while reached_goal == False:

        #Updating the point when we are close enough to the previous point
        path.update_point()

        next_point = path.get_point()
        x_ref.append(next_point[0])
        y_ref.append(next_point[1])

        #When there are no more points in the path planner list, we end the simulation
        if next_point[0] == None or next_point[1] == None:
            reached_goal = True
            break

        #Implementing the kinematic model of the robot
        delta_xe.append(x_ref[k] - x[k-1])
        delta_ye.append(y_ref[k] - y[k-1])

        err_sum_x = err_sum_x + delta_xe[k]
        err_sum_y = err_sum_y + delta_ye[k] 
        
        #Increasing the error with the PID-controller
        delta_x.append(delta_xe[k]*Kp+Ki*Ts*err_sum_x+Kd*(delta_xe[k]-delta_xe[k-1])/Ts)
        delta_y.append(delta_ye[k]*Kp+Ki*Ts*err_sum_y+Kd*(delta_ye[k]-delta_ye[k-1])/Ts)

        #Calculating delta_omega(k) and delta_S(k)
        delta_omega.append(cmath.asin((delta_x[k]*math.sin(theta[k-1])-delta_y[k]*math.cos(theta[k-1]))/D).real)
        delta_S.append(D*math.cos(delta_omega[k])+D+delta_x[k]*math.cos(theta[k-1])+delta_y[k]*math.sin(theta[k-1]))

        #Calculating delta_omega1(k) and delta_omega2(k)
        delta_omega1.append(1/r*(delta_S[k]+L*delta_omega[k]))
        delta_omega2.append(1/r*(delta_S[k]-L*delta_omega[k]))
        
        #Calculating the needed angular velocity of each wheel

        dtheta1_dt.append(-1*(delta_omega1[k]))
        dtheta2_dt.append(-1*(delta_omega2[k]))

        #Clamping the angular velocity of the wheels to be more representable of the real lawnmower
        if dtheta1_dt[k]>6.5:
            dtheta1_dt[k] = 6.5
        elif dtheta1_dt[k]<-6.5:
            dtheta1_dt[k] = -6.5
        if dtheta2_dt[k]>6.5:
            dtheta2_dt[k] =6.5
        elif dtheta2_dt[k]<-6.5:
            dtheta2_dt[k] = -6.5
            
        #Calculating the steering variable to see the output
        s.append((dtheta1_dt[k]-dtheta2_dt[k])/dtheta1_dt[k])

        #Converting to linear and angular movement of the robot to use in the Kalman filter
        lin_vel.append(r/2*(dtheta1_dt[k]+dtheta2_dt[k]))
        ang_vel.append(r/(2*L)*(dtheta1_dt[k]-dtheta2_dt[k]))

        #The random noise was calculated by finding the resolution of the lawnmower (360/PPR) and estimating that a reasonable error would be if the robot misses a step or reports back a too high or low step
        rand1 = random.uniform(0,2*math.pi/PPR) + random.uniform(0,0.01)
        rand2 = random.uniform(0,2*math.pi/PPR) + random.uniform(0,0.01)
        rand3 = random.uniform(-360/PPR,360/PPR) + random.uniform(-0.001,0.001)

        theta_1_meas.append(theta_1_meas[k-1]+(-1*dtheta1_dt[k]*Ts)+rand1)
        theta_2_meas.append(theta_2_meas[k-1]+(-1*dtheta2_dt[k]*Ts)+rand2)

        #Calculating the angular difference between the two samples
        delta_theta_1.append(theta_1_meas[k]-theta_1_meas[k-1])
        delta_theta_2.append(theta_2_meas[k]-theta_2_meas[k-1])     

        delta_s.append(r/2*(delta_theta_1[k]+delta_theta_2[k]))
        delta_theta.append(r/(2*L)*(delta_theta_1[k]-delta_theta_2[k])) 

        #Updating the robots base position
        x_base.append(x_base[k-1] +delta_s[k]*math.cos(theta[k-1]))
        y_base.append(y_base[k-1] + delta_s[k]*math.sin(theta[k-1]))
        theta.append(theta[k-1] + delta_theta[k])

        #Variables to the Kalman function
        Z_k = np.array([[x_base[k]],[y_base[k]],[theta[k]]])
        control_inputs = np.array([[lin_vel[k]],[ang_vel[k]]])
        sensor_error = np.array([[rand1],[rand2],[rand3]])
        no_sensor_error = np.array([[0],[0],[0]])
        #Filtering with the Kalman filter to get a better estimation of the position
        state = state_estimation.update(Z_k,control_inputs,sensor_error)
        
        x_base_kalman.append(state[0].item())
        y_base_kalman.append(state[1].item())
        theta_kalman.append(state[2].item())

        #Updating the chalking mechanism position
        x.append(x_base[k] - D*math.cos(theta[k])) 
        y.append(y_base[k] - D*math.sin(theta[k])) 

        x_kalman.append(x_base_kalman[k] - D*math.cos(theta_kalman[k]))
        y_kalman.append(y_base_kalman[k]-D*math.sin(theta[k]))
        #print(x[k]-x_kalman[k])
        #print(y-y_kalman)

        x_error.append(x[k]-x_ref[k])
        y_error.append(y[k]-y_ref[k])
        tot_error = math.sqrt(x_error[k]**2+y_error[k]**2)
        k += 1
        
    #print(lin_vel)
    #print(delta_ye)

    #plt.figure()
    plt.plot(x,y,label = "Actual trajectory")
    #plt.plot(x_ref,y_ref, label = "Desired trajectory")
    #plt.plot([],[],' ',label="Kp = %i, Ki = %i, Kd = %.2f" %(Kp, Ki, Kd))
    plt.title("Trajectory following")
    plt.legend(loc="upper left")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.show()
    #plt.figure()
    #plt.plot(t,tot_error,label="Total error")
    #plt.title("Error measurement")
    #plt.xlabel("Simulation time [s]")
    #plt.ylabel("Total error [m]")
    #plt.legend(loc="upper left")
    #plt.figure()
    #plt.plot(theta)
    #plt.ylabel("Theta [rad]")
    #plt.xlabel("Number of samples")
simulation()



def rotate90deg(theta, r, L,):
    dtheta1_dt = 0.1
    dtheta2_dt = -0.1
    
