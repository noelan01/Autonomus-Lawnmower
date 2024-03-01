
def regulation():
    import math
    import time
    import numpy as np
    import matplotlib.pyplot as plt
    import cmath
    import random
    

    #Starting with defining variables for the robot
    D = 0.4
    r = 0.752/(2*math.pi)
    L = (43/2+3.2/2)/100
    
    #Other variables
    x = [0]
    y = [0]
    theta = [0]
    delta_x = [0]
    delta_y = [0]
    delta_xe = [0]
    delta_ye = [0]

    x_ref = [0]
    y_ref = [0]

    x_base = [0]
    y_base = [0]
    Kp = 20
    Ts = 0.025
    acc_sum_delta_omega_1 = [0]
    acc_sum_delta_omega_2 = [0]    
    theta_1_meas = [0]
    theta_2_meas = [0]
    theta_1_increment = [0]
    theta_2_increment = [0]
    delta_theta_1 = [0]
    delta_theta_2 = [0]

    delta_omega = [0]
    delta_S = [0]
    delta_s = [0]
    delta_theta = [0]
    delta_omega1 = [0]
    delta_omega2 = [0]
    PPR = 349
    x_printa = [0]*100
    
    #Defining simulation time
    simTime = 20
    nrOfSteps = simTime/Ts

    rand_vec = np.random.rand(800,1)
    test = random.random()/1000

    print(Kp)

    for k in range(1,800):
    #Updating x_ref
        x_ref.append(x_ref[k-1]+0.02)
        y_ref.append(y_ref[k-1])
        #Implementing the kinematic model of the robot
        delta_xe.append(x_ref[k] - x[k-1])
        delta_ye.append(y_ref[k] - y[k-1])

        #Increasing the error with the proportional gain
        delta_x.append(delta_xe[k]*Kp)
        delta_y.append(delta_ye[k]*Kp)

        #Calculating delta_omega(k) and delta_S(k)
        delta_omega.append(cmath.asin((delta_x[k]*math.sin(theta[k-1])-delta_y[k]*math.cos(theta[k-1]))/D).real)
        delta_S.append(D*math.cos(delta_omega[k])-D+delta_x[k]*math.cos(theta[k-1])+delta_y[k]*math.sin(theta[k-1]))

        #Calculating delta_omega1(k) and delta_omega2(k)
        delta_omega1.append(1/r*(delta_S[k]+L*delta_omega[k]))
        delta_omega2.append(1/r*(delta_S[k]-L*delta_omega[k]))

        #Discrete time integration using backwards Euler
        acc_sum_delta_omega_1.append(acc_sum_delta_omega_1[k-1] + Ts*delta_omega1[k])
        acc_sum_delta_omega_2.append(acc_sum_delta_omega_2[k-1] + Ts*delta_omega2[k]) 

        #Measure the difference between the old wheel counter and the accumulated sum to find angular displacement
        #theta_1_increment = acc_sum_delta_omega_1-theta_1_meas
        #theta_2_increment =  acc_sum_delta_omega_2-theta_2_meas

        #Calculating the needed angular velocity of each wheel
        #dtheta1_dt = theta_1_increment/Ts
        #dtheta2_dt = theta_2_increment/Ts

        #Converting to linear and angular movement of the robot
        #lin_vel = r/2*(dtheta1_dt+dtheta2_dt)
        #ang_vel = r/(2*L)*(dtheta1_dt-dtheta2_dt)

        #Publish angular and linear velocity to the lawnmower

        #Publish lin_vel
        #Publish ang_vel

        #Sleep for Ts = 0.025 s
        #time.sleep(0.025)

        #wheel_1_counter = #Subscribe to HQV topic /hqv_mower/wheel0/speed
        #wheel_2_counter = #Subscribe to HQV topic /hqv_mower/wheel1/speed

        #Convert to angular displacement
        #theta_1_meas = wheel_1_counter*2*math.pi/PPR
        #theta_2_meas = wheel_2_counter*2*math.pi/PPR

        theta_1_meas.append(acc_sum_delta_omega_1[k]-random.random()/10)
        theta_2_meas.append(acc_sum_delta_omega_2[k]-random.random()/10)

        #Calculating the angular difference between the two samples
        delta_theta_1.append(theta_1_meas[k]-theta_1_meas[k-1])
        delta_theta_2.append(theta_2_meas[k]-theta_2_meas[k-1])     

        delta_s.append(r/2*(delta_theta_1[k]+delta_theta_2[k]))
        delta_theta.append(r/(2*L)*(delta_theta_1[k]-delta_theta_2[k])) 

        #Updating the robots base position
        x_base.append(x_base[k-1] +delta_s[k]*math.cos(theta[k-1]))
        y_base.append(y_base[k-1] + delta_s[k]*math.sin(theta[k-1]))
        theta.append(theta[k-1] + delta_theta[k])

        #Updating the chalking mechanism position
        x.append(x_base[k] - D*math.cos(theta[k]) + D) 
        y.append(y_base[k] - D*math.sin(theta[k])) 
    #print(x)
    print(y)
    plt.figure()
    plt.plot(x,y)
    plt.show()

regulation()





    



    