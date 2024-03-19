import numpy as np
import sys
import matplotlib.pyplot as plt

#glöm inte byta från öst till väst
#Göra en subscriber till hqv_mower-gnss_rtk-rel_enu och skriva till en fil (txt eller json)
#köra/gå  30 meter rakt fram

RTK_file = './calibration_data_rtk.txt'
IMU_file = './calibration_data_imu.txt'
ODO_file = './calibration_data_odometry.txt'

#--------------------FILE READERS------------------------

def RTK_file_reader(file):      #opens a txt file with RTK data and returns 2 lists, one with all north values and one with all east values
    with open(file, 'r',encoding='utf8') as f:     
        north_values = []
        east_values = []                        
        for line in f:
            if line.startswith('north:'):
                    north_value = float(line.split(': ')[1])
                    north_values.append(north_value)
            if line.startswith('east:'):
                    east_value = float(line.split(': ')[1])
                    east_values.append(east_value)
    return(north_values,east_values)

def IMU_file_reader(file):      #opens a txt file with IMU data and returns a list of all yaw values
    with open(file, 'r',encoding='utf8') as f:     
        yaw_values = []                       
        for line in f:
            if line.startswith('yaw:'):
                    yaw_value = float(line.split(': ')[1])
                    yaw_value =yaw_value % (2*np.pi)
                    yaw_values.append(yaw_value)
    return(yaw_values)

def ODO_file_reader(file):      #opens a txt file with odometry data and returns 3 lists, one with all x, one with all y values, one with heading (theta)
    x_list = []
    y_list = []
    theta_list = []
    with open(file, 'r') as f:
        lines = f.readlines()    
        for line in lines:
            if line.startswith("x"):
                x_list.append(float(line.split(":")[1].split(',')[0].strip()))
                y_list.append(float(line.split(":")[2].split(',')[0].strip()))                
                theta_list.append(float(line.split(":")[3].strip()))
    return x_list, y_list, theta_list

#---------------------------------------------------------------------------------

def linear_regression(x_values,y_values):
    x_np = np.array(x_values)
    y_np = np.array(y_values)
    slope,intercept = np.polyfit(x_np,y_np,1)
    return slope,intercept

#------------------------CALCULATE COORDINATE SYSTEMS OFFSET ANGLE-------------------------------------

def get_offset(x_start_rtk,y_start_rtk,x_end_rtk,y_end_rtk): 
    #---------EXCEPTIONS FOR SINGULARITIES----------
    if x_end_rtk == x_start_rtk:
        if y_start_rtk > y_end_rtk:
            rtk_heading = np.pi
        else:
            rtk_heading = 0
    elif y_end_rtk == y_start_rtk:
        if x_start_rtk > x_end_rtk:
            rtk_heading = 3*np.pi/2
        else:
            rtk_heading = np.pi/2
    #-----------------------------
    else: 
        slope = (y_end_rtk-y_start_rtk) / (x_end_rtk-x_start_rtk)
        if x_end_rtk>x_start_rtk:
            rtk_heading = np.arctan2(1, slope) 
        else:
            rtk_heading = np.arctan2(1,slope) + np.pi 
        
    rtk_east = np.pi/2
    offset_angle = rtk_heading-rtk_east
    print("RTK Heading: ", np.degrees(rtk_heading))
    return offset_angle

print(np.degrees(get_offset(0,0,-1,0)))
#------------------------CALCULATE IMU OFFSET-------------------------------------

#offset between IMU angle and RTK heading
def get_offset_imu():
    north,east = RTK_file_reader(RTK_file)      
    slope,intercept = linear_regression(east,north)
    if slope > 1:    
        rtk_heading = np.arctan2(1, slope) 
    else:
         rtk_heading = np.arctan2(1,slope) + np.pi
    rtk_heading = rtk_heading % (2 * np.pi)

    yaw_angles = IMU_file_reader(IMU_file)      #kan sättas manuellt med bara första och sista värdet ex yaw_angles = [pi,pi] (kom ihåg % (2*np.pi))
    yaw_heading = np.mean(yaw_angles)%(2*np.pi)
    
    offset_angle1 = abs(yaw_heading-rtk_heading)
    offset_angle2 = 2*np.pi-(abs(offset_angle1))
    
    return min(offset_angle1,offset_angle2)

#------------------------CALCULATE ODOMETRY OFFSET-------------------------------------

#offset between Odometry heading and RTK heading
def get_offset_odo():
    north,east = RTK_file_reader(RTK_file)      
    rtk_slope,rtk_intercept = linear_regression(east,north)
    if rtk_slope > 1:    
        rtk_heading = np.arctan2(1, rtk_slope) 
    else:
         rtk_heading = np.arctan2(1,rtk_slope) + np.pi
    rtk_heading = rtk_heading % (2 * np.pi)

    odo_x_values, odo_y_values, _ = ODO_file_reader(ODO_file)
    odo_slope,odo_intercept = linear_regression(odo_x_values,odometry_y_values)
    if odo_slope >1: 
        odo_heading = np.arctan2(1, odo_slope) 
    else: 
         odo_heading = np.arctan2(1, odo_slope) + np.pi  
    odo_heading = odo_heading % (2 * np.pi)

    offset_angle1 = abs(odo_heading-rtk_heading)
    offset_angle2 = 2*np.pi-(abs(offset_angle1))
    return min(offset_angle1,offset_angle2)

#------------------------ PLOT FUNCTIONS -------------------------------------

def plot_RTK_and_odometry_positions(RTK_north_values, RTK_east_values, odometry_x_values, odometry_y_values, imu_angle):
    slope_rtk,intercept_rtk = linear_regression(RTK_east_values,RTK_north_values)
    slope_odo,intercept_odo = linear_regression(odometry_x_values,odometry_y_values)

    #plt.scatter(RTK_east_values, RTK_north_values, label='RTK Positions', color='blue')
    #plt.plot(RTK_east_values, slope * np.array(RTK_east_values) + intercept, color='red', label='Linear Regression')
    plt.plot(RTK_east_values,RTK_north_values,label='RTK Positions', color='blue')

    translated_x = []
    translated_y = []
    for x_value in odometry_x_values:
        x_value = x_value - odometry_x_values[0] + RTK_east_values[0]
        translated_x.append(x_value)
    for y_value in odometry_y_values:
        y_value = y_value - odometry_y_values[0] +  RTK_north_values[0]
        translated_y.append(y_value)
    #plt.plot(odometry_x_values, slope_odo * np.array(odometry_x_values) + intercept_odo, color='red', label='Linear Regression')
    plt.plot(translated_x, translated_y, label='Odometry Positions', color='orange')

    if 0 <= imu_angle <= np.pi/2:
        imu_x = 40 * np.sin(imu_angle) + RTK_east_values[0]
        imu_y = 40 * np.cos(imu_angle) + RTK_north_values[0]
    elif np.pi/2 < imu_angle <= np.pi:
        imu_x = 40 * np.sin(np.pi-imu_angle) + RTK_east_values[0]
        imu_y = 40 * -np.cos(np.pi-imu_angle) + RTK_north_values[0]
    elif np.pi < imu_angle <= 3*np.pi/2:
        imu_x = 40 * -np.cos(3*np.pi/2-imu_angle) + RTK_east_values[0]
        imu_y = 40 * -np.sin(3*np.pi/2-imu_angle) + RTK_north_values[0]
    elif 3*np.pi/2 < imu_angle <= 2*np.pi:
        imu_x = 40 * -np.sin(2*np.pi-imu_angle) + RTK_east_values[0]
        imu_y = 40 * np.cos(2*np.pi-imu_angle) + RTK_north_values[0]
         
    plt.plot([RTK_east_values[0],imu_x],[RTK_north_values[0],imu_y], label='IMU direction', color='black')

    plt.xlabel('East')
    plt.ylabel('North')
    plt.legend()
    plt.show()
 
RTK_north_values, RTK_east_values = RTK_file_reader(RTK_file)
odometry_x_values, odometry_y_values, _ = ODO_file_reader(ODO_file)
imu_angle = IMU_file_reader(IMU_file)
imu_angle = np.mean(imu_angle)%(2*np.pi)


#plot_RTK_and_odometry_positions(RTK_north_values, RTK_east_values, odometry_x_values,odometry_y_values,imu_angle)

