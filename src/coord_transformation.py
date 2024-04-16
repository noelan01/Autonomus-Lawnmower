#referensstationen är satt som origo i ett fast globalt koordninatsystem på gräsmattan. y pekar norr, x pekar öst

import numpy as np
#import calibration_offset


#offset_angle = calibration_offset.get_offset()
#offset_angle = calibration_offset.get_offset_odo


def rotation_matrix(angle):
    return np.array([
        [np.cos(angle), -np.sin(angle)],
        [np.sin(angle),  np.cos(angle)],
    ])

def pos_global_to_local(x_rtk,y_rtk, x_init_rtk,y_init_rtk,offset_angle):

    translated = np.array([x_rtk,y_rtk]) - np.array([x_init_rtk,y_init_rtk])
    rotated = np.dot(rotation_matrix(offset_angle),translated)  
    pos_xy_local = rotated
    return pos_xy_local[0],pos_xy_local[1]


#print(pos_global_to_local(-8.47,-4.79,-5.97,-1.66,2.22))





'''
def rotation_matrix(angle):
    return np.array([
        [np.cos(angle), -np.sin(angle), 0],
        [np.sin(angle),  np.cos(angle), 0],
        [0,              0,             1]
    ])
  

def goal_local_to_global(x_goal,y_goal,x_start,y_start,offset_angle):       #tar in Målkordinater från roboten i x/y
    #nedan fås från topics
    yaw_angle = 2*np.pi + np.arccos(4/5)                    #IMU som fås från roboten
    yaw_angle = yaw_angle % (2*np.pi)       #Göras om till att gå mellan 0 och 2pi istället för -pi till pi
    theta = yaw_angle - offset_angle        #Rotationsvinkel (skillnad mellan norr och heading) Används i rotationsmatris

    rotated = np.dot(rotation_matrix(theta),np.array([x_goal,y_goal,1]))     #målkordinater i norr/syd relativt referensstation
    goal_xy_global = rotated + np.array([x_start,-y_start,0])
    return goal_xy_global*[1,-1,1]      #[1,-1,1] för att få väst istället för öst


#robotens globala position har vi från topic


def inverse_rotation_matrix(angle):
    return np.array([
        [np.cos(angle),  np.sin(angle),0], 
        [-np.sin(angle), np.cos(angle),0],
        [0,              0,            0] 
    ])

def goal_global_to_local(x_goal,y_goal,x_start,y_start,offset_angle):      #tar in Målkordinater från referensstation i norr/väst. x_start,y_start är nuvarande pos i globala
    yaw_angle = 0          #IMU som fås från roboten
    yaw_angle = yaw_angle % (2*np.pi)       #Görs om till att gå mellan 0 och 2pi istället för -pi till pi
    theta = yaw_angle - offset_angle

    rotated = np.dot(inverse_rotation_matrix(theta),np.array([x_goal,y_goal,1]))       #målkordinater i x,y relativt roboten
    goal_xy_local = rotated -[x_start,y_start,0]
    return goal_xy_local




'''
