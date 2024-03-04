#referensstationen är satt som origo i ett fast globalt koordninatsystem på gräsmattan. x pekar norr, y pekar väst
#Gräsklipparen är satt som ett dynamiskt lokalt koordinatsystem, x pekar frammåt, y pekar vänster
#topic hqv_mower/gnss_rtk/rel_enu ger gräsklipparens position i norr/öst relativ referensstation, mower_xy_global. Tänk på att göra öst till väst
import numpy as np
import calibration_offset

yaw_angle = 0
offset_angle = calibration_offset.get_offset_imu()
#offset_angle = calibration_offset.get_offset_odo


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






#-------------- If RTK doesn't work and we need to use robots GNSS --------------
def convert_to_xy(lat, lon, lat_start, lon_start):
    x = (lat - lat_start) * 111139
    y = (lon - lon_start) * 111139
    return x, y 

def mower_gnss_to_global():
    pass




print(goal_local_to_global(5,0,3,4,0))
print(goal_global_to_local(5,4,3,3,0))




