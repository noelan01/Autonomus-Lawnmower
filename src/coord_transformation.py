#referensstationen är satt som origo i ett fast globalt koordninatsystem på gräsmattan. x pekar norr, y pekar väst
#Gräsklipparen är satt som ett dynamiskt lokalt koordinatsystem, x pekar frammåt, y pekar vänster
#topic hqv_mower/gnss_rtk/rel_enu ger gräsklipparens position i norr/öst relativ referensstation, mower_xy_global. Tänk på att göra öst till väst
import numpy as np

yaw_angle = 0
yaw_angle = yaw_angle % (2*np.pi)
offset_angle = np.pi/4    #offset_angle skillnaden i vinkel mellan norr i globala systemet och IMU, mmellan -pi och pi?(fås från kalibrering)
#Kalibrering kan fås genom att köra rakt fram eller gå med den och se ta ut rtk data, trigonometri kan ge vinkeln i det globala systemet som
#mowern kör i. IMU vinkeln jämförs och en offset kam fås ut
def rotation_matrix(angle):
    return np.array([
        [np.cos(angle),  -np.sin(angle), 0],
        [np.sin(angle),  np.cos(angle), 0],
        [0,            0,           1] 
    ])
  

def goal_local_to_global(offset_angle,goal_xy_local):       #tar in Målkordinater från roboten i x/y
    #nedan fås från topics
    yaw_angle = np.pi/4                     #IMU som fås från roboten
    yaw_angle = yaw_angle % (2*np.pi)       #Göras om till att gå mellan 0 och 2pi istället för -pi till pi
    theta = yaw_angle - offset_angle        #Rotationsvinkel (skillnad mellan norr och heading) Används i rotationsmatris


    goal_xy_global = np.dot(rotation_matrix(theta),goal_xy_local)       #målkordinater i norr/syd relativt referensstation
    return goal_xy_global


#robotens globala position har vi från topic


def inverse_rotation_matrix(angle):
    return np.array([
        [np.cos(angle),  np.sin(angle), 0], 
        [-np.sin(angle), np.cos(angle), 0], 
        [0,            0,            1] 
    ])

def goal_global_to_local(offset_angle,goal_xy_global):      #tar in Målkordinater från referensstation i norr/väst
    yaw_angle = np.pi/4           #IMU som fås från roboten
    yaw_angle = yaw_angle % (2*np.pi)       #Göras om till att gå mellan 0 och 2pi istället för -pi till pi
    theta = yaw_angle - offset_angle

    goal_xy_local = np.dot(inverse_rotation_matrix(theta),goal_xy_global)       #målkordinater i x,y relativt roboten
    return goal_xy_local






#-------------- If RTK doesn't work and we need to use robots GNSS --------------
def convert_to_xy(lat, lon, lat_start, lon_start):
    x = (lat - lat_start) * 111139
    y = (lon - lon_start) * 111139
    return x, y 

def mower_gnss_to_global():
    pass




print(goal_local_to_global(9.899999999962716e-05,np.array([3,5,0])))





