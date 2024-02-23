#referensstationen är satt som origo i ett fast globalt koordninatsystem på gräsmattan. x pekar norr, y pekar väst
#Gräsklipparen är satt som ett dynamiskt lokalt koordinatsystem, x pekar frammåt, y pekar vänster
#topic hqv_mower/gnss_rtk/rel_enu ger gräsklipparens position i norr/syd relativ referensstation, mower_xy_global
import numpy as np

yaw_angle = 0
yaw_angle = yaw_angle % (2*np.pi)
offset_angle = 0    #offset_angle skillnaden i vinkel mellan norr i globala systemet och IMU, positiv medurs(fås från kalibrering)
#Kalibrering kan fås genom att köra rakt fram och se ta ut rtk data, trigonometri kan ge vinkeln i det globala systemet som
#mowern kör i. IMU vinkeln jämförs och en offset kam fås ut
def rotation_matrix(angle):
    return np.array([
        [np.cos(angle),  -np.sin(angle), 0],
        [np.sin(angle),  np.cos(angle), 0],
        [0,            0,           1] 
    ])
  

def goal_local_to_global(offset_angle,goal_xy_local):       #tar in Målkordinater från roboten i x/y
    #nedan fås från topics
    yaw_angle = 0                           #IMU som fås från roboten, kanske måste göras om till att gå mellan 0 och 2pi
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
    yaw_angle = 0           #IMU som fås från roboten, kanske måste göras om till att gå mellan 0 och 2pi
    theta = yaw_angle - offset_angle

    goal_xy_local = np.dot(inverse_rotation_matrix(theta),goal_xy_global)       #målkordinater i x,y relativt roboten
    return goal_xy_local







