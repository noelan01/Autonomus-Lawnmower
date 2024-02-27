import numpy as np
import sys
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Arc
from matplotlib.lines import Line2D
#glöm inte byta från öst till väst

#Göra en subscriber till hqv_mower-gnss_rtk-rel_enu och skriva till en fil (txt eller json)
#köra/gå  30 meter rakt fram


RTK_file = './calibration_data_rtk.txt'
IMU_file = './calibration_data_imu.txt'

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

def IMU_file_reader(file):      #opens a txt file with RTK data and returns 2 lists, one with all north values and one with all east values
    with open(file, 'r',encoding='utf8') as f:     
        yaw_values = []                       
        for line in f:
            if line.startswith('yaw:'):
                    yaw_value = float(line.split(': ')[1])
                    yaw_value =yaw_value % (2*np.pi)
                    yaw_values.append(yaw_value)

    return(yaw_values)

def get_gnss_heading(north_values,east_values):
    north_mean = np.mean(north_values)
    east_mean = np.mean(east_values)
    #1a/4e kvadranten
    if east_mean >= 0:
        gnss_heading = np.arctan2(east_mean,north_mean)
        return gnss_heading

    #2a/3e kvadranten
    if east_mean <0:
        gnss_heading = np.arctan2(east_mean,north_mean)
        return (gnss_heading + 2*np.pi)
    

def get_offset():
    north = RTK_file_reader(RTK_file)[0]        #kan sättas manuellt med bara första och sista värdet ex north = [0,10]
    east = RTK_file_reader(RTK_file)[1]
    gnss_heading = get_gnss_heading(north,east)

    yaw_angles = IMU_file_reader(IMU_file)      #kan sättas manuellt med bara första och sista värdet ex yaw_angles = [pi,pi] (kom ihåg % (2*np.pi))
    yaw_heading = np.mean(yaw_angles)

    offset_angle = gnss_heading-yaw_heading
    print(gnss_heading)
    print(yaw_heading)
    return offset_angle

    
print(get_offset())


def plot_IMU_RTK():
    north = RTK_file_reader(RTK_file)[0]        
    east = RTK_file_reader(RTK_file)[1]
    

'''plt.plot(east_values,north_values)
plt.xlabel('väst')
plt.ylabel('norr')
plt.show()'''








'''
def get_angle_plot(line1, line2, offset=1, color=None, origin=[0, 0], len_x_axis=1, len_y_axis=1):
    l1xy = line1.get_xydata()

    # Angle between line1 and x-axis
    slope1 = (l1xy[1][1] - l1xy[0][1]) / float(l1xy[1][0] - l1xy[0][0])
    angle1 = abs(math.degrees(math.atan(slope1)))  # Taking only the positive angle

    l2xy = line2.get_xydata()

    # Angle between line2 and x-axis
    slope2 = (l2xy[1][1] - l2xy[0][1]) / float(l2xy[1][0] - l2xy[0][0])
    angle2 = abs(math.degrees(math.atan(slope2)))

    theta1 = min(angle1, angle2)
    theta2 = max(angle1, angle2)

    angle = theta2 - theta1

    if color is None:
        color = line1.get_color()  # Uses the color of line 1 if color parameter is not passed.

    return Arc(origin, len_x_axis * offset, len_y_axis * offset, angle=0, theta1=theta1, theta2=theta2, color=color, label=str(angle) + u"\u00b0")


def get_angle_text(angle_plot):
    angle = angle_plot.get_label()[:-1] # Excluding the degree symbol
    angle = "%0.2f"%float(angle)+u"\u00b0" # Display angle up to 2 decimal places

    # Get the vertices of the angle arc
    vertices = angle_plot.get_verts()

    # Get the midpoint of the arc extremes
    x_width = (vertices[0][0] + vertices[-1][0]) / 2.0
    y_width = (vertices[0][1] + vertices[-1][1]) / 2.0

    separation_radius = max(x_width/2.0, y_width/2.0)

    return [x_width + separation_radius, y_width + separation_radius, angle]

fig = plt.figure()

line_1 = Line2D([0,1], [0,4], linewidth=1, linestyle = "-", color="green")
line_2 = Line2D([0,4.5], [0,3], linewidth=1, linestyle = "-", color="red")

ax = fig.add_subplot(1,1,1)

ax.add_line(line_1)
ax.add_line(line_2)

angle_plot = get_angle_plot(line_1, line_2, 1)
angle_text = get_angle_text(angle_plot) 
# Gets the arguments to be passed to ax.text as a list to display the angle value besides the arc

ax.add_patch(angle_plot) # To display the angle arc
ax.text(*angle_text) # To display the angle value

ax.set_xlim(0,7)
ax.set_ylim(0,5)

plt.legend()
plt.show()'''