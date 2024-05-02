import json
import matplotlib.pyplot as plt
import numpy as np
import sys

sys.path.append('../src/')

import path_planner
path = path_planner.Path()

path.set_path(0, 0, 0, 2, 60,"y")
path.set_path(0, 2, 2, 2, 60,"x")
path.set_path(2, 2, 2, 0, 60,"-y")
path.set_path(2, 0, 0, 0,60,"-x")
x_ref_plot = []
y_ref_plot = []

# JSON_OBJECT1 = "assets/data/following_path_08-03-24/x0_y2_dynamic_speed.json"
# JSON_OBJECT2 = "assets/data/following_path_08-03-24/ref_x0_y2_dynamic_speed.json"

# "../assets/data/2024_03_28_ChangedWheelIndex/path.json"

JSON_OBJECT1 = "RTK_kvadrat_60ppm/Kvadrat_60ppm.json"
err_to_plot=[]
#JSON_OBJECT2 = "assets/data/2024_03_28_Mossen_rtk/ref_path_straight_line_50_2.json"
#JSON_OBJECT3 = "assets/data/2024_03_28_Mossen_rtk/ref_path_straight_line_50_odometry_2.json"

def plot_data(file1):
    # Load data from JSON files
    with open(file1, 'r') as f1:
        data1 = json.load(f1)
    
    # Remove yaw angle
    kalman_data = data1["kalman"]
    ref_data = data1["ref"]
    odometry_data = data1["odometry"]
    rtk_data = data1["rtk"]
    error_data = data1["error"]

    for key in kalman_data.keys():
        kalman_data[key].pop()

    # Extract x and y values from the data

    x_kalman =     [kalman_data[key][0] for key in kalman_data.keys()]
    y_kalman =     [kalman_data[key][1] for key in kalman_data.keys()]
    dir_kalman = [ref_data[key][2] for key in ref_data.keys()]
    x_ref = [ref_data[key][0] for key in ref_data.keys()]
    y_ref = [ref_data[key][1] for key in ref_data.keys()]

    x_odo = [odometry_data[key][0] for key in odometry_data.keys()]
    y_odo = [odometry_data[key][1] for key in odometry_data.keys()]
    x_rtk = [rtk_data[key][0] for key in rtk_data.keys()]
    y_rtk = [rtk_data[key][1] for key in rtk_data.keys()]
    x_error = [error_data[key][0] for key in error_data.keys()]
    y_error = [error_data[key][1] for key in error_data.keys()]

    for i in range(len(error_data)):
        if dir_kalman[i] == "x" or dir_kalman[i] == "-x":
            err_to_plot.append(abs(y_error[i]))
        else:
            err_to_plot.append(abs(x_error[i]))

    for k in range(len(path._path)):
        x_ref_plot.append(path._path[k][0])
        y_ref_plot.append(path._path[k][1]) 
    
    
    print("The mean absolute error is: ",np.mean(err_to_plot))
    # Plotting
    plt.figure()
    plt.plot(x_kalman, y_kalman, label='kalman', color='blue')
    plt.plot(x_ref_plot, y_ref_plot, label='referens', color='orange')
    plt.plot(x_odo, y_odo, label='odometri', color='green')
    plt.plot(x_rtk, y_rtk, label='rtk', color='cyan')


    # Add legends and labels
    plt.legend(loc="upper left")
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')

    ax = plt.gca()
    ax.set_aspect('equal', adjustable='box')
    
    plt.title('följning av referenslinje')

    
    # Show the plot
    plt.show()

    
    plt.figure()
    plt.xlabel("Sampling")
    plt.ylabel("Total avvikelse [m]")
    plt.plot(err_to_plot,label="Total avvikelse")
    plt.legend(loc="upper left")
    plt.title("Avvikelse vid cirkelkörning")
    #ax = plt.gca()
    #ax.set_aspect('equal', adjustable='box')
    plt.show()
    

def plot_data_with_odo(file1, file2, file3):
    # Load data from JSON files
    with open(file1, 'r') as f1:
        data1 = json.load(f1)   
    
    # Remove yaw angle
    for key in data1.keys():
        data1[key].pop()

    with open(file2, 'r') as f2:
        data2 = json.load(f2)
    
    with open(file3, 'r') as f3:
        data3 = json.load(f3)

    # Extract x and y values from the data

    x =     [data1[key][0] for key in data1.keys()]
    y =     [data1[key][1] for key in data1.keys()]
    x_ref = [data2[key][0] for key in data2.keys()]
    y_ref = [data2[key][1] for key in data2.keys()]

    x_odo = [data3[key][0] for key in data3.keys()]
    y_odo = [data3[key][1] for key in data3.keys()]

    # Plotting
    plt.plot(x, y, label='RTK path', color='blue')
    plt.plot(x_ref, y_ref, label='Reference path', color='orange')
    plt.plot(x_odo, y_odo, label='Odometry path', color='red')

    # Add legends and labels
    plt.legend()
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title('Lawnmower following reference line')

    # Show the plot
    plt.show()

if __name__ == "__main__":
    # Replace 'file1.json' and 'file2.json' with your actual file paths
    file1_path = JSON_OBJECT1
    #file2_path = JSON_OBJECT2
    #file3_path = JSON_OBJECT3

    plot_data(file1_path)

    #plot_data_with_odo(file1_path,file2_path,file3_path)