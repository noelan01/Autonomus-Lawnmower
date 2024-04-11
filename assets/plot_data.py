import json
import matplotlib.pyplot as plt

# JSON_OBJECT1 = "assets/data/following_path_08-03-24/x0_y2_dynamic_speed.json"
# JSON_OBJECT2 = "assets/data/following_path_08-03-24/ref_x0_y2_dynamic_speed.json"

# "../assets/data/2024_03_28_ChangedWheelIndex/path.json"


#JSON_OBJECT1 = "assets/data/10-04-24/50m-straight.json"
JSON_OBJECT1 = "rosbag2_2024_03_28-16_45_52/data.json"
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

    for key in kalman_data.keys():
        kalman_data[key].pop()

    # Extract x and y values from the data

    x_kalman =     [kalman_data[key][0] for key in kalman_data.keys()]
    y_kalman =     [kalman_data[key][1] for key in kalman_data.keys()]
    x_ref = [ref_data[key][0] for key in ref_data.keys()]
    y_ref = [ref_data[key][1] for key in ref_data.keys()]
    x_odo = [odometry_data[key][0] for key in odometry_data.keys()]
    y_odo = [odometry_data[key][1] for key in odometry_data.keys()]
    x_rtk = [rtk_data[key][0] for key in rtk_data.keys()]
    y_rtk = [rtk_data[key][1] for key in rtk_data.keys()]


    # Plotting
    plt.plot(x_kalman, y_kalman, label='kalman', color='blue')
    plt.plot(x_ref, y_ref, label='referens', color='orange')
    plt.plot(x_odo, y_odo, label='odometri', color='green')
    plt.plot(x_rtk, y_rtk, label='rtk', color='cyan')

    # Add legends and labels
    plt.legend()
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title('följning av referenslinje')

    # Show the plot
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