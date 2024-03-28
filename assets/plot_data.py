import json
import matplotlib.pyplot as plt

# JSON_OBJECT1 = "assets/data/following_path_08-03-24/x0_y2_dynamic_speed.json"
# JSON_OBJECT2 = "assets/data/following_path_08-03-24/ref_x0_y2_dynamic_speed.json"

# "../assets/data/2024_03_28_ChangedWheelIndex/path.json"


JSON_OBJECT1 = "../assets/data/2024_03_28_ChangedWheelIndex/diag_path.json"
JSON_OBJECT2 = "../assets/data/2024_03_28_ChangedWheelIndex/ref_diag_path.json"

def plot_data(file1, file2):
    # Load data from JSON files
    with open(file1, 'r') as f1:
        data1 = json.load(f1)
    
    # Remove yaw angle
    for key in data1.keys():
        data1[key].pop()

    with open(file2, 'r') as f2:
        data2 = json.load(f2)

    # Extract x and y values from the data

    x =     [data1[key][0] for key in data1.keys()]
    y =     [data1[key][1] for key in data1.keys()]
    x_ref = [data2[key][0] for key in data2.keys()]
    y_ref = [data2[key][1] for key in data2.keys()]

    # Plotting
    plt.plot(x, y, label='Measured path', color='blue')
    plt.plot(x_ref, y_ref, label='Reference path', color='orange')

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
    file2_path = JSON_OBJECT2

    plot_data(file1_path, file2_path)