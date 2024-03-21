import numpy as np
import sys
import matplotlib.pyplot as plt


#wheelcount0_05 = '\assets\data\05-03-24\linear05\wheelcounter0_linear05.txt'
wheelcount0_05 = 'wheelcounter0_linear05.txt'
wheelcount0_1 = './wheelcounter0_linear1.txt'

def read_data(filename):
    data_dict = {}
    prev_counter = None
    with open(filename, 'r') as file:
        current_key = None
        current_value = None
        for line in file:
            line = line.strip()
            if line.startswith('sec:'):
                current_key = line.split(':')[1].strip()
            elif line.startswith('nanosec:'):
                current_key += line.split(':')[1].strip()
            elif line.startswith('counter:'):
                counter_value = int(line.split(':')[1].strip())
                if prev_counter is None or counter_value > prev_counter:
                    data_dict[current_key] = counter_value
                    prev_counter = counter_value
                else:
                    break  # Stop reading if counter doesn't increase
    return data_dict



time_tics_dict0_05 = read_data(wheelcount0_05)

def get_speed(dict):
    start_t = list(dict)[0]
    end_t = list(dict)[-1]
    delta_t = (int(end_t)-int(start_t))/10**9

    delta_count = dict[end_t] - dict[start_t]
    distance = delta_count*0.751/349
    speed = distance/delta_t
    print(speed)
    return start_t,end_t, distance,speed



def plot_speed(data_dict, label):
    start_t, end_t, distance, speed = get_speed(data_dict)

    plt.plot([0, int(end_t)-int(start_t)], [0, distance], label=f'Speed: {speed:.2f} m/s for {label}', color='black')
    plt.xlabel('Time [s]')
    plt.ylabel('Distance [m/s]')
    plt.legend()
    plt.show()


time_tics_dict0_05 = read_data(wheelcount0_05)

plot_speed(time_tics_dict0_05, '0.5 linear')




