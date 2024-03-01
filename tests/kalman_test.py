import json
import sys
import numpy as np

sys.path.append('./src/')

import sub_data
import kalman

def distance_calc(states):
    init_coord = {"lat": states[0][0].item()*np.pi/180,
                  "lon": states[0][1].item()*np.pi/180}
    
    latest_coord = {"lat": states[-1][0].item()*np.pi/180,
                    "lon": states[-1][1].item()*np.pi/180}
    
    delta_lat = init_coord["lat"] - latest_coord["lat"]
    delta_lon = init_coord["lon"] - latest_coord["lon"]
    mean_lat = (init_coord["lat"] + latest_coord["lat"])/2

    r = 6371.009

    distance = r * np.sqrt((delta_lat**2) + (np.cos(mean_lat) * delta_lon)**2)
    return distance




########################################
#       Simulation
########################################

def simulation():
    sim_data = sub_data.Get_data()
    datapoints = 3000
    noise = np.array([[0],[0],[0]])
    control_inputs = np.array([[0], [0]])

    init_state = np.array([[0], [0], [0]])
    init_input = np.array([[0], [0]])
    init_noise = np.array ([[0], [0], [0]])
    init_pos_reading = np.array([[0], [0], [0]])
    sim = True

    state_estimation = kalman.EKF(init_state, init_input, init_noise, init_pos_reading, sim)
    states = []

    for i in range(datapoints):
        states.append(state_estimation.update())
        distance = distance_calc(states)
        print(distance)
        if distance > 10:
            break

    return states


########################################
#   Plot result
########################################


if __name__ == "__main__":
    states = simulation()
    #print(states)
    

