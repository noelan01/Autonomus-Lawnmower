import json
import sys
import numpy as np

sys.path.append('./src/')

import sub_data
import kalman


########################################
#       Simulation
########################################

def simulation():
    sim_data = sub_data.Get_data()
    datapoints = 5
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

    return states


########################################
#   Plot result
########################################


if __name__ == "__main__":
    states = simulation()
    print(states)

