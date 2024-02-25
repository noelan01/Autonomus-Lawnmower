import json
import sys
import numpy as np

sys.path.append('./src/')

import kalman


########################################
#       Simulation
########################################

def simulation():
    datapoints = 30
    noise = 0
    control_inputs = np.array([0, 0, 0])
    time_init = "00:00:00.00"
    time_step = 0.200

    init_state = np.array([0, 0, 0])
    init_input = np.array([0, 0])
    init_noise = np.array ([0, 0])
    init_pos_reading = np.array([0, 0, 0])

    state_estimation = kalman.EKF(init_state, init_input, init_noise, init_pos_reading)
    states = []

    for i in range(datapoints):
        states.append(state_estimation.update())

    return states


########################################
#   Plot result
########################################


if __name__ == "__main__":
    states = simulation()
