import json
import sys
import numpy as np

sys.path.append('./src/')

from kalman import EKF

gps_data1 = "gps_2018-11-29.json"
gps_data2 = "gps_2018-11-28.json"

def read_data(jsonobject):
    with open(jsonobject, 'r',encoding='utf8') as infile:
        gps_data = json.load(infile)
    infile.close()

    return gps_data

def time_to_float(time_string):
    time_lst = time_string.split(':')       # [h, m, s]
    time = (float(time_lst[0])*60 + float(time_lst[1]))*60 + float(time_lst[2])      # time in seconds
    return time

def time_to_string(time_float):
    hours = str(int(time_float // (60*60)))
    rest = time_float % (60*60)
    minutes = str(int(rest // 60))
    rest = rest % 60
    seconds = str(int(rest))
    milliseconds = str(int((rest % 1)*1000))
    
    if len(hours) == 1:
        hours = '0' + hours
    if len(minutes) == 1:
        minutes = '0' + minutes
    if len(seconds) == 1:
        seconds = '0' + seconds
    while len(milliseconds) < 3:
        milliseconds = milliseconds + '0'

    time = hours + ':' + minutes + ':' + seconds + '.' + milliseconds
    return time



########################################
#   Initieringar för simulation
########################################

datapoints = 30
noise = 0
control_inputs = np.array([0, 0, 0])
time_init = "00:00:00.00"
time_step = 0.200

for i in range(datapoints):
    # TODO
    # Simulering här
    pass

