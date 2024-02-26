import json
import sys
import numpy as np

sys.path.append('./src/')

GPS_DATA1 = '/home/noelan/chalmers/kandidatarbete/Autonomus-Lawnmower/tests/gps_2018-11-29_new.json'
GPS_DATA2 = '/home/noelan/chalmers/kandidatarbete/Autonomus-Lawnmower/tests/gps_2018-11-29_new.json'

def read_data(jsonobject):
    with open(jsonobject, 'r',encoding='utf8') as infile:
        gps_data = json.load(infile)
    infile.close()

    return gps_data

gps_data = read_data(GPS_DATA1)



class Get_data():
    def __init__(self):
        self._step = 0

    def time_to_float(self, time_string):
        time_lst = time_string.split(':')       # [h, m, s]
        time = (float(time_lst[0])*60 + float(time_lst[1]))*60 + float(time_lst[2])      # time in seconds
        return time

    def time_to_string(self, time_float):
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
    
    def get_pos_reading(self):
        time = gps_data['time'][self._step]
        self._step += 1
        pos = np.array([[gps_data[time]['lat']], [gps_data[time]['lon']], [0]])
        #print("pos reading",pos)
        return self.time_to_float(time), pos
    
    def get_control_input(self):
        return np.array([[0],[0]])
    
    def get_sensor_error(self):
        return np.array([[0],[0],[0]])
    
    
