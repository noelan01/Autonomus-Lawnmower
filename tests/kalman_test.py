import json
import sys

sys.path.append('../data/')

from kalman import EKF


def read_data(jsonobject):
    with open(jsonobject, 'r',encoding='utf8') as infile:
        gps_data = json.load(infile)
    infile.close()

    return gps_data



