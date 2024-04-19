import rclpy
import threading
import sys
import tty
import termios
import signal
from statistics import mean
import numpy as np
import json

import node_coordinate_init
coord_node = node_coordinate_init.Coordinate_Node()

keep_going = True

JSON_PATH = "coordinate_init/config/init_positions.json"



#------------------------READ POINTS CLASS-------------------------------------
class Points_Init():
    def __init__(self):
        self._east_1_list = []
        self._north_1_list = []
        self._east_2_list = []
        self._north_2_list = []

        self._east1 = 0
        self._north1 = 0 
        self._east2 = 0
        self._north2 = 0

        self._yaw_offset = 0
        self._yaw_offset_list = []

        self._done = False
        self._read_json = False


    def point_set(self, east, north, yaw_angle):
        global keep_going
        key = GetchUnix()

        if key == 'd':
            print("done")
            self._east1, self._north1, self._east2, self._north2, self._yaw_offset = self.point_mean()
            self.write()
            self._done = True
            
        elif key == 'a':
            print("Point 1:     east: ", east, "north: ", north)
            self.append_point(1, east, north, yaw_angle)
            self._done = False

        elif key == 'b':
            print("Point 2:     east: ", east, "north: ", north)
            self.append_point(2, east, north, yaw_angle)
            self._done = False

        elif key == 'e':
            print("erase points")
            self.clear_points()
            self._done = False

        elif key == 'j':
            print("read from JSON file")
            try:
                self.read()
                self._read_json = True
                print("JSON file read succesfully")
            except:
                print("JSON file NOT read")

        elif key == 'q':
            print("QUIT COORD INIT")
            self._done = True
            keep_going = False

        else:
            keep_going = True


    def append_point(self, point, east, north, yaw_angle):
        if point == 1:
            self._east_1_list.append(east)
            self._north_1_list.append(north)
        elif point == 2:
            self._east_2_list.append(east)
            self._north_2_list.append(north)

        self._yaw_offset_list.append(yaw_angle)


    def point_mean(self):
        if (len(self._east_1_list) and len(self._north_1_list) and len(self._yaw_offset_list)) >= 1:
            east1 = mean(self._east_1_list)
            north1 = mean(self._north_1_list)
            yaw_offset = mean(self._yaw_offset_list)
        else:
            east1 = 0
            north1 = 0
            yaw_offset = 0

        if (len(self._east_2_list) and len(self._north_2_list)) >= 1:
            east2 = mean(self._east_2_list)
            north2 = mean(self._north_2_list)
        else:
            east2 = 0
            north2 = 0

        return east1, north1, east2, north2, yaw_offset
    
    
    def read(self):
        with open(JSON_PATH, 'r') as f1:
            data = json.load(f1)

        self._east1 =   data["point 1"]["east"]
        self._north1 =  data["point 1"]["north"]
        self._east2 =   data["point 2"]["east"]
        self._north2 =  data["point 2"]["north"]
        self._yaw_offset = data["angle"]

        f1.close()


    def write(self):
        json_data = {"point 1": {"east": self._east1,
                                 "north": self._north1},
                    "point 2": {"east": self._east2,
                                "north": self._north2},
                    "angle":    self._yaw_offset}

        json_object = json.dumps(json_data, indent=2, ensure_ascii=True)
 
        with open(JSON_PATH, "w",) as outfile:
            outfile.write(json_object)

        print("JSON file written")

    

    def clear_points(self):
        self._east_1_list = []
        self._north_1_list = []
        self._east_2_list = []
        self._north_2_list = []
        self._yaw_offset_list = []


    def get_points(self):
        return self._east1, self._north1, self._east2, self._north2
    
    def get_yaw_offset(self):
        return self._yaw_offset

    def is_done(self):
        return self._done
    
    def read_json(self):
        return self._read_json
    

#------------------------CALCULATE COORDINATE SYSTEMS OFFSET ANGLE-------------------------------------

def get_offset(x_start_rtk,y_start_rtk,x_end_rtk,y_end_rtk): 
    #---------EXCEPTIONS FOR SINGULARITIES----------
    if x_end_rtk == x_start_rtk:
        if y_start_rtk > y_end_rtk:
            rtk_heading = np.pi
        else:
            rtk_heading = 0
    elif y_end_rtk == y_start_rtk:
        if x_start_rtk > x_end_rtk:
            rtk_heading = 3*np.pi/2
        else:
            rtk_heading = np.pi/2
    #-----------------------------
    else: 
        slope = (y_end_rtk-y_start_rtk) / (x_end_rtk-x_start_rtk)
        if x_end_rtk>x_start_rtk:
            rtk_heading = np.arctan2(1, slope) 
        else:
            rtk_heading = np.arctan2(1,slope) + np.pi 
        
    rtk_east = np.pi/2
    offset_angle = rtk_heading-rtk_east
    #print("RTK Heading: ", np.degrees(rtk_heading))
    return offset_angle    


def GetchUnix():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def ctrlc_shutdown(sig, frame):
    rclpy.shutdown()



def main():
    points = Points_Init()

    signal.signal(signal.SIGINT, ctrlc_shutdown)

    thread = threading.Thread(target=rclpy.spin, args=(coord_node,))
    thread.start()

    rate = coord_node.get_rate()
    rate.sleep()

    print("q - quit\ne - erase points\nd - done\nj - JSON file\na - point 1\nb - point 2\n")
    while keep_going:
        coord_node.pub_ongoing(True)

        east, north = coord_node.get_rtk()
        yaw_angle = coord_node.get_yaw()
        points.point_set(east, north, yaw_angle)

        if points.is_done() or points.read_json() == True:
            east1, north1, east2, north2 = points.get_points()
            yaw_offset = points.get_yaw_offset()

            coord_node.pub_point1(east1, north1)
            coord_node.pub_point2(east2, north2)
            coord_node.pub_yaw_offset(yaw_offset)

            angle_offset = get_offset(east1, north1, east2, north2)

            coord_node.pub_rtk_angle_offset(angle_offset)

        coord_node.pub_done(points.is_done())

        rate.sleep

    coord_node.pub_ongoing(False)

    coord_node.destroy_node()
    print("End of coordinate node")
    rclpy.shutdown()


if __name__ == '__main__':
    main()