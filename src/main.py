#!/usr/bin/env python

import node_lawnmower_control
import signal
import rclpy
import threading
import numpy as np
import json

drive_node = node_lawnmower_control.Lawnmower_Control()
import regulation
regulator = regulation.Regulation(drive_node)

import path_planner
path = path_planner.Path()

def ctrlc_shutdown(sig, frame):
    drive_node.stop_drive()
    rclpy.shutdown()


def constant_speed():
    rate = drive_node.get_rate()
    drive_node.drive(0.5, 0.0)
    rate.sleep()


def goal(x_error,y_error):
    total_error = np.sqrt(x_error**2 +  y_error**2)

    if total_error < 0.1:
        path.update_point()
    
    point = path.get_point()
    
    if point[0] == None or point[1] == None:
        drive_node.stop_drive()
        rclpy.shutdown()
    return point


def write_json(measured_pos, ref_pos):
    json_object = json.dumps(measured_pos, indent=2, ensure_ascii=True)
    json_object2 = json.dumps(ref_pos, indent=2, ensure_ascii=True)
 
    with open("../assets/data/following_path_08-03-24/circle.json", "w",) as outfile:
        outfile.write(json_object)
    
    with open("../assets/data/following_path_08-03-24/ref_circle.json", "w",) as outfile:
        outfile.write(json_object2)



def main():
    signal.signal(signal.SIGINT, ctrlc_shutdown)

    thread = threading.Thread(target=rclpy.spin, args=(drive_node,))
    thread.start()

    #regulator.__init__(drive_node)

    rate = drive_node.get_rate()
    rate.sleep()
    
    # set ref path
    #path.set_path(0.4, 0, 1.4, 0, 100)      # (x_0, y_0, x_n, y_n, ppm)
    path.set_circle_path(1, (-1,0.4), 500)

    
    next_point = path.get_point()
    print("PATH", path._path)

    measured_pos = {}
    ref_pos = {}

    while rclpy.ok():    # send drive commands to Lawnmower_Control node
        #constant_speed()
        x_error, y_error, x, y, theta, time = regulator.update(next_point[0], next_point[1])
        print("TIME: ", time)

        # Data logging
        measured_pos[time] = [x, y, theta]
        ref_pos[time] = [next_point[0], next_point[1]]

        # calc next ref point
        next_point = goal(x_error, y_error)
    
    write_json(measured_pos, ref_pos)

    drive_node.destroy_node()
    print("End of main")


if __name__ == '__main__':
    main()