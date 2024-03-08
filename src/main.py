#!/usr/bin/env python

import node_lawnmower_control
import signal
import rclpy
import threading
import numpy as np

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

    if total_error < 0.0125:
        path.update_point()
    
    point = path.get_point()
    
    if point[0] == None or point[1] == None:
        drive_node.stop_drive()
        rclpy.shutdown()
    return point


def main():
    signal.signal(signal.SIGINT, ctrlc_shutdown)

    thread = threading.Thread(target=rclpy.spin, args=(drive_node,))
    thread.start()

    #regulator.__init__(drive_node)

    rate = drive_node.get_rate()
    rate.sleep()

    path.set_path(0.4, 0, 3.4, 0, 8)      # (x_0, y_0, x_n, y_n, ppm)
    #path.set_path(2, 0, 3, 1, 8)
    next_point = path.get_point()

    print("PATH", path._path)

    while rclpy.ok():    # send drive commands to Lawnmower_Control node
        #constant_speed()
        x_error, y_error = regulator.update(next_point[0], next_point[1])
        next_point = goal(x_error, y_error)

    drive_node.destroy_node()
    print("End of main")


if __name__ == '__main__':
    main()