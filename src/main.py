#!/usr/bin/env python

import node_lawnmower_control
import regulation
import signal
import rclpy
import threading
import numpy as np
import json

drive_node = node_lawnmower_control.Lawnmower_Control()
regulator = regulation.Regulation(drive_node)

import differential_drive
diff_drive = differential_drive.Differential_Drive(drive_node)

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

    if total_error < 0.5:
        path.update_point()
        regulator.reset_error_sum()
    
    point = path.get_point()
    
    if point[0] == None or point[1] == None:
        drive_node.stop_drive()
        rclpy.shutdown()
    return point


def write_json(measured_pos, ref_pos):
    json_object = json.dumps(measured_pos, indent=2, ensure_ascii=True)
    json_object2 = json.dumps(ref_pos, indent=2, ensure_ascii=True)
 
    # with open("../assets/data/2024_03_28_ChangedWheelIndex/path.json", "w",) as outfile:
    #     outfile.write(json_object)
    
    # with open("../assets/data/2024_03_28_ChangedWheelIndex/ref_path.json", "w",) as outfile:
    #     outfile.write(json_object2)

    with open("assets/data/2024_03_28_ChangedWheelIndex/circle.json", "w",) as outfile:
        outfile.write(json_object)
    
    with open("assets/data/2024_03_28_ChangedWheelIndex/ref_circle.json", "w",) as outfile:
        outfile.write(json_object2)


def main():
    signal.signal(signal.SIGINT, ctrlc_shutdown)

    thread = threading.Thread(target=rclpy.spin, args=(drive_node,))
    thread.start()

    rate = drive_node.get_rate()
    rate.sleep()
    
    # set ref path
    #path.set_path(0, 0, 1, 0, 100)
    #path.set_path(1,0,2,1,100)      # (x_0, y_0, x_n, y_n, ppm)
    #path.set_path(2,0,2,2,100)
    radius = 0.5
    path.set_circle_path(radius, (-radius,0), 300)

    
    next_point = path.get_point()
    print("PATH", path._path)

    measured_pos = {}
    ref_pos = {}
    angle_offset = 0
    while rclpy.ok():
        # if drive_node.get_coord_init_ongoing() == True:     # Initialization of local coordinate system
        #     if drive_node.get_coord_init_done() == True:    # Pos1 and pos2 has been set
        #         pos1 = drive_node.get_coord_init_pos1()
        #         pos2 = drive_node.get_coord_init_pos2()
        #         print("Positions set. Pos1: ", pos1, "Pos2: ", pos2)

        #         angle_offset = drive_node.get_rtk_angle_offset()
        #         print("Angle_offset: ", angle_offset)

        #     rate = drive_node.get_rate()
        #     drive_node.drive(0.0, 0.0)
        #     rate.sleep()
        if(False):
            print(" ")
        else:
            print("-----------------------------------------------")
            # Original regulator
            x_error, y_error, x, y, theta, time = regulator.update(next_point[0], next_point[1])

            # New regulator
            #x_error, y_error, x, y, theta, time = diff_drive.update(next_point[0], next_point[1])
            
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