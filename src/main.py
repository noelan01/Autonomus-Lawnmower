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

    if total_error < 1:
        path.update_point()
        regulator.reset_error_sum()
    
    point = path.get_point()
    
    if point[0] == None or point[1] == None:
        drive_node.stop_drive()
        rclpy.shutdown()
    return point


def write_json(measured_pos, ref_pos, odometry_pos):
    json_object = json.dumps(measured_pos, indent=2, ensure_ascii=True)
    json_object2 = json.dumps(ref_pos, indent=2, ensure_ascii=True)
    json_object3 = json.dumps(odometry_pos, indent=2, ensure_ascii=True)
 
    # with open("../assets/data/2024_03_28_ChangedWheelIndex/path.json", "w",) as outfile:
    #     outfile.write(json_object)
    
    # with open("../assets/data/2024_03_28_ChangedWheelIndex/ref_path.json", "w",) as outfile:
    #     outfile.write(json_object2)

    with open("assets/data/2024_03_28_Mossen_rtk/path_straight_line_30_rtk_2.json", "w",) as outfile:
        outfile.write(json_object)
    
    with open("assets/data/2024_03_28_Mossen_rtk/ref_path_straight_line_30_2.json", "w",) as outfile:
        outfile.write(json_object2)

    with open("assets/data/2024_03_28_Mossen_rtk/ref_path_straight_line_30_odometry_2.json", "w",) as outfile:
        outfile.write(json_object3)


def main():
    signal.signal(signal.SIGINT, ctrlc_shutdown)

    thread = threading.Thread(target=rclpy.spin, args=(drive_node,))
    thread.start()

    rate = drive_node.get_rate()
    rate.sleep()
    
    # set ref path
    path.set_path(0, 0, 10, 0, 10,"x")
    path.set_path(10, 0, 10, 10, 10,"y")
    path.set_path(10, 10, 0, 10,10,"x")
    path.set_path(0, 10, 0, 0, 10,"y")


    # path.set_path(0,0,100,0, 20)

    # path.set_path(1,0,2,1,100)      # (x_0, y_0, x_n, y_n, ppm, dir)
    # path.set_path(2,0,2,2,100)

    # kom ihåg startvinkel
    radius = 9.15
    #path.set_circle_path(radius, (-radius,0), 3000)

    
    next_point = path.get_point()
    print("PATH", path._path)

    measured_pos = {}
    ref_pos = {}
    odometry_pos = {}
    angle_offset = 0
    while rclpy.ok():
        if drive_node.get_coord_init_ongoing() == True:     # Initialization of local coordinate system
            if drive_node.get_coord_init_done() == True:    # Pos1 and pos2 has been set
                pos1 = drive_node.get_coord_init_pos1()
                pos2 = drive_node.get_coord_init_pos2()
                print("Positions set. Pos1: ", pos1, "Pos2: ", pos2)

                angle_offset = drive_node.get_rtk_angle_offset()
                print("Angle_offset: ", angle_offset)

            rate = drive_node.get_rate()
            drive_node.drive(0.0, 0.0)
            rate.sleep()
        else:
            print("-----------------------------------------------")
            # Original regulator
            x_error, y_error, x, y, theta, time, x_odometry, y_odometry = regulator.update(next_point[0], next_point[1])

            # New regulator
            #x_error, y_error, x, y, theta, time = diff_drive.update(next_point[0], next_point[1])
            
            print("TIME: ", time)

            # Data logging
            measured_pos[time] = [x, y, theta]
            ref_pos[time] = [next_point[0], next_point[1]]
            odometry_pos[time] = [x_odometry, y_odometry]

            # calc next ref point
            next_point = goal(x_error, y_error)

    # write_json(measured_pos, ref_pos, odometry_pos)

    drive_node.destroy_node()
    print("End of main")


if __name__ == '__main__':
    main()