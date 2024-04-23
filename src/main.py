#!/usr/bin/env python

import node_lawnmower_control
import regulation
import regulation_circle
import signal
import rclpy
import threading
import numpy as np
import json
#read keys
import sys
import tty
import termios

drive_node = node_lawnmower_control.Lawnmower_Control()
regulator = regulation.Regulation(drive_node)
regulator_circle = regulation_circle.Regulation(drive_node)

#import differential_drive
#diff_drive = differential_drive.Differential_Drive(drive_node)

import path_planner
path = path_planner.Path()


def ctrlc_shutdown(sig, frame):
    drive_node.stop_drive()
    rclpy.shutdown()


def constant_speed():
    rate = drive_node.get_rate()
    drive_node.drive(0.5, 0.0)
    rate.sleep()


def goal(x_error, y_error, x_error_old, y_error_old, dir, reset_integral, theta_ref,index_end_point):
    total_error = np.sqrt(x_error**2 +  y_error**2)

    point = path.get_point()
    end_points = path.get_endpoints()
    rotate = False

    if point in end_points and index_end_point == end_points.index(point):
        rotate = True
        print("INDEX END POINT:", index_end_point)
        while rotate == True:            
            x_error,y_error, x_error_old, y_error_old, x_kalman, y_kalman, theta, time, x_odometry, y_odometry, dir, x_rtk, y_rtk, reset_integral = regulator.update(point[0], point[1], point[2], rotate)

            if abs(theta_ref - theta) >= np.pi/2:
                rotate = False
                path.update_point()
        index_end_point +=1
        regulator.reset_error_sum_rotation()

    seperate = True
    
    if seperate == True:
        if dir =="x":
            if x_error<0.3:
                path.update_point()
                regulator.reset_error_sum_dir(dir)
        elif dir == "y":
            if y_error<0.3:
                path.update_point()
                regulator.reset_error_sum_dir(dir)
    else:

        if total_error < 0.3:
            path.update_point()
            regulator.reset_error_sum_dir(dir)
    
    if reset_integral == True:
        regulator.reset_error_sum_crossed_line(dir)
    
    point = path.get_point()
    
    if point[0] == None or point[1] == None:
        drive_node.stop_drive()
        rclpy.shutdown()
    return point, index_end_point


def write_json(kalman_pos, ref_pos, odometry_pos,rtk_pos):
    json_data = {
        "kalman":kalman_pos,
        "ref":ref_pos,
        "odometry":odometry_pos,
        "rtk":rtk_pos}

    json_object = json.dumps(json_data, indent=2, ensure_ascii=True)

    with open("assets/data/11-04-24/50m-straight-20-12-0_5.json", "w",) as outfile:
        outfile.write(json_object)
   


def main():
    signal.signal(signal.SIGINT, ctrlc_shutdown)

    thread = threading.Thread(target=rclpy.spin, args=(drive_node,))
    thread.start()

    rate = drive_node.get_rate()
    rate.sleep()
    
    # set ref path
    #När vi sätter path så behöver vi tänka på att vi stannar en bit innan samt att vi roterar baserat på avståndet D så vi behöver lägga till/ta bort 0,5 i x och 0,2 i y
    # path.set_path(0, 0, 50, 0, 25,"x")
    path.set_path(0, 0, 5.5, 0, 50,"x")
    path.set_path(5, 0.2, 5, 5, 50,"y")
    # path.set_path(10, -10, 0, -10,25,"x")
    # path.set_path(0, -10, 0, 0, 25,"y")


    # path.set_path(0,0,100,0, 20)

    # path.set_path(1,0,2,1,100)      # (x_0, y_0, x_n, y_n, ppm, dir)
    # path.set_path(2,0,2,2,100)

    # kom ihåg startvinkel
    radius = 3
    rotate = False
    index_end_point = 0
    #path.set_circle_path(radius, (9.15,0), 3000,dir = "None")

    
    next_point = path.get_point()
    print("PATH", path._path)

    kalman_pos = {}
    ref_pos = {}
    odometry_pos = {}
    rtk_pos = {}
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
            x_error,y_error, x_error_old, y_error_old, x_kalman, y_kalman, theta, time, x_odometry, y_odometry, dir, x_rtk, y_rtk, reset_integral = regulator.update(next_point[0], next_point[1], next_point[2], rotate)
            theta_ref = theta
            # New regulator
            #x_error, y_error, x, y, theta, time = diff_drive.update(next_point[0], next_point[1])
            
            print("TIME: ", time)

            # Data logging
            #kalman_pos[time] = [x_kalman, y_kalman, theta]
            #ref_pos[time] = [next_point[0], next_point[1]]
            #odometry_pos[time] = [x_odometry, y_odometry]
            #rtk_pos[time] = [x_rtk,y_rtk]

            # calc next ref point
            next_point,index_end_point = goal(x_error, y_error, x_error_old, y_error_old, dir,reset_integral, theta_ref,index_end_point)

    # write_json(kalman_pos, ref_pos, odometry_pos, rtk_pos)


    drive_node.destroy_node()
    print("End of main")


if __name__ == '__main__':
    main()