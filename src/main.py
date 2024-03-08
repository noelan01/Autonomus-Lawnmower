#!/usr/bin/env python

import node_lawnmower_control
import signal
import rclpy
import threading

drive_node = node_lawnmower_control.Lawnmower_Control()
import regulation
regulator = regulation.Regulation(drive_node)

def ctrlc_shutdown(sig, frame):
    drive_node.stop_drive()
    rclpy.shutdown()


def constant_speed():
    rate = drive_node.get_rate()
    drive_node.drive(0.5, 0.0)
    rate.sleep()


def main():
    signal.signal(signal.SIGINT, ctrlc_shutdown)

    thread = threading.Thread(target=rclpy.spin, args=(drive_node,))
    thread.start()

    #regulator.__init__(drive_node)

    rate = drive_node.get_rate()
    rate.sleep()

    while rclpy.ok():    # send drive commands to Lawnmower_Control node
        #constant_speed()
        regulator.update()

    drive_node.destroy_node()
    print("End of main")


if __name__ == '__main__':
    main()