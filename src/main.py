#!/usr/bin/env python

import node_lawnmower_control
from geopy import distance
import signal
import rclpy
import threading

drive_node = node_lawnmower_control.Lawnmower_Control()

def ctrlc_shutdown(sig, frame):
    drive_node.stop_drive()
    rclpy.shutdown()


def main():
    signal.signal(signal.SIGINT, ctrlc_shutdown())

    thread = threading.Thread(target=rclpy.spin, args=(drive_node,))
    thread.start()

    # TODO
    """
        Kalla på funktioner som sköter styrningen och låt sedan main kalla på
        drive_node.drive() som skickar in styrsignalerna till motorerna.
        Likt nedan
    """

    while rclpy.ok():    # send drive commands to Lawnmower_Control node
        rate = drive_node.get_rate()
        drive_node.drive(0.5, 0.0)
        rate.sleep()
    print("End of main")


if __name__ == '__main__':
    main()