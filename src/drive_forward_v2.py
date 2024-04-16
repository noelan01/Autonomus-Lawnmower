import node_lawnmower_control
import rclpy
import threading
import time
import signal

from rclpy.node import Node

drive_node = node_lawnmower_control.Lawnmower_Control()

def drive_sequence():
    speed = 1.0
    steering = 0.0
    rate = drive_node.get_rate()
    start_time = time.time()
    #drive forward for 3 seconds
    while rclpy.ok() and time.time() - start_time < 4:
        drive_node.drive(speed, steering) #linear speed of 1, straight line expected
        drive_node.get_logger().info(f'Publishing message \n time elapsed {time.time()-start_time} \n')
        rate.sleep()   #ensure 10hz publish rate

    # Stop the movement
    drive_node.drive(speed=0.0, steering=0.0)

def ctrcl_shutdown(sig, frame):
    drive_node.stop_drive()
    rclpy.shutdown()

def main():

    signal.signal(signal.SIGINT, ctrcl_shutdown)

    thread = threading.Thread(target=rclpy.spin, args=(drive_node,))
    thread.start()

    drive_sequence()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
