import rclpy
import threading
import sys
import tty
import termios
import matplotlib.pyplot as plt
import signal
import json
import numpy as np

from rclpy.node import Node
from hqv_public_interface.msg import MowerGnssRtkRelativePositionENU
from hqv_public_interface.msg import MowerGnssUnixTime
from hqv_public_interface.msg import RemoteDriverDriveCommand
from hqv_public_interface.msg import MowerImu
from hqv_public_interface.msg import MowerGnssPosition
from hqv_public_interface.msg import MowerWheelSpeed


class Lawnmower_Control(Node):
    def __init__(self):
        super().__init__('lawnmower_control')

        # Publishers
        self.drive_publisher = self.create_publisher(RemoteDriverDriveCommand, '/hqv_mower/remote_driver/drive', 100)
        
        # Subscribers
        self.rtk_subscriber = self.create_subscription(MowerGnssRtkRelativePositionENU, '/hqv_mower/gnss_rtk/rel_enu', self.rtk_callback, 10)
        
        self.gnss_subscriber = self.create_subscription(MowerGnssPosition, '/hqv_mower/gnss/position', self.gnss_callback, 10)
        
        self.time_subscriber = self.create_subscription(MowerGnssUnixTime, '/hqv_mower/gnss/unixtime', self.time_callback, 10)
        
        self.IMU_subscriber = self.create_subscription(MowerImu, '/hqv_mower/imu0/orientation', self.IMU_callback, 10)
        
        self.wheelspeed_left_subscriber = self.create_subscription(MowerWheelSpeed, '/hqv_mower/wheel0/speed', self.wheelspeed_left_callback, 10)
        
        self.wheelspeed_right_subscriber = self.create_subscription(MowerWheelSpeed, '/hqv_mower/wheel1/speed', self.wheelspeed_right_callback, 10)

        # messages
        self._msg_drive = RemoteDriverDriveCommand()

        # other
        self._update_freq = 10.0
        self._x = None
        self._y = None
        self._x_init = None
        self._y_init = None
        self._gnss_x = None
        self._gnss_y = None
        self._gnss_x_init = None
        self._gnss_y_init = None
        self._yaw_init = 0



    """
    CALLBACKS
    """
    
    # TODO
    # Skriv callback functioner

    def rtk_callback(self, rtk):
        if (self._x_init and self._y_init) is None:
            self._x_init = rtk.east
            self._y_init = rtk.north

        self._x = rtk.east
        self._y = rtk.north
        

    def gnss_callback(self, gnss):
        if (self._gnss_x_init and self._gnss_y_init) is None:
            self._gnss_x_init = gnss.latitude
            self._gnss_y_init = gnss.longitude

        self._gnss_x = gnss.latitude
        self._gnss_y = gnss.longitude

    def time_callback(self):
        pass

    def drive(self, linear_vel, yaw_rate):
        rate = self.create_rate(self._update_freq)
        self._msg_drive.header.stamp = self.get_clock().now().to_msg()
        self._msg_drive.speed = linear_vel
        self._msg_drive.steering = yaw_rate
        print(self._msg_drive)


        self.drive_publisher.publish(self._msg_drive)

    def stop_drive(self):
        self.msg.speed = 0.0
        self. msg.steering = 0.0
        self.drive_publisher.publish(self.msg)
        print("STOP")

    def IMU_callback(self, imu):
        if self._yaw_init == 0:
            self._yaw_init = imu.yaw
        self._yaw = imu.yaw

    def wheelspeed_left_callback(self):
        pass

    def wheelspeed_right_callback(self):
        pass