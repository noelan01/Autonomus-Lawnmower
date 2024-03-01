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
from hqv_public_interface.msg import MowerWheelCounter

rclpy.init(args=None)


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
        
        self.wheelspeed_0_subscriber = self.create_subscription(MowerWheelSpeed, '/hqv_mower/wheel0/speed', self.wheelspeed_0_callback, 10)
        
        self.wheelspeed_1_subscriber = self.create_subscription(MowerWheelSpeed, '/hqv_mower/wheel1/speed', self.wheelspeed_1_callback, 10)
        
        self.wheelcounter_0_subscriber = self.create_subscription(MowerWheelCounter, '/hqv_mower/wheel0/counter', self.wheelcounter_0_callback, 10)

        # messages
        self._msg_drive = RemoteDriverDriveCommand()

        # other
        self._update_rate = 10.0
        self._x = None
        self._y = None
        self._x_init = None
        self._y_init = None
        self._gnss_x = None
        self._gnss_y = None
        self._gnss_x_init = None
        self._gnss_y_init = None
        self._yaw_init = 0
        
        self._wheelspeed0 = 0
        self._wheelspeed1 = 0
        
        self._wheelcounter0 = 0
        self._wheelcounter1 = 0



    ######   ######   #        #        #####    ######   ######   #   ##   ######
    #        #    #   #        #        #    #   #    #   #        # ##     #
    #        ######   #        #        #####    ######   #        ##       ######
    #        #    #   #        #        #    #   #    #   #        # ##          #
    ######   #    #   ######   ######   #####    #    #   ######   #   ##   ######
    
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
        self._msg_drive.header.stamp = self.get_clock().now().to_msg()
        self._msg_drive.speed = linear_vel
        self._msg_drive.steering = yaw_rate

        self.drive_publisher.publish(self._msg_drive)

    def stop_drive(self):
        self._msg_drive.speed = 0.0
        self._msg_drive.steering = 0.0
        self.drive_publisher.publish(self._msg_drive)
        print("STOP")

    def IMU_callback(self, imu):
        if self._yaw_init == 0:
            self._yaw_init = imu.yaw
        self._yaw = imu.yaw

    def wheelspeed_0_callback(self, wheelspeed_0):
        self._wheelspeed0 = wheelspeed_0.speed
        
    def wheelspeed_1_callback(self, wheelspeed_1):
        self._wheelspeed1 = wheelspeed_1.speed

    def wheelspeed_1_callback(self):
        pass
    
    def wheelcounter_0_callback(self, counter):
        self._wheelcounter0 = counter.counter
        
    def wheelcounter_1_callback(self, counter):
        self._wheelcounter1 = counter.counter

    ######   ######   #######   #######    ######    #######    ######
    #        #           #         #       #         #     #    #
    #  ###   ###         #         #       ###       #######    ######
    #    #   #           #         #       #         # ##            #
    ######   ######      #         #       ######    #   ##     ######

    # Kalla på dessa för att komma åt ros data från andra filer
    # ex yaw = drive_node.get_yaw()

    def get_rate(self):
        return self.create_rate(self._update_rate)
    
    def get_yaw(self):
        return self._yaw
    
    def get_rtk(self):
        return self._