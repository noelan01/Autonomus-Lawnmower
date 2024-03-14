import rclpy
import threading    
import tty
import termios
import signal
import numpy as np

from rclpy.node import Node
from hqv_public_interface.msg   import MowerGnssRtkRelativePositionENU
from hqv_public_interface.msg   import MowerImu
from hqv_public_interface.msg   import MowerGnssPosition
from hqv_public_interface.msg   import MowerGnssPosAcc
from std_msgs.msg               import Float64MultiArray
from std_msgs.msg               import Bool

rclpy.init(args=None)


class Coordinate_Node(Node):
    def __init__(self):
        super().__init__('Coordinate_node')

        # Publishers
        self.point1_publisher = self.create_publisher(Float64MultiArray, '/pos_init/position1', 100)
        self.point2_publisher = self.create_publisher(Float64MultiArray, '/pos_init/position2', 100)

        self.init_publisher = self.create_publisher(Bool, '/pos_init/ongoing', 100)

        
        # Subscribers
        self.rtk_subscriber = self.create_subscription(MowerGnssRtkRelativePositionENU, '/hqv_mower/gnss_rtk/rel_enu', self.rtk_callback, 10)
        
        self.gnss_subscriber = self.create_subscription(MowerGnssPosition, '/hqv_mower/gnss/position', self.gnss_callback, 10)

        self.gnss_accuracy_subscriber = self.create_subscription(MowerGnssPosAcc, '/hqv_mower/gnss/position/acc', self.gnss_accuracy_callback, 10)
                
        self.IMU_subscriber = self.create_subscription(MowerImu, '/hqv_mower/imu0/orientation', self.IMU_callback, 10)

        # msg
        self._msg_point1 = Float64MultiArray()
        self._msg_point2 = Float64MultiArray()
        self._msg_ongoing = Bool()

        self._update_rate = 2

        # RTK
        self._rtk_east = 0
        self._rtk_north = 0

        # GNSS
        self._gnss_long = 0
        self._gnss_lat = 0
        self._gnss_accuracy_horizontal = 0
        self._gnss_accuracy_vertical = 0

        # IMU
        self._yaw = 0
        self._yaw_init = 0

        # Flags
        self._yaw_init_flag = 0


    ######   ######   #        #        #####    ######   ######   #   ##   ######
    #        #    #   #        #        #    #   #    #   #        # ##     #
    #        ######   #        #        #####    ######   #        ##       ######
    #        #    #   #        #        #    #   #    #   #        # ##          #
    ######   #    #   ######   ######   #####    #    #   ######   #   ##   ######
        
    # RTK
    def rtk_callback(self, rtk):
        self._rtk_east = rtk.east
        self._rtk_north = rtk.north

        
    # GNSS
    def gnss_callback(self, gnss):
        self._gnss_lat = gnss.latitude
        self._gnss_long = gnss.longitude


    def gnss_accuracy_callback(self, accuracy):
        self._gnss_accuracy_horizontal = accuracy.horizontal
        self._gnss_accuracy_vertical = accuracy.vertical


    # IMU
    def IMU_callback(self, imu):
        if self._yaw_init_flag == 0:
            self._yaw_init = imu.yaw
            self._yaw_init_flag = 1
        self._yaw = imu.yaw


    """
        Publishers
    """

    def pub_point1(self, east, north):
        self._msg_point1.data = [float(east), float(north)]
        self.point1_publisher.publish(self._msg_point1)


    def pub_point2(self, east, north):
        self._msg_point2.data = [float(east), float(north)]
        self.point2_publisher.publish(self._msg_point2)


    def pub_ongoing(self, ongoing):
        self._msg_ongoing.data = ongoing
        self.init_publisher.publish(self._msg_ongoing)

    

    ######   ######   #######   #######    ######    #######    ######
    #        #           #         #       #         #     #    #
    #  ###   ###         #         #       ###       #######    ######
    #    #   #           #         #       #         # ##            #
    ######   ######      #         #       ######    #   ##     ######


    def get_rate(self):
        return self.create_rate(self._update_rate)
    
    def get_yaw(self):
        return self._yaw
    
    def get_rtk(self):
        return self._rtk_east, self._rtk_north
    
    def get_gnss_pos(self):
        return self._gnss_lat, self._gnss_long
    
    def get_gnss_accuracy(self):
        return self._gnss_accuracy_horizontal, self._gnss_accuracy_vertical