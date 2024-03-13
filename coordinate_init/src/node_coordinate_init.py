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

rclpy.init(args=None)


class Coordinate_Node(Node):
    def __init__(self):
        super().__init__('Coordinate_node')

        # Publishers
        self.point1_publisher = self.create_publisher(Float64MultiArray, '/pos/position1', 100)
        self.point2_publisher = self.create_publisher(Float64MultiArray, '/pos/position2', 100)
        
        # Subscribers
        self.rtk_subscriber = self.create_subscription(MowerGnssRtkRelativePositionENU, '/hqv_mower/gnss_rtk/rel_enu', self.rtk_callback, 10)
        
        self.gnss_subscriber = self.create_subscription(MowerGnssPosition, '/hqv_mower/gnss/position', self.gnss_callback, 10)

        self.gnss_accuracy_subscriber = self.create_subscription(MowerGnssPosAcc, '/hqv_mower/gnss/position/acc', self.gnss_accuracy_callback, 10)
                
        self.IMU_subscriber = self.create_subscription(MowerImu, '/hqv_mower/imu0/orientation', self.IMU_callback, 10)

        #self._msg_drive = RemoteDriverDriveCommand()
        self._msg_point1 = Float64MultiArray()
        self._msg_point2 = Float64MultiArray()

        self._update_rate = 2

        # RTK
        self._rtk_east = None
        self._rtk_north = None

        # GNSS
        self._gnss_long = None
        self._gnss_lat = None
        self._gnss_accuracy_horizontal = None
        self._gnss_accuracy_vertical = None

        # IMU
        self._yaw = 0


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


    """
        Publishers
    """

    def pub_point1(self, point):
        self._msg_point1.header.stamp = self.get_clock().now().to_msg()

        self._msg_point1.msg = point

        self.point1_publisher.publish(self._msg_point1)


    def pub_point2(self, point):
        self._msg_point2.header.stamp = self.get_clock().now().to_msg()

        self._msg_point2.msg = point

        self.point2_publisher.publish(self._msg_point2)

    

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