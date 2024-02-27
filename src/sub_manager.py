"""
    Sub manager:
    Hämtar data från ROS2 nätverket.
    För att undvika redundans bör ROS data endast hämtas via denna fil
"""

import rclpy

from rclpy.node import Node
from hqv_public_interface.msg import MowerGnssRtkRelativePositionENU
from hqv_public_interface.msg import MowerGnssUnixTime
from hqv_public_interface.msg import RemoteDriverDriveCommand
from hqv_public_interface.msg import MowerImu


class Sub_manager():
    def __init__(self):

        # gps data
        self.rtk_subscriber = self.create_subscription(MowerGnssRtkRelativePositionENU, '/hqv_mower/gnss_rtk/rel_enu', self.rtk_callback, 10)
        # tiden
        self.time_subscriber = self.create_subscription(MowerGnssUnixTime, '/hqv_mower/gnss/unixtime', self.time_callback, 10)
        # styrsignaler
        self.control_input_subscriber = self.create_subscription(RemoteDriverDriveCommand, '/hqv_mower/traction/drive', self.control_input_callback, 10)
        # imu data
        self.IMU_subscriber = self.create_subscription(MowerImu, '/hqv_mower/imu0/orientation', self.IMU_callback, 10)


    """
    CALLBACKS
    """
    
    # TODO
    # Skriv callback functioner

    def rtk_callback(self):
        pass

    def gnss_callback(self):
        pass

    def time_callback(self):
        pass

    def control_input_callback(self):
        pass

    def IMU_callback(self):
        pass