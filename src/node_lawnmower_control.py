import rclpy
import threading    
import tty
import termios
import signal
import numpy as np

from rclpy.node import Node
from hqv_public_interface.msg import MowerGnssRtkRelativePositionENU
from hqv_public_interface.msg import RemoteDriverDriveCommand
from hqv_public_interface.msg import MowerImu
from hqv_public_interface.msg import MowerGnssPosition
from hqv_public_interface.msg import MowerWheelSpeed
from hqv_public_interface.msg import MowerWheelCounter
from hqv_public_interface.msg import MowerGnssPosAcc
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool
from std_msgs.msg import Float64

rclpy.init(args=None)


class Lawnmower_Control(Node):
    def __init__(self):
        super().__init__('lawnmower_control')

        # Publishers
        self.drive_publisher = self.create_publisher(RemoteDriverDriveCommand, '/hqv_mower/remote_driver/drive', 100)
        
        # Subscribers
        self.rtk_subscriber = self.create_subscription(MowerGnssRtkRelativePositionENU, '/hqv_mower/gnss_rtk/rel_enu', self.rtk_callback, 10)
        
        self.gnss_subscriber = self.create_subscription(MowerGnssPosition, '/hqv_mower/gnss/position', self.gnss_callback, 10)

        self.gnss_accuracy_subscriber = self.create_subscription(MowerGnssPosAcc, '/hqv_mower/gnss/position/acc', self.gnss_accuracy_callback, 10)
                
        self.IMU_subscriber = self.create_subscription(MowerImu, '/hqv_mower/imu0/orientation', self.IMU_callback, 10)
        
        self.wheelspeed_0_subscriber = self.create_subscription(MowerWheelSpeed, '/hqv_mower/wheel0/speed', self.wheelspeed_0_callback, 10)
        
        self.wheelspeed_1_subscriber = self.create_subscription(MowerWheelSpeed, '/hqv_mower/wheel1/speed', self.wheelspeed_1_callback, 10)
        
        self.wheelcounter_0_subscriber = self.create_subscription(MowerWheelCounter, '/hqv_mower/wheel0/counter', self.wheelcounter_0_callback, 10)

        self.wheelcounter_1_subscriber = self.create_subscription(MowerWheelCounter, '/hqv_mower/wheel1/counter', self.wheelcounter_1_callback, 10)

        self.coord_init_ongoing_subscriber = self.create_subscription(Bool, '/pos_init/ongoing', self.coord_init_ongoing_callback, 10)

        self.coord_init_done_subscriber = self.create_subscription(Bool, '/pos_init/done', self.coord_init_done_callback, 10)

        self.coord_init_pos1_subscriber = self.create_subscription(Float64MultiArray, '/pos_init/position1', self.coord_init_pos1_callback, 10)

        self.coord_init_pos2_subscriber = self.create_subscription(Float64MultiArray, '/pos_init/position2', self.coord_init_pos2_callback, 10)

        self.rtk_angle_offset_subscriber = self.create_subscription(Float64, '/pos_init/angle_offset', self.rtk_angle_offset_callback, 10)

        # drive message
        self._msg_drive = RemoteDriverDriveCommand()
        self._msg_angle_offset = Float64()

        # other
        self._update_rate = 40.0
        self._rtk_x = None
        self._rtk_y = None
        self._rtk_x_init = None
        self._rtk_y_init = None
        self._gnss_x = None
        self._gnss_y = None
        self._gnss_x_init = None
        self._gnss_y_init = None
        self._gnss_accuracy_horizontal = None
        self._gnss_accuracy_vertical = None
        self._yaw_init = 0
        
        self._wheelspeed0 = 0
        self._wheelspeed1 = 0
        
        self._wheelcounter0 = 0
        self._wheelcounter1 = 0
        self._wheelcounter0_init = 0
        self._wheelcounter1_init = 0

        # init flags
        self._wheel0_init = 0
        self._wheel1_init = 0
        self._yaw_init_flag = 0

        # Coordinate system init
        self._coord_init_ongoing = True
        self._coord_init_done = False
        self._coord_init_pos1 = [0,0]
        self._coord_init_pos2 = [0,0]

        self._rtk_angle_offset = 0

        """
            ADD INIT FLAGS FOR ALL INIT CALLBACKS
        """

        self._time = 0
        self._time_prev = 0



    ######   ######   #        #        #####    ######   ######   #   ##   ######
    #        #    #   #        #        #    #   #    #   #        # ##     #
    #        ######   #        #        #####    ######   #        ##       ######
    #        #    #   #        #        #    #   #    #   #        # ##          #
    ######   #    #   ######   ######   #####    #    #   ######   #   ##   ######
    
    # RTK
    def rtk_callback(self, rtk):
        if (self._rtk_x_init and self._rtk_y_init) is None:
            self._rtk_x_init = rtk.east
            self._rtk_y_init = rtk.north

        self._rtk_x = rtk.east
        self._rtk_y = rtk.north


    # GNSS
    def gnss_callback(self, gnss):
        if (self._gnss_x_init and self._gnss_y_init) is None:
            self._gnss_x_init = gnss.latitude
            self._gnss_y_init = gnss.longitude

        self._gnss_x = gnss.latitude
        self._gnss_y = gnss.longitude


    def gnss_accuracy_callback(self, accuracy):
        self._gnss_accuracy_horizontal = accuracy.horizontal
        self._gnss_accuracy_vertical = accuracy.vertical


    # Drive
    def drive(self, speed, steering):
        self._msg_drive.header.stamp = self.get_clock().now().to_msg()

        self._time_prev = self._time
        self._time = self.get_clock().now().nanoseconds

        self._msg_drive.speed = speed
        self._msg_drive.steering = steering

        self.drive_publisher.publish(self._msg_drive)

    def stop_drive(self):
        self._msg_drive.speed = 0.0
        self._msg_drive.steering = 0.0
        self.drive_publisher.publish(self._msg_drive)
        print("STOP")

    # IMU
    def IMU_callback(self, imu):
        if self._yaw_init_flag == 0:
            self._yaw_init = imu.yaw
            self._yaw_init_flag = 1
        self._yaw = imu.yaw


    # Wheelspeeds
    def wheelspeed_0_callback(self, wheelspeed_0):
        self._wheelspeed0 = wheelspeed_0.speed
        
    def wheelspeed_1_callback(self, wheelspeed_1):
        self._wheelspeed1 = wheelspeed_1.speed

    
    # Wheelcounters
    def wheelcounter_0_callback(self, counter):
        if self._wheel0_init == 0:
            self._wheelcounter0_init = counter.counter
            self._wheel0_init = 1
            
        self._wheelcounter0 = counter.counter
        
    def wheelcounter_1_callback(self, counter):
        if self._wheel1_init == 0:
            self._wheelcounter1_init = counter.counter
            self._wheel1_init = 1
        self._wheelcounter1 = counter.counter

    # Coordinate system init
    def coord_init_ongoing_callback(self, msg):
        self._coord_init_ongoing = msg.data

    def coord_init_done_callback(self, msg):
        self._coord_init_done = msg.data

    def coord_init_pos1_callback(self, msg):
        self._coord_init_pos1 = msg.data

    def coord_init_pos2_callback(self, msg):
        self._coord_init_pos2 = msg.data

    def rtk_angle_offset_callback(self, msg):
        self._rtk_angle_offset = msg.data


    ######   ######   #######   #######    ######    #######    ######
    #        #           #         #       #         #     #    #
    #  ###   ###         #         #       ###       #######    ######
    #    #   #           #         #       #         # ##            #
    ######   ######      #         #       ######    #   ##     ######

    # Getters are used to get the latest data from ROS topics
    # ex: yaw = drive_node.get_yaw()

    def get_rate(self):
        return self.create_rate(self._update_rate)
    
    def get_yaw(self):
        return self._yaw
    
    def get_rtk_init(self):
        return self._rtk_x_init, self._rtk_y_init
    
    def get_rtk(self):
        return self._rtk_x, self._rtk_y
    
    def get_gnss_pos(self):
        return self._gnss_x, self._gnss_y
    
    def get_gnss_pos_init(self):
        return self._gnss_x_init, self._gnss_y_init
    
    def get_gnss_accuracy(self):
        return self._gnss_accuracy_horizontal, self._gnss_accuracy_vertical
    
    def get_wheelspeeds(self):
        return self._wheelspeed0, self._wheelspeed1
    
    def get_wheelcounters(self):
        return self._wheelcounter0, self._wheelcounter1
    
    def get_wheelcounters_init(self):
        return self._wheelcounter0_init, self._wheelcounter1_init
    
    def get_time(self):
        return self._time_prev, self._time
    
    def get_updaterate(self):
        return self._update_rate

    def get_coord_init_ongoing(self):
        return self._coord_init_ongoing

    def get_coord_init_done(self):
        return self._coord_init_done

    def get_coord_init_pos1(self):
        return self._coord_init_pos1

    def get_coord_init_pos2(self):
        return self._coord_init_pos2

    def get_rtk_angle_offset(self):
        return self._rtk_angle_offset
    
    
