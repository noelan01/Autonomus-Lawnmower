import rclpy
import threading
import time

from rclpy.node import Node
from hqv_public_interface.msg import RemoteDriverDriveCommand #kanske måste lägga annan filepath
from hqv_public_interface.msg import MowerImu

class HQVMowerController(Node):
    """ 
    To test publishing on drive topic and subscription on imu yaw topic
    
    """
    
    def __init__(self):
        super().__init__('HQWMowerController')

        self.drive_publisher = self.create_publisher(RemoteDriverDriveCommand, '/hqv_mower/remote_driver/drive', 100)
        self.imu_subscription = self.create_subscription(MowerImu, '/hqv_mower/imu0/orientation', self.imu_callback, 10)

        self.yaw = None

        self.msg_drive = RemoteDriverDriveCommand()

    def imu_callback(self, msg):
        self.yaw = msg
        self.get_logger().info(f'Received position: {msg}')

    def move(self, speed, steering):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.speed = speed
        self.msg.steering = steering
        self.drive_publisher.publish(self.msg)

    def drive_sequence(self):
        # Command to drive forward
        self.move(speed=1.0, steering=0.0)

        start_time = time.time()

        #drive forward for 3 seconds
        while time.time() - start_time < 3.0:
            self.move(speed=1.0, steering=0.0) #linear speed of 1, straight line expected
            rclpy.spin_once(self, timeout_sec=0.1)  # Process any incoming messages/events
            self.get_logger().info('Publishing message')
            self.rate.sleep()   #ensure 10hz publish rate

        # Stop the movement
        self.move(speed=0.0, steering=0.0)

def main():
    rclpy.init()
    hqv_mower_controller = HQVMowerController()

    thread = threading.Thread(target=rclpy.spin, args=(hqv_mower_controller.drive_sequence,))
    thread.start()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
