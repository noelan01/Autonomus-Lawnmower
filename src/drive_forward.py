import rclpy
import threading
import time

from rclpy.node import Node

from hqv_public_interface.msg import RemoteDriverDriveCommand
from hqv_public_interface.msg import MowerImu

class HQVMowerController(Node):
    def __init__(self):
        super().__init__('hqv_mower_controller')
        self.drive_publisher = self.create_publisher(RemoteDriverDriveCommand, '/hqv_mower/remote_driver/drive', 100)
        self.imu_subscription = self.create_subscription(MowerImu, '/hqv_mower/imu0/orientation', self.imu_callback, 10)
        self.yaw = None

    def imu_callback(self, msg):
        self.yaw = msg
        self.get_logger().info(f'Received position: {msg}')

    def move(self, speed, steering):
        msg = RemoteDriverDriveCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.speed = speed
        msg.steering = steering
        self.drive_publisher.publish(msg)

    def drive_sequence(self):
        # Command to drive forward
        self.move(speed=1.0, steering=0.0)

        # Wait for 3 seconds
        start_time = time.time()
        while time.time() - start_time < 3.0:
            self.move(speed=1.0, steering=0.0)
            rclpy.spin_once(self, timeout_sec=0.1)  # Process any incoming messages/events
            self.get_logger().info('Publishing message')
            self.rate.sleep()

        # Stop the movement
        self.move(speed=0.0, steering=0.0)

def main():
    rclpy.init()

    hqv_mower_controller = HQVMowerController()

    hqv_mower_controller.thread = threading.Thread(target=rclpy.spin, args=(hqv_mower_controller,))
    hqv_mower_controller.thread.start()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
