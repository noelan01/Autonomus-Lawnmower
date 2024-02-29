import rclpy
import threading
import time
import signal

from rclpy.node import Node
from hqv_public_interface.msg import RemoteDriverDriveCommand #kanske måste lägga annan filepath

from hqv_public_interface.msg import RemoteDriverDriveCommand
from hqv_public_interface.msg import MowerImu

class HQVMowerController(Node):
    """ 
    To test publishing on drive topic and subscription on imu yaw topic
    
    """
    
    def __init__(self):
        super().__init__('HQWMowerController')

        self.drive_publisher = self.create_publisher(RemoteDriverDriveCommand, '/hqv_mower/remote_driver/drive', 100)
        self.drive_sub = self.create_subscription(RemoteDriverDriveCommand, '/hqv_mower/remote_driver/drive', self.drive_callback, 10)
        self.imu_subscription = self.create_subscription(MowerImu, '/hqv_mower/imu0/orientation', self.imu_callback, 10)

        self.yaw = None        
        self.drive = None
        
        self.speed = 0.0
        self.rate = 10
        self.msg_drive = RemoteDriverDriveCommand()

    def imu_callback(self, msg):
        self.yaw = msg
        self.get_logger().info(f'Received position: {msg}')
    
    def drive_callback(self, msg):
        self.drive = msg
        self.get_logger().info(f'Received drive: {msg} \n')

    def move(self, speed, steering):
        self.msg_drive.header.stamp = self.get_clock().now().to_msg()
        self.msg_drive.speed = speed
        self.msg_drive.steering = steering
        self.drive_publisher.publish(self.msg_drive)

    def get_rate(self):
        return self.create_rate(self.rate)



    def drive_sequence(self):

        rate = self.get_rate()
        start_time = time.time()
        #drive forward for 3 seconds
        while rclpy.ok() and time.time() - start_time < 15:
            self.speed += 1.0
            self.move(speed=self.speed, steering=0.0) #linear speed of 1, straight line expected
            self.get_logger().info(f'Publishing message')
            rclpy.spin_once(self, timeout_sec=0.1)  # Process any incoming messages/events
            rate.sleep()   #ensure 10hz publish rate

        # Stop the movement
        self.move(speed=0.0, steering=0.0)

def ctrcl_shutdown(sig, frame):
    rclpy.shutdown()

def main():
    rclpy.init()
    signal.signal(signal.SIGINT, ctrcl_shutdown)
    hqv_mower_controller = HQVMowerController()

    thread = threading.Thread(target=rclpy.spin, args=(hqv_mower_controller,))
    thread.start()

    hqv_mower_controller.drive_sequence()
    drive_thread.join()


    rclpy.shutdown()

if __name__ == '__main__':
    main()
