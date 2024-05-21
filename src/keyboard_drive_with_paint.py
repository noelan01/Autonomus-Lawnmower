import rclpy
import threading
import sys
import tty
import termios
import RPi.GPIO as GPIO

from rclpy.node import Node
from hqv_public_interface.msg import RemoteDriverDriveCommand

RELAY_PIN = 7
GPIO.setmode(GPIO.BOARD)
GPIO.setup(RELAY_PIN, GPIO.OUT)
GPIO.output(RELAY_PIN, False)


keep_going = True
paint = GPIO.LOW


def GetchUnix():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def thread_function(keyboard_controller):
    global keep_going
    print("q - quit\nw - move forward\ns - move backward\na - turn left\nd - turn right\nr - increase speed\nf - decrease speed \n p - på/av målning \n" )
    s = 0.5
    while True:
        GPIO.output(RELAY_PIN, paint)
        key = GetchUnix()
        if key == 'q':
            keep_going = False
            keyboard_controller.executor.shutdown()
            return
        elif key == 'w':
            keyboard_controller.move(s, 0.0)
        elif key == 's':
            keyboard_controller.move(-s, 0.0)
        elif key == 'a':
            keyboard_controller.move(0.1, 2.0)
        elif key == 'd':
            keyboard_controller.move(0.1, -2.0)
        elif key == 'r':
            s += 0.1
            if s > 2:
                s = 2.0
            s = round(s, 1)
            print("Speed set to ", s)
        elif key == 'f':
            s -= 0.1
            if s < 0.0:
                s = 0.0
            s = round(s, 1)
            print("Speed set to ", s)
        
        elif key == 'p':
            if paint == GPIO.HIGH:
                paint = GPIO.LOW
            elif paint == GPIO.LOW:
                paint = GPIO.HIGH


class KeyboardRemoteDrive(Node):

    def __init__(self):
        super().__init__('keyboard_remote_drive')
        self.drive_publisher = self.create_publisher(RemoteDriverDriveCommand, '/hqv_mower/remote_driver/drive', 100)

    def move(self, speed, steering):
        msg = RemoteDriverDriveCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.speed = speed
        msg.steering = steering
        self.drive_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    keyboard_remote_drive = KeyboardRemoteDrive()

    thread = threading.Thread(target=thread_function, args=(keyboard_remote_drive,))
    thread.start()

    while keep_going:
        rclpy.spin_once(keyboard_remote_drive)

    GPIO.output(RELAY_PIN, GPIO.LOW)
    GPIO.cleanup()
    keyboard_remote_drive.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
