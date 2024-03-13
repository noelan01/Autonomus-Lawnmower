import rclpy
import threading
import sys
import tty
import termios
import signal
from statistics import mean

import node_coordinate_init
coord_node = node_coordinate_init.Coordinate_Node()

keep_going = True


class Points():
    def __init__(self):
        self._east_1 = []
        self._north_1 = []
        self._east_2 = []
        self._north_2 = []

        self._point1 = 0
        self._point2 = 0


    def point_set(self, east, north):
        global keep_going
        key = GetchUnix()

        if key == 'd':
            print("done")
            self._point1, self._point2 = self.point_mean()
            
        elif key == 'a':
            print("east: ", east, "north: ", north)
            self.append_point(1, east, north)

        elif key == 'b':
            print("east: ", east, "north: ", north)
            self.append_point(2, east, north)

        elif key == 'e':
            print("erase points")
            self.clear_points()

        elif key == 'q':
            print("QUIT COORD INIT")
            keep_going = False


    def append_point(self, point, east, north):
        if point == 1:
            self._east_1.append(east)
            self._north_1.append(north)
        elif point == 2:
            self._east_2.append(east)
            self._north_2.append(north)


    def point_mean(self):
        point1 = [mean(self._east_1), mean(self._north_1)]
        point2 = [mean(self._east_2), mean(self._north_2)]

        return point1, point2
    

    def clear_points(self):
        self._east_1 = []
        self._north_1 = []
        self._east_2 = []
        self._north_2 = []


    def get_points(self):
        return self._point1, self._point2
    


def GetchUnix():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def ctrlc_shutdown(sig, frame):
    rclpy.shutdown()



def main():
    signal.signal(signal.SIGINT, ctrlc_shutdown)

    thread = threading.Thread(target=rclpy.spin, args=(coord_node,))
    thread.start()

    rate = coord_node.get_rate()
    rate.sleep()


    print("q - quit\na - point 1\nb - point 2\ne - erase points")
    while keep_going:

        east, north = coord_node.get_rtk()    
        Points.point_set(east, north)

        point1, point2 = Points.get_points()

        coord_node.pub_point1(point1)
        coord_node.pub_point2(point2)

        rate.sleep

    coord_node.destroy_node()
    print("End of coordinate node")


if __name__ == '__main__':
    main()