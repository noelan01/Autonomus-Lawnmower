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


class Points_Init():
    def __init__(self):
        self._east_1_list = []
        self._north_1_list = []
        self._east_2_list = []
        self._north_2_list = []

        self._east1 = 0
        self._north1 = 0 
        self._east2 = 0
        self._north2 = 0

        self._done = False


    def point_set(self, east, north):
        global keep_going
        key = GetchUnix()

        if key == 'd':
            print("done")
            self._east1, self._north1, self._east2, self._north2 = self.point_mean()
            self._done = True
            
        elif key == 'a':
            print("Point 1:     east: ", east, "north: ", north)
            self.append_point(1, east, north)

        elif key == 'b':
            print("Point 2:     east: ", east, "north: ", north)
            self.append_point(2, east, north)

        elif key == 'e':
            print("erase points")
            self.clear_points()
            self._done = False

        elif key == 'q':
            print("QUIT COORD INIT")
            keep_going = False


    def append_point(self, point, east, north):
        if point == 1:
            self._east_1_list.append(east)
            self._north_1_list.append(north)
        elif point == 2:
            self._east_2_list.append(east)
            self._north_2_list.append(north)


    def point_mean(self):
        east1 = mean(self._east_1_list)
        north1 = mean(self._north_1_list)
        east2 = mean(self._east_2_list)
        north2 = mean(self._north_2_list)

        return east1, north1, east2, north2
    

    def clear_points(self):
        self._east_1_list = []
        self._north_1_list = []
        self._east_2_list = []
        self._north_2_list = []


    def get_points(self):
        return self._east1, self._north1, self._east2, self._north2

    def is_done(self):
        return self._done
    


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
    points = Points_Init()

    signal.signal(signal.SIGINT, ctrlc_shutdown)

    thread = threading.Thread(target=rclpy.spin, args=(coord_node,))
    thread.start()

    rate = coord_node.get_rate()
    rate.sleep()

    print("q - quit\ne - erase points\nd - done\na - point 1\nb - point 2\n")
    while keep_going:
        coord_node.pub_ongoing(True)

        east, north = coord_node.get_rtk()
        points.point_set(east, north)

        if points.is_done() == True:
            east1, north1, east2, north2 = points.get_points()

            coord_node.pub_point1(east1, north1)
            coord_node.pub_point2(east2, north2)

        rate.sleep

    coord_node.pub_ongoing(False)

    coord_node.destroy_node()
    print("End of coordinate node")
    rclpy.shutdown()


if __name__ == '__main__':
    main()