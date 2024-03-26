import numpy as np
import math

class Path():
    def __init__(self):
        self._path = []
        self._current_point = 0
        self._num_points = 0


    def set_path(self, x_0, y_0, x_n, y_n , ppm):

        if y_0 - y_n == 0:
            if x_n > x_0:
                angle = 0
            else:
                angle = np.pi
        else:
            angle = np.arctan2((y_n - y_0),(x_n - x_0))

        distance = np.sqrt((x_0 - x_n)**2 + (y_0 - y_n)**2)
        num_points = int(distance * ppm)
        print("NUM POINTS: ", num_points)
        if distance * ppm > num_points:
            num_points -= 1

        self._num_points += num_points

        new_path = []
        
        for i in range(num_points):
            x = (distance / num_points) * i * np.cos(angle)
            y = (distance / num_points) * i * np.sin(angle)
            next_point = (x_0 + x, y_0 + y)
            new_path.append(next_point)

        new_path.append((x_n, y_n))

        self._path += new_path

    def set_circle_path(self, radius, center, num_points):
        circular_planner = CircularPathPlanner(radius, center, num_points)
        path = circular_planner.plan_path()
        self._path += path
        self._num_points += len(path)

    def set_lower_arc_path(self, radius, center, num_points):
        lower_arc_planner = ArcPathPlanner(radius, center, num_points)
        path = lower_arc_planner.plan_path_lower()
        self._path += path
        self._num_points += len(path)
    
    def set_upper_arc_path(self, radius, center, num_points):
        upper_arc_planner = ArcPathPlanner(radius, center, num_points)
        path = upper_arc_planner.plan_path_upper()
        self._path += path
        self._num_points += len(path)

    def update_point(self):
        self._current_point += 1

    def get_point(self):
        if self._current_point >= self._num_points:
            print("REACHED GOAL")
            print("")
            return (None, None)
        else:
            # print("DESIRED POINT: ", self._path[self._current_point])
            # print("")
            return self._path[self._current_point]

    def set_bottom_right_corner(self,radius,center,num_points):
        bottom_right_corner_planner = cornerPath(radius, center, num_points)
        bottom_right_corner_path = bottom_right_corner_planner.plan_path_bottom_right()
        self._path += bottom_right_corner_path
        self._num_points += len(bottom_right_corner_path)

    def set_bottom_left_corner(self,radius,center,num_points):
        bottom_left_corner_planner = cornerPath(radius, center, num_points)
        bottom_left_corner_path = bottom_left_corner_planner.plan_path_bottom_left()
        self._path += bottom_left_corner_path
        self._num_points += len(bottom_left_corner_path)

    def set_upper_right_corner(self,radius,center,num_points):
        upper_right_corner_planner = cornerPath(radius, center, num_points)
        upper_right_corner_path = upper_right_corner_planner.plan_path_upper_right()
        self._path += upper_right_corner_path
        self._num_points += len(upper_right_corner_path)

    def set_upper_left_corner(self,radius,center,num_points):
        upper_left_corner_planner = cornerPath(radius, center, num_points)
        upper_left_corner_path = upper_left_corner_planner.plan_path_upper_left()
        self._path += upper_left_corner_path
        self._num_points += len(upper_left_corner_path)
        

class CircularPathPlanner:
    def __init__(self, radius, center=(0, 0), num_points=100):
        self.radius = radius
        self.center = center
        self.num_points = num_points

    def plan_path(self):
        path = []
        for i in range(self.num_points):
            theta = 2 * math.pi * i / self.num_points
            x = self.center[0] - self.radius * math.cos(theta)
            y = self.center[1] + self.radius * math.sin(theta)
            path.append((x, y))
        return path


class ArcPathPlanner:
    def __init__(self, radius, center=(0, 0), num_points=2000):
        self.radius = radius
        self.center = center
        self.num_points = num_points
        self._ArcAngle = math.acos(5.5/9.15)

    def plan_path_lower(self):
        path = []
        for i in range(self.num_points):
            theta = 2*math.pi*i/self.num_points
            if theta > math.pi/2 - self._ArcAngle and theta < math.pi/2 + self._ArcAngle:
                x = self.center[0] - self.radius * math.cos(theta)
                y = self.center[1] + self.radius * math.sin(theta)
                path.append((x, y))
        return path

    def plan_path_upper(self):
        path = []
        for i in range(self.num_points):
            theta = 2*math.pi*i/self.num_points
            if theta > math.pi/2 - self._ArcAngle and theta < math.pi/2 + self._ArcAngle:
                x = self.center[0] - self.radius * math.cos(theta)
                y = self.center[1] - self.radius * math.sin(theta)
                path.append((x, y))
        return path    

class cornerPath:
    def __init__(self, radius, center=(0, 0), num_points=100):
        self.radius = radius
        self.center = center
        self.num_points = num_points
    
    def plan_path_bottom_right(self):
        path = []
        for i in range(self.num_points):
            theta = 2 * math.pi * i / self.num_points
            if theta<=math.pi/2:
                x = self.center[0] - self.radius * math.cos(theta)
                y = self.center[1] + self.radius * math.sin(theta)
            path.append((x, y))
        return path
    
    def plan_path_bottom_left(self):
        path = []
        for i in range(self.num_points):
            theta = 2 * math.pi * i / self.num_points
            if theta<=math.pi/2:
                x = self.center[0] + self.radius * math.sin(theta)
                y = self.center[1] + self.radius * math.cos(theta)
            path.append((x, y))
        return path

    def plan_path_upper_right(self):
        path = []
        for i in range(self.num_points):
            theta = 2 * math.pi * i / self.num_points
            if theta<=math.pi/2:
                x = self.center[0] - self.radius * math.sin(theta)
                y = self.center[1] - self.radius * math.cos(theta)
            path.append((x, y))
        return path
    
    def plan_path_upper_left(self):
        path = []
        for i in range(self.num_points):
            theta = 2 * math.pi * i / self.num_points
            if theta<=math.pi/2:
                x = self.center[0] + self.radius * math.cos(theta)
                y = self.center[1] - self.radius * math.sin(theta)
            path.append((x, y))
        return path

