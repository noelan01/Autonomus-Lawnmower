import numpy as np
import math

class Path():
    def __init__(self):
        self._path = []
        self._current_point = 0
        self._num_points = 0
        self._prev_point = 0
        self._next_point = 0
        self._end_points = []

        #La till att man kan bestämma vilken riktning den åker i för att ändra regleringen
        self._dir = None

    #Interpolera punkter vid roteringar så att inte avstånden mellan önskad och verklig punkt blir för stort
    def interpolate_points(self,coord_list, threshold):
        self.interpolated_coords = [coord_list[0]]  # Initialize with the first point

        
        self.start_x, self.start_y,self.dir = coord_list[self._current_point]
        self.end_x, self.end_y,self.new_dir = coord_list[self._next_point]
        self.distance = np.sqrt((self.end_x - self.start_x) ** 2 + (self.end_y - self.start_y) ** 2)

        if self.distance > threshold:
            num_intermediate_points = int(self.distance // threshold)
            self.dx = (self.end_x - self.start_x) / (num_intermediate_points + 1)
            self.dy = (self.end_y - self.start_y) / (num_intermediate_points + 1)
            new_index = 0

            for j in range(1, num_intermediate_points + 1):
                interpolated_x = self.start_x + j * self.dx
                interpolated_y = self.start_y + j * self.dy
                self.interpolated_coords.append((interpolated_x, interpolated_y))
                index_to_insert = self._path.index((self.start_x,self.start_y,self.dir))+new_index
                new_index+=new_index
                self._path.insert(index_to_insert,(interpolated_x,interpolated_y,self.new_dir))  # Add the next point
                self._num_points += 1


    def set_path(self, x_0, y_0, x_n, y_n , ppm, dir):

        self._end_points.append((x_n, y_n, dir))

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
            next_point = (x_0 + x, y_0 + y,dir)
            new_path.append(next_point)

        new_path.append((x_n, y_n, dir))

        self._path += new_path

    def set_circle_path(self, radius, center, num_points,dir):
        circular_planner = CircularPathPlanner(radius, center, num_points,dir)
        path = circular_planner.plan_path()
        self._path += path
        self._num_points += len(path)

    def set_lower_arc_path(self, radius, center, num_points,dir):
        lower_arc_planner = ArcPathPlanner(radius, center, num_points,dir)
        path = lower_arc_planner.plan_path_lower()
        self._path += path
        self._num_points += len(path)
    
    def set_upper_arc_path(self, radius, center, num_points,dir):
        upper_arc_planner = ArcPathPlanner(radius, center, num_points,dir)
        path = upper_arc_planner.plan_path_upper()
        self._path += path
        self._num_points += len(path)

    def update_point(self):
        self._current_point += 1
        self._prev_point = self._current_point - 1
        self._next_point = self._current_point + 1
        return self._current_point

    def get_point(self):
        if self._current_point >= self._num_points:
            print("REACHED GOAL")
            print("Total number of points: ",self._num_points)
            return (None, None,None)
        else:
            print("Desired point: ",self._path[self._current_point])
            return self._path[self._current_point]
        
    def get_prev_point(self):
        if self._prev_point < 0:
            return None
        else:
            return self._path[self._prev_point]
        
    def get_next_point(self):
        if self._next_point > len(self._path):
            return None
        else:
            return self._path[self._next_point]
        
    def get_endpoints(self):
        return self._end_points

    def set_bottom_right_corner(self,radius,center,num_points,dir):
        bottom_right_corner_planner = cornerPath(radius, center, num_points,dir)
        bottom_right_corner_path = bottom_right_corner_planner.plan_path_bottom_right()
        self._path += bottom_right_corner_path
        self._num_points += len(bottom_right_corner_path)

    def set_bottom_left_corner(self,radius,center,num_points,dir):
        bottom_left_corner_planner = cornerPath(radius, center, num_points,dir)
        bottom_left_corner_path = bottom_left_corner_planner.plan_path_bottom_left()
        self._path += bottom_left_corner_path
        self._num_points += len(bottom_left_corner_path)

    def set_upper_right_corner(self,radius,center,num_points,dir):
        upper_right_corner_planner = cornerPath(radius, center, num_points,dir)
        upper_right_corner_path = upper_right_corner_planner.plan_path_upper_right()
        self._path += upper_right_corner_path
        self._num_points += len(upper_right_corner_path)

    def set_upper_left_corner(self,radius,center,num_points,dir):
        upper_left_corner_planner = cornerPath(radius, center, num_points,dir)
        upper_left_corner_path = upper_left_corner_planner.plan_path_upper_left()
        self._path += upper_left_corner_path
        self._num_points += len(upper_left_corner_path)
        

class CircularPathPlanner:
    def __init__(self, radius, center=(0, 0), num_points=100,dir=None):
        self.radius = radius
        self.center = center
        self.num_points = num_points
        self.dir = dir

    def plan_path(self):
        path = []
        for i in range(self.num_points):
            theta = 2 * math.pi * i / self.num_points
            x = self.center[0] - self.radius * math.cos(theta)
            y = self.center[1] + self.radius * math.sin(theta)
            path.append((x, y,dir))
        return path


class ArcPathPlanner:
    def __init__(self, radius, center=(0, 0), num_points=2000,dir=None):
        self.radius = radius
        self.center = center
        self.num_points = num_points
        self.dir = dir
        self._ArcAngle = math.acos(5.5/9.15)

    def plan_path_lower(self):
        path = []
        for i in range(self.num_points):
            theta = 2*math.pi*i/self.num_points
            if theta > math.pi/2 - self._ArcAngle and theta < math.pi/2 + self._ArcAngle:
                x = self.center[0] - self.radius * math.cos(theta)
                y = self.center[1] + self.radius * math.sin(theta)
                path.append((x, y,dir))
        return path

    def plan_path_upper(self):
        path = []
        for i in range(self.num_points):
            theta = 2*math.pi*i/self.num_points
            if theta > math.pi/2 - self._ArcAngle and theta < math.pi/2 + self._ArcAngle:
                x = self.center[0] - self.radius * math.cos(theta)
                y = self.center[1] - self.radius * math.sin(theta)
                path.append((x, y,dir))
        return path    

class cornerPath:
    def __init__(self, radius, center=(0, 0), num_points=100,dir=None):
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
            path.append((x, y,dir))
        return path
    
    def plan_path_bottom_left(self):
        path = []
        for i in range(self.num_points):
            theta = 2 * math.pi * i / self.num_points
            if theta<=math.pi/2:
                x = self.center[0] + self.radius * math.sin(theta)
                y = self.center[1] + self.radius * math.cos(theta)
            path.append((x, y,dir))
        return path

    def plan_path_upper_right(self):
        path = []
        for i in range(self.num_points):
            theta = 2 * math.pi * i / self.num_points
            if theta<=math.pi/2:
                x = self.center[0] - self.radius * math.sin(theta)
                y = self.center[1] - self.radius * math.cos(theta)
            path.append((x, y,dir))
        return path
    
    def plan_path_upper_left(self):
        path = []
        for i in range(self.num_points):
            theta = 2 * math.pi * i / self.num_points
            if theta<=math.pi/2:
                x = self.center[0] + self.radius * math.cos(theta)
                y = self.center[1] - self.radius * math.sin(theta)
            path.append((x, y,dir))
        return path

