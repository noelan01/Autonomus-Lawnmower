import numpy as np

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

    def update_point(self):
        self._current_point += 1


    def get_point(self):
        if self._current_point >= self._num_points:
            print("REACHED GOAL", self._path[self._current_point])
            print("")
            return (None, None)
        else:
            print("DESIRED POINT: ", self._path[self._current_point])
            print("")
            return self._path[self._current_point]
        
