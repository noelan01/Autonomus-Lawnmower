import json
import numpy as np
import matplotlib.pyplot as plt

def load_pitch_data(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)
    return data


def get_paint_order(full_pitch):
    file_path = "data.json"
    pitch_data = load_pitch_data(file_path)

    shortside = round(float(pitch_data["shortside"]), 4)
    longside = round(float(pitch_data["longside"]), 4)
    penalty_width = 40.3
    penalty_height = 16.5
    center_circle_radius = 9.15

    print(f"penalty_height: {penalty_height} \npenalty_width: {penalty_width} \nshortside: {shortside} \nlongside: {longside} \n ")

    if not full_pitch:
        longside0 = {"start": (0,0),
                    "end": (0,longside)}
        shortside0 = {"start": longside0["end"],
                    "end": (shortside, longside)}
        longside1 = {"start": shortside0["end"],
                    "end": (shortside,0)}
        shortside1 = {"start": longside1["end"],
                    "end": (0, 0)}


    # Calculate the number of steps for each side based on the length
    n = 0.01
    # n = 5
    num_steps_longside = int(longside / n)
    num_steps_shortside = int(shortside / n)

    print(num_steps_longside)
    print(num_steps_shortside)

    # Generate points for each side with the same step size
    longside0_points = np.linspace(longside0["start"], longside0["end"], num_steps_longside + 2)[0:-1]
    shortside0_points = np.linspace(shortside0["start"], shortside0["end"], num_steps_shortside + 2)[0:-1]
    longside1_points = np.linspace(longside1["start"], longside1["end"], num_steps_longside + 2)[0:-1]
    shortside1_points = np.linspace(shortside1["start"], shortside1["end"], num_steps_shortside + 2)[0:]

    print(longside0_points)
    print(shortside0_points)

    # Combine all points
    points = np.concatenate([longside0_points, shortside0_points, longside1_points, shortside1_points])

    # points = np.concatenate([longside0_points[0:300]])
    points = np.round(points, decimals=2)


    print(f"Number of points {len(points)}")
    # print("Points in order:")
    # for point in points:
    #     print(point)

    plt.figure(figsize=(8, 6))
    plt.plot(points[:, 0], points[:, 1], marker='o')
    plt.title('Football Pitch Points')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.gca().set_aspect('equal', adjustable='box')
    plt.grid(True)
    plt.show()


def generate_route(radius, points_per_meter, midpoint, startpoint):
    num_points = int(2 * np.pi * radius * points_per_meter)
    theta = np.linspace(0, 2*np.pi, num_points)
    x = midpoint[0] + radius * np.cos(theta)
    y = midpoint[1] + radius * np.sin(theta)

    # Calculate the distance between the startpoint and each point on the circle
    distances = np.sqrt((x - startpoint[0])**2 + (y - startpoint[1])**2)

    # Find the index of the point on the circle closest to the startpoint
    start_index = np.argmin(distances)

    # Shift the x and y coordinates to start from the chosen startpoint
    x = np.roll(x, -start_index)
    y = np.roll(y, -start_index)

    return x, y

# Parameters
radius = 9.15  # radius of the circle
points_per_meter = 1  # number of points per meter on the circle
midpoint = (0, 0)  # midpoint at the center of the circle
startpoint = (radius, 0)  # start point on the outline of the circle

# Generate route
x, y = generate_route(radius, points_per_meter, midpoint, startpoint)

# Plot the circle and the route
plt.plot(x, y, label='Route', marker='.')
circle = plt.Circle(midpoint, radius, color='r', fill=False, label='Circle')
plt.gca().add_artist(circle)
plt.scatter(*midpoint, color='g', label='Midpoint')
plt.scatter(*startpoint, color='b', label='Startpoint')
plt.axis('equal')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Route Planning for Circle')
plt.legend()
plt.grid(True)
plt.show()


get_paint_order(full_pitch=False)
