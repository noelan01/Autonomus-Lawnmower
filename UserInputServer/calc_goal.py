import json
import numpy as np
import matplotlib.pyplot as plt

def load_pitch_data(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)
    return data


def get_paint_order():
    file_path = "data.json"
    pitch_data = load_pitch_data(file_path)

    shortside = round(float(pitch_data["shortside"]), 4)
    longside = round(float(pitch_data["longside"]), 4)
    penalty_width = round(float(pitch_data["penalty_width"]), 4)
    penalty_height = round(float(pitch_data["penalty_height"]), 4)

    print(f"penalty_height: {penalty_height} \npenalty_width: {penalty_width} \nshortside: {shortside} \nlongside: {longside} \n ")

    longside0 = {"start": (0,0),
                 "end": (0,longside)}
    shortside0 = {"start": longside0["end"],
                  "end": (shortside, longside)}
    longside1 = {"start": shortside0["end"],
                 "end": (shortside,0)}
    shortside1 = {"start": longside1["end"],
                  "end": (0, 0)}

    # Calculate the number of steps for each side based on the length
    n = 0.5
    num_steps_longside = int(longside / n)
    num_steps_shortside = int(shortside / n)

    # Generate points for each side with the same step size
    longside0_points = np.linspace(longside0["start"], longside0["end"], num_steps_longside + 2)[0:-1]
    shortside0_points = np.linspace(shortside0["start"], shortside0["end"], num_steps_shortside + 2)[0:-1]
    longside1_points = np.linspace(longside1["start"], longside1["end"], num_steps_longside + 2)[0:-1]
    shortside1_points = np.linspace(shortside1["start"], shortside1["end"], num_steps_shortside + 2)[0:]

    # Combine all points
    points = np.concatenate([longside0_points, shortside0_points, longside1_points, shortside1_points])

    print("Points in order:")
    for point in points:
        print(point)

    plt.figure(figsize=(8, 6))
    plt.plot(points[:, 0], points[:, 1], marker='o')
    plt.title('Football Pitch Points')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.gca().set_aspect('equal', adjustable='box')
    plt.grid(True)
    plt.show()


get_paint_order()
