import csv
import matplotlib.pyplot as plt
import numpy as np
import os
import random
import time
from plotter.navigation_map_plotter import is_point_in_obstacle, plot_navigation_map
from dataset.planner import AStarPlanner

ITERATIONS = 5
show_animation = True
MAP_WIDTH = 120
MAP_HEIGHT = 60
CELL_SIZE = 5


def save_path_data(start_x, start_y, goal_x, goal_y, path_x, path_y):
    data = {
        "Start_X": start_x,
        "Start_Y": start_y,
        "Goal_X": goal_x,
        "Goal_Y": goal_y,
        "Path_X": path_x,
        "Path_Y": path_y,
    }

    file_exists = os.path.isfile("path_data.csv")

    with open("path_data.csv", "a", newline="") as csvfile:
        fieldnames = data.keys()
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

        if not file_exists:
            writer.writeheader()

        writer.writerow(data)


def initialize_grid(obstacles, map_width, map_height, cell_size):
    grid = [
        [True for _ in range(map_height // cell_size + 1)]
        for _ in range(map_width // cell_size + 1)
    ]

    for obs in obstacles:
        for point in obs:
            x, y = int(point[0] // cell_size), int(point[1] // cell_size)
            if 0 <= x < len(grid) and 0 <= y < len(grid[0]):
                grid[x][y] = False  # Mark cell as occupied

    return grid


def generate_valid_point(grid, cell_size, map_width, map_height):
    free_cells = [
        (x, y) for x in range(len(grid)) for y in range(len(grid[0])) if grid[x][y]
    ]
    if not free_cells:
        raise Exception("No free space available.")

    while True:
        cell = random.choice(free_cells)
        x = random.randint(
            cell[0] * cell_size, min((cell[0] + 1) * cell_size, map_width)
        )
        y = random.randint(
            cell[1] * cell_size, min((cell[1] + 1) * cell_size, map_height)
        )
        return (x, y)


def main():
    obstacles = plot_navigation_map()
    grid = initialize_grid(obstacles, MAP_WIDTH, MAP_HEIGHT, CELL_SIZE)
    planner = AStarPlanner(list(zip(*obstacles)), grid_size=1, robot_radius=1.0)

    for _ in range(ITERATIONS):

        # Generate valid start and goal points
        start_point = generate_valid_point(grid, CELL_SIZE, MAP_WIDTH, MAP_HEIGHT)
        goal_point = generate_valid_point(grid, CELL_SIZE, MAP_WIDTH, MAP_HEIGHT)

        sx, sy = start_point
        gx, gy = goal_point

        rx, ry = planner.planning(sx, sy, gx, gy)

        if rx and ry:  # Check if a valid path was found
            save_path_data(sx, sy, gx, gy, rx, ry)  # Save path data to CSV

        if show_animation:  # pragma: no cover
            plt.figure(figsize=(10, 6))
            plt.plot(*zip(*obstacles), "sk", label="Obstacles")
            plt.plot(sx, sy, "^r", label="Start Point")
            plt.plot(gx, gy, "^c", label="Goal Point")
            plt.plot(rx, ry, "-r", label="Planned Path")
            plt.legend()
            plt.show()

        print("Path planning iteration completed.")


if __name__ == "__main__":
    main()
