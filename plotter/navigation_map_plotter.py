"""
navigation_map_plotter.py

This module provides a function to plot a navigation map with predefined obstacles,
a customizable start point, and a goal point. The map can be used in pathfinding algorithms
and robotics simulations.

The main function, plot_navigation_map, plots the map based on the specified start and goal
points, and draws predefined obstacles. It ensures that the start and goal points are not
located on any obstacle and are within the navigable area defined by the map's dimensions.

Usage Examples:

1. Plotting a map with default start and goal points:
   plot_navigation_map()

2. Plotting a map with custom start and goal points:
   start_point = (20, 10)
   goal_point = (100, 50)
   plot_navigation_map(start_point=start_point, goal_point=goal_point)

3. Checking if a point is within any obstacle (useful for dynamic obstacle addition or point 
   validation):
   
   point = (30, 45)
   obstacles = [
       [(0, 30), (0, 40), (7, 40), (7, 30)],
       [(15, 30), (15, 40), (26, 40), (26, 30)],
       # Add more obstacles as needed
   ]
   is_in_obstacle = is_point_in_obstacle(point, obstacles)
   print(f"Point {point} is in an obstacle: {is_in_obstacle}")

Please ensure that any customization of start, goal points, or obstacles follows the expected 
formats.
"""

from typing import List, Tuple
import matplotlib.pyplot as plt
from matplotlib import patches
from matplotlib.path import Path


# Define a type alias for a point and an obstacle
Point = Tuple[float, float]
Obstacle = List[Point]


def is_point_in_obstacle(point: Point, obstacles: List[Obstacle]) -> bool:
    """
    Check if the given point is within any of the defined obstacles.

    Parameters:
    point (tuple): The coordinates for the point to check (x, y).
    obstacles (list): A list of obstacle corner points, each defined as a list of tuples.

    Returns:
    bool: True if the point is within any obstacle, False otherwise.
    """
    for corners in obstacles:
        obstacle_path = Path(corners)
        if obstacle_path.contains_point(point):
            return True
    return False


def plot_navigation_map(
    start_point: Point = (10, 50), goal_point: Point = (110, 10)
) -> List[Obstacle]:
    """
    Plot a navigation map with predefined obstacles, a customizable start point, and goal point.

    Parameters:
    start_point (tuple): The coordinates for the start point, default is (10, 50).
    goal_point (tuple): The coordinates for the goal point, default is (110, 10).

    A red triangle indicates the start point, and a cyan triangle indicates the goal point.
    The map is enclosed with a border to define the navigable area.

    Returns:
    List[Obstacle]: A list of obstacles with their corner points.
    """
    # Define the corner points of the obstacles as rectangles on the map
    obstacle_corners: List[Obstacle] = [
        # Each tuple represents the bottom left and top right corners of rectangles
        [(0, 30), (0, 40), (7, 40), (7, 30)],
        [(15, 30), (15, 40), (26, 40), (26, 30)],
        [(20, 40), (20, 60), (21, 60), (21, 40)],
        [(34, 30), (34, 40), (40, 40), (40, 30)],
        [(36, 28), (36, 30), (40, 30), (40, 28)],
        [(40, 28), (40, 33), (42, 33), (42, 28)],
        [(40, 33), (40, 47), (42, 47), (42, 33)],
        [(42, 33), (42, 47), (68, 47), (68, 33)],
        [(62, 30), (62, 33), (68, 33), (68, 30)],
        [(42, 10), (42, 13), (62, 13), (62, 10)],
        [(62, 10), (62, 21), (68, 21), (68, 10)],
        [(36, 10), (36, 20), (42, 20), (42, 10)],
        [(36, 0), (36, 3), (68, 3), (68, 0)],
        [(76, 20), (76, 24), (100, 24), (100, 20)],
        [(88, 0), (88, 20), (100, 20), (100, 0)],
        [(109, 24), (109, 30), (120, 30), (120, 24)],
        [(116, 30), (116, 60), (120, 60), (120, 30)],
        [(68, 56), (68, 60), (116, 60), (116, 56)],
        [(40, 55), (40, 60), (68, 60), (68, 55)],
    ]

    # Check if the points are within the defined limits
    assert (
        0 < start_point[0] < 120 and 0 < start_point[1] < 60
    ), "Start point out of bounds"
    assert (
        0 < goal_point[0] < 120 and 0 < goal_point[1] < 60
    ), "Goal point out of bounds"

    # Ensure start and goal points are not on obstacles
    assert not is_point_in_obstacle(
        start_point, obstacle_corners
    ), "Start point is located on an obstacle."
    assert not is_point_in_obstacle(
        goal_point, obstacle_corners
    ), "Goal point is located on an obstacle."

    # Plot each obstacle with the defined corners
    for corners in obstacle_corners:
        polygon = patches.Polygon(corners, closed=True, color="k")
        plt.gca().add_patch(polygon)

    # Plot the start and goal points
    plt.plot(*start_point, "^r")
    plt.plot(*goal_point, "^c")

    # Set the limits of the plot to frame the map properly
    plt.xlim(0, 120)
    plt.ylim(0, 60)

    # Add a black border around the plot to represent the boundaries of the navigable area
    plt.gca().add_patch(
        patches.Rectangle((0, 0), 120, 60, linewidth=4, edgecolor="k", facecolor="none")
    )

    # Display the plot with the obstacles and points
    plt.show()

    return obstacle_corners


if __name__ == "__main__":
    plot_navigation_map()
