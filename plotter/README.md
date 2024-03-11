# Navigation Map Plotter

This Python script plots a navigation map with predefined obstacles, a customizable start point, and a goal point. It's useful for pathfinding algorithms and robotics simulations.

## Features

- Plotting a map with obstacles
- Customizable start and goal points
- Ensures points are not located on obstacles
- Uses Matplotlib for plotting

## Prerequisites

- Python 3.6 or higher
- Matplotlib library

## Setup and Installation

1. **Install Python**: Ensure Python 3.6 or higher is installed on your system. You can download it from [python.org](https://www.python.org/downloads/).

2. **Virtual Environment** (Optional but recommended):
   - Open a terminal or command prompt.
   - Navigate to your project directory and create a virtual environment:
     ```
     python -m venv venv
     ```
   - Activate the virtual environment:
     - Windows: `.\venv\Scripts\activate`
     - macOS/Linux: `source venv/bin/activate`

3. **Install Matplotlib**:
   - With the virtual environment activated, install Matplotlib:
     ```
     pip install matplotlib
     ```

## Running the Script

- **To Run with Default Parameters**:
  - Open a terminal or command prompt.
  - Navigate to the directory containing `navigation_map_plotter.py`. 
  - Simply run the script with Python:
    ```bash
    python navigation_map_plotter.py
    ```
This will plot the navigation map using the default start and goal points along with the predefined obstacles.

- **Custom Start and Goal Points**:
To customize the start and goal points, modify the script or extend the `if __name__ == "__main__":` block at the end of the script before running it.

Example modification for custom points:
```python
if __name__ == "__main__":
    custom_start = (20, 10)  # Custom start point
    custom_goal = (100, 50)  # Custom goal point
    plot_navigation_map(start_point=custom_start, goal_point=custom_goal)
```
Then, run the script as described above.

## Additional Notes:

- **Matplotlib Backend**: Depending on your system's configuration, Matplotlib might require a specific backend for displaying plots. The script assumes the default backend works fine for your setup. If you encounter issues displaying the plot window, you might need to configure Matplotlib to use a different backend.

- **Running in an IDE**: If you're using an Integrated Development Environment (IDE) like PyCharm, VSCode, or Jupyter notebook, you can run the script by opening the `navigation_map_plotter.py` file in the IDE and using the IDE's run functionality. Ensure the environment configured in the IDE has Matplotlib installed.

This guide should help you get started with running and modifying `navigation_map_plotter.py` on your system.