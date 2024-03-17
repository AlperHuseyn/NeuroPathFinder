# NeuroPathFinder

NeuroPathFinder is an innovative project aimed at enhancing pathfinding algorithms and robotics simulations through the integration of complex indoor environment mapping and advanced pathfinding techniques. This repository hosts a module that plots a navigation map, which can be utilized in pathfinding algorithms and robotics simulations, and will soon include implementations of various pathfinding algorithms.

## Features

- **Navigation Map Plotting:** A Python script capable of plotting a navigation map with predefined obstacles, customizable start and goal points, ensuring these points are not located on obstacles. This feature is crucial for testing pathfinding algorithms in a simulated environment.

- **Pathfinding Algorithms (Coming Soon):** Implementation of various pathfinding algorithms to navigate through the plotted maps efficiently.

## Getting Started

### Prerequisites

- Python 3.6 or higher
- Matplotlib library

### Installation

Ensure Python 3.6 or higher is installed on your system. You can download it from python.org.

It's recommended to use a virtual environment for Python projects. To set it up:

```bash
python -m venv venv
source venv/bin/activate  # On macOS/Linux
.\venv\Scripts\activate  # On Windows
```

### Install Matplotlib

To install Matplotlib, run the following command in your terminal:

```bash
pip install matplotlib
```

### Running the Navigation Map Plotter

To run the Navigation Map Plotter, navigate to the plotter directory and execute:

```bash
python navigation_map_plotter.py
```

This command plots the navigation map using the default start and goal points along with the predefined obstacles. For custom start and goal points, modify the script accordingly.

### Research Paper

The repository includes a research paper titled "Navigating in Complex Indoor Environments: A Comparative Study". This paper offers a comprehensive comparison of various pathfinding algorithms in complex indoor environments, highlighting the challenges and proposing innovative solutions to enhance navigation accuracy and efficiency.

[Read the Research Paper](https://github.com/AlperHuseyn/NeuroPathFinder/blob/main/Navigating%20in%20Complex%20Indoor%20Environments%3A%20A%20Comparative%20Study.pdf)

### Contributing

Contributions to NeuroPathFinder are welcome! Whether it's implementing new algorithms, enhancing the navigation map plotter, or improving the documentation, your contributions are valuable to us.

### License

This project is licensed under the MIT License - see the [LICENSE file](https://github.com/AlperHuseyn/NeuroPathFinder/blob/main/license/LICENSE.md) for details.