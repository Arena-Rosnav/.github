# Flatland Simulator Interface Documentation

## Introduction
The `FlatlandSimulator` class implements various methods to create, spawn and remove robots and obstacles within the Flatland environment using ROS services and messages.


### Class Overview
Here are some key aspects of the class:

- **Initialization**: The class is initialized with a namespace to allow for separate environments or scenarios.
- **Robot and Obstacle Management**: The class provides methods for managing robots and obstacles, such as spawning, moving, and removing them within the Flatland environment.
- **Random Obstacle Generation**: It can generate random dynamic or static obstacles with specified characteristics.
- **YAML File Handling**: It handles the creation and management of YAML files for models, including robots and obstacles.
- **Dynamic Obstacle Management**: Methods for creating and handling dynamic obstacles.
- **Service Proxy and Publishers**: The class uses ROS service proxies for various services and publishers for moving robots.
- **Namespace Handling**: Namespace and topic handling for managing multiple robots and obstacles.

### Usage Example
Here's a basic example of how to use the Flatland Simulator interface:

```python
# Create an instance of the FlatlandSimulator
flatland_sim = FlatlandSimulator("my_flatland_namespace")

# Spawn a robot in the environment
flatland_sim.spawn_robot("my_robot", "robot_model_name")

# Move the robot to a new position
flatland_sim.move_robot([x, y, theta], "my_robot")

# Create and spawn dynamic or static obstacles
obstacle_model, obstacle_name, obstacle_position = flatland_sim.create_dynamic_obstacle()
flatland_sim.spawn_obstacle(obstacle_position, obstacle_model)

# Remove all obstacles from the environment
flatland_sim.remove_all_obstacles()
```

