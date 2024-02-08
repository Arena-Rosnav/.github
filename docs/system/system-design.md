[Task-manager]: ../images/system-design/task-manager-system-design.jpg "example image"

# System Design 
Arena-rosnav (arena) is implemented in a modular way with each class being outsourced into a seperate repository (different from the [first version of Arena-rosnav](https://github.com/ignc-research/arena-rosnav-3d)). The core repository is called arena-rosnav, which will install all other necessary modules. Figure 1 visualizes the general system design with all modules of the arena ecosystem. The following subsections, will provide a high-level explanation of each of those modules. For low-level descriptions of all functions and APIs, we refere to the [Packages]() chapter. For instructions on how to work with and/or extend them, the [Tutorials](../tutorials/) chapter provides more detailed explanations. 



## The Task Manager
The task manager is one of the main and most important components of the Arena platform. The main purpose is to have an abstracted module to generate different worlds and scenarios independently from the simulator in use. Currently, we integrated three simulators: Flatland (2D), Gazebo (3D), and Unity (3D). The task manager is responsible to generate different task setups such as randomly populated scenarios and also initializes the robot and planners by infering the respective manager classes such as the robot_manager.py or the obstacle_manager.py. Overall there are different task types currently provided, which are explained in more detail in [Packages/Task-Manager](../packages/task_generator.md).
[Figure 2][Task-manager], illustrates all modules within the task manager folder and how they interconnect. For a technical low-level overview of the system, please refer to [Packages/Task-Manager](../packages/task_generator.md).

![Task Manager System Design](../images/system-design/task-manager-system-design.jpg)

The task manager is designed in a way to easiliy integrate new simulators into the Arena-Rosnav platform by implementing the respective functions and using the API. Therefore, we refer to the technical low-level overview of the system in [Packages/Task-Manager](../packages/task_generator.md) and the tutorial on [How to add a new simulator](../tutorials/add_new_environment.md).

## The Map Manager
The map manager is another important entity to generate worlds and maps across all simulators. More information will follow shortly due Sunday 05.11.23.

## Arena Simulation Setup
TODO

## Overall System Design
TODO