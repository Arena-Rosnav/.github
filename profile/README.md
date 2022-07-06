# Arena Rosnav

Arena Rosnav is a set of ROS packages and tools, forming an infrastructure
for benchmarking different navigation approaches in different environments
and train neural networks for collision avoidance, while being scalable and 
adjustable for your specific need.


## Repositories

The oganizations repos are divided into planners, simulation and training environments, and general utils.
Planners and general utils are designed to work in all simulation and training environments.

### Planners

Planners are essential packages and control the robot, which is simulated in the environments.

We offer the following planners:

- ROSNav: Our own planner based on neural networks.
- Dragon: TODO
- Trail: TODO
- Applr: TODO
- RLCA-ROS: TODO
- CADRL: TODO
- SARL-Star: TODO
- Crowdnav-ROS

### Simulation and Training environment

Training and simulation environment are the core of Arena ROSNav, it combines the utils and planners.

We offer following environments:

- arena-rosnav: 2D simulation environment for benchmarking and training.
- arena-rosnav-3d: 3D environment especially designed for benchmarking.

### General Utils

General utils packages are used in the simulation environments and cover one specific task.

We have the following utils repos: 

- task-generator: Prepares the simulation environment and offers specific tasks for the robot to do.
- waypoint-generator: Generates the plan and subgoal for the planners.
- arena-tools: A collection of arena specific tools for generating own robot footprints and scenarios that can be used with the task generator.
- arena-simulation-setup: Contains robots, maps, and a bunch of other decleration files used in the simulations.
- arena-utils: A collection of different utils packages for planner, simulation and visualization.


