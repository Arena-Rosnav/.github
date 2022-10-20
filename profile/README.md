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
- Dragon: from the [BARN challenge](https://www.cs.utexas.edu/~xiao/BARN_Challenge/BARN_Challenge.html) 
- Trail: from the [BARN challenge](https://www.cs.utexas.edu/~xiao/BARN_Challenge/BARN_Challenge.html) 
- Applr: a hybrid approach by [Xuesu et al.](https://arxiv.org/abs/2105.07620) 
- RLCA-ROS: a DRL-based colision avoidance approach from [Long et al.](https://github.com/Acmece/rl-collision-avoidance)
- CADRL: a DRL-based colision avoidance approach from [Everett et al.](https://github.com/mit-acl/cadrl_ros)
- SARL-Star
- Crowdnav-ROS: a DRL-based colision avoidance approach from [Chen et al.](https://github.com/vita-epfl/CrowdNav)
- TEB: a classic approach by [Rösmann et al.](https://github.com/rst-tu-dortmund/teb_local_planner) 
- DWA: the standard ROS local planning approach by [Marder-Eppstein et al.](http://wiki.ros.org/dwa_local_planner)
- MPC: a classic approach by [Rösmann et al.](https://github.com/rst-tu-dortmund/teb_local_planner)

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


