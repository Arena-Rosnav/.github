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

We offer the among others following DRL planners:

- **Rosnav**: Our self-developed DRL planner. [Paper](https://arxiv.org/abs/2104.03616)
- **TRAIL**: DRL approach with velocity obstacles for the Jackal. 3rd place at the BARN challenge, created at [TRAIL Lab](https://sites.google.com/site/zhantengxie/home?authuser=0).
- **APPLR**: RL policy for online adjustment of navigation parameters of DWA for the Jackal. Produced by [UTexas](https://www.cs.utexas.edu/~xiao/Research/APPL/APPL.html#applr)
- **RLCA-ROS**: RL navigation policy trained on and intended for multi-agent systems. [Paper](https://arxiv.org/abs/1709.10082).
- **CADRL**: DEEPRL policy, works good across different robots, given that external information about environment's obstacles can be provided. [Paper](https://arxiv.org/abs/1805.01956).
- **SARL-Star**: Pedestrian-aware approach for indoor environments. [Paper](https://ieeexplore.ieee.org/abstract/document/8961764)
- **Crowdnav-ROS**: Used for mobile navigation in crowds by predicting the natural human behavior. [Paper](https://arxiv.org/abs/1809.08835)

As well as the following non-DRL planners:

- **Dragon**: Finds path by evaluating the gaps between obstacles in the environment. Suitable for static scenarios. Developed by [AMRLab](https://www.bezzorobotics.com/) for the BARN challenge.
- **TEB**
- **DWA**
- **MPC**

**Note:**
    All available planners can be found in ```../catkin_ws/src/planners/```.

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


