# all_in_one
This repository contains the results of work presented in [All-in-One: A DRL-based Control Switch Combining State-of-the-art Navigation Planners](https://arxiv.org/pdf/2109.11636.pdf). The All-In-One Planner (AIO) chooses from a given list of local planners in each iteration with the objective to combine the strengths of learning-based planners and classical planners like TEB / MPC.

This code is intended for usage with Arena-Rosnav infrastructure.
For more information regarding AIO please refer to the [original repository](https://github.com/ignc-research/all-in-one-DRL-planner).


<p align="center">
 <img width="800" height="400" src="../../../images/gifs/all_in_one.gif"> 
<p>

Currently we offer a control policy for 4 different robot models:
- Turtlebot3 Burger
- Jackal
- Robotino
- Youbot  

# Installation
## Activate poetry shell
```bash
cd ~/catkin_ws/src/arena-rosnav # Navigate to your arena-rosnav location
poetry shell
```
## Make sure to source the workspace environment
```bash
cd ../.. # navigate to the catkin_ws directory
catkin_make
source devel/setup.zsh # if you use bash: source devel/setup.bash 
```
# Usage
## This planner can be chosen using the local_planner argument like so:
```bash
roslaunch arena_bringup start_arena.launch local_planner:=aio # Make sure that your virtual env/poetry is activated
```
## For more details regarding usage, please refer to our [documentation](https://arena-rosnav.readthedocs.io/en/latest/user_guides/usage/)
