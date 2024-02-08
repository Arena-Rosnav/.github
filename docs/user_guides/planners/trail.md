# TRAIL

TRAIL is a DRL-VO navigation approach created at Temple University with the [BARN challenge](https://www.cs.utexas.edu/~xiao/BARN_Challenge/BARN_Challenge.html) in mind.  
This repository contains the TRAIL policy modified to function within arena-rosnav platform.  
Original repository can be found [here](https://github.com/TempleRAIL/nav-competition-icra2022-drl-vo)

# Installation
## Activate poetry shell
```sh
cd ~/catkin_ws/src/arena-rosnav # Navigate to your arena-bench location
poetry shell
```
## Make sure to source the workspace environment
```sh
cd ../.. # navigate to the catkin_ws directory
catkin_make
source devel/setup.zsh # if you use bash: source devel/setup.bash 
```
# Usage
## This planner can be chosen using the local_planner argument like so:
```sh
roslaunch arena_bringup start_arena.launch local_planner:=trail # Make sure that your virtual env/poetry is activated
```
## For more details regarding usage, please refer to our [documentation](https://arena-rosnav.readthedocs.io/en/latest/user_guides/usage/)
