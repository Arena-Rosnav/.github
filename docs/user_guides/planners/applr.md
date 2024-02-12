# Adaptive Planner Parameter Learning from Reinforcement

This repository contains the [APPLR](https://www.cs.utexas.edu/~xiao/papers/applr.pdf) navigation approach adapted for usage in arena-rosnav platform.    
For original work refer to [UTexas](https://www.cs.utexas.edu/~xiao/Research/APPL/APPL.html#applr) and [original repository](https://github.com/Daffan/nav-competition-icra2022/tree/applr).  

**Please note** APPLR is appropriate for usage on **only** the [Jackal platform](https://clearpathrobotics.com/jackal-small-unmanned-ground-vehicle/).

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
<!-- ## Install Python dependencies
```sh
roscd applr
pip install -r requirements.txt # Make sure your virtual environment is activated
``` -->
# Usage
## This planner can be chosen using the local_planner argument like so:
```sh
roslaunch arena_bringup start_arena.launch local_planner:=applr # Make sure that your virtual env/poetry is activated
```
## For more details regarding usage, please refer to our [documentation](https://arena-rosnav.readthedocs.io/en/latest/user_guides/usage/)
