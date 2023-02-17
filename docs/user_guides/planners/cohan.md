# CoHAN Planner Multi

The extension of the [CoHAN Planner](https://github.com/sphanit/CoHAN_Planner) that allows usage within the Arena-Bench infrastructure. More details about the CoHAN Planner can be found [here](https://github.com/sphanit/CoHAN_Planner/blob/master/README.md).

# Installation (for Arena-Bench)
## Make sure to source the workspace environment
```bash
cd ../.. # navigate to the catkin_ws directory
catkin_make
source devel/setup.zsh # if you use bash: source devel/setup.bash 
```
# Usage
## This planner can be chosen using the local_planner argument like so:
```bash
roslaunch arena_bringup start_arena.launch local_planner:=cohan # Make sure that your virtual env/poetry is activated
```
## For more details regarding usage, please refer to our [documentation](https://arena-rosnav.readthedocs.io/en/latest/user_guides/usage/)

