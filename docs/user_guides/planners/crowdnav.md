# Installation
 If you have already went through the installation process of SARL*, you can skip this guide.

## Activate poetry shell
```
cd arena-rosnav # navigate to the arena-rosnav directory
poetry shell
```
## Make sure to source the workspace environment
```bash
cd ../.. # navigate to the catkin_ws directory
source devel/setup.zsh # if you use bash: source devel/setup.bash 
```
## Install RVO2 (make sure SARL* planner is downloaded)
```bash
roscd sarl_star_ros
cd ../Python-RVO2
pip install Cython
python setup.py build
python setup.py install
```
## Install Crowdnav dependencies
```bash
roscd crowdnav-ros
cd scripts
pip install -e .
```
# Usage
## This planner can be chosen using the local_planner argument like so:
```bash
roslaunch arena_bringup start_arena.launch local_planner:=crowdnav # Make sure that your virtual env/poetry is activated
```
## For more details regarding usage, please refer to our [documentation](https://arena-benchmark.readthedocs.io/en/latest/user_guides/usage/)
