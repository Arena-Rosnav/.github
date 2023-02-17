# sarl_star
ROS implementation of the paper [SARL*: Deep Reinforcement Learning based Human-Aware Navigation for Mobile Robot in Indoor Environments](https://ieeexplore.ieee.org/abstract/document/8961764) presented in ROBIO 2019. This mobile robot navigation framework is implemented on a Turtlebot2 robot platform with lidar sensors (Hokuyo or RPlidar), integrating SLAM, path planning, pedestrian detection and deep reinforcement learning algorithms.

**Video demonstration can be found on**   [Youtube](https://youtu.be/izrrctuUd-g) or [bilibili](https://www.bilibili.com/video/av74520445).
Original work can be found [here](https://github.com/LeeKeyu/sarl_star).

## Build & Install
1. Activate poetry shell
```
cd arena-rosnav # navigate to the arena-rosnav directory
poetry shell
```
2. Make sure to source the workspace environment
```bash
cd ../.. # navigate to the catkin_ws directory
source devel/setup.zsh # if you use bash: source devel/setup.bash 
```
3. Install Python-RVO2
```bash
roscd sarl_star_ros
cd ../Python-RVO2
pip install Cython
python setup.py build
python setup.py install
```

4. Install Python dependencies

```
roscd sarl_star_ros
cd CrowdNav/
pip install -e .
```
# Usage
## This planner can be chosen using the local_planner argument like so:
```bash
roslaunch arena_bringup start_arena.launch local_planner:=sarl # Make sure that your virtual env/poetry is activated
```
## For more details regarding usage, please refer to our [documentation](https://arena-benchmark.readthedocs.io/en/latest/user_guides/usage/)

