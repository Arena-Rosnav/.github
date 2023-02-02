# Installation Guide

## Requirements

- ROS: Arena-Rosnav is developed for Ubuntu 20.04. That means, for running Arena Benchmark you need to have [ROS Noetic](http://wiki.ros.org/noetic/Installation) installed.
- [Poetry](https://python-poetry.org/) and [PyEnv](https://github.com/pyenv/pyenv): we recommend you to use a dependency manager for Python. We use Poetry because its easy to set up and easy to use, but you are open to use any other manager or virtual environment provider.

## Clone the repository

Clone the Arena Benchmark repository in any existing **catkin workspace** or [create a new workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

```bash
git clone git@github.com:Arena-Rosnav/arena-rosnav.git
```

## Update ROS workspace

For running Arena Benchmark you need a set of other packages. The majority of them can be installed and cloned directly with ROS. Therefore, you should navigate into the newly cloned repository and run following command:

```bash
rosws update
```

## Install required Python packages

You should also install the necessary Python packages. You can do so by first activating the poetry shell and then installing all packages listed in the _pyproject.toml_.

```bash
poetry shell && poetry install
```

## Install additional Packages

At last, you need to install a whole bunch of ros packages for running all planners and for other purposes.

```bash
sudo apt update && sudo apt install -y libopencv-dev liblua5.2-dev \
    ros-noetic-navigation ros-noetic-teb-local-planner \
    ros-noetic-mpc-local-planner libarmadillo-dev ros-noetic-nlopt \
    ros-noetic-turtlebot3-description ros-noetic-turtlebot3-navigation \
    ros-noetic-lms1xx ros-noetic-velodyne-description ros-noetic-hector-gazebo \
    ros-noetic-ira-laser-tools
```

## Install stable-baselines3

In order to run the trainings process, you need to have our fork of the stable baselines 3 library installed.

```
cd ../utils/stable-baselines3 && pip install -e .
```

## Build your workspace

```
cd ../../.. && catkin_make
```

!!! note

    If packages are missing during the build process, simply add them with `poetry add <package_name>`.

## Source the build

```
source devel/setup.bash
```

Finished! Check out the [Usage](usage.md) to start using Arena Rosnav.

Remember to always have the poetry shell active and the catkin workspace sourced when using Arena Rosnav.
