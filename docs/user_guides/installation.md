# Installation Guide

**Arena4 is intended to run on Ubuntu 22.04 (ROS2 Humble).** Other platforms might work with limited support.
We additionaly provide a [Dockerfile](https://github.com/voshch/arena-rosnav/blob/humble/installers/Dockerfile) to run Arena4 on different platforms.

## Automatic Installation


Start installing Arena4 by running:
```sh
curl https://raw.githubusercontent.com/voshch/arena-rosnav/refs/heads/humble/installers/install.sh > install.sh
bash install.sh
```

### Things to note
ROS2 and by extension Arena4 are *big*. ROS2 and Gazebo are compiled in the workspace. This has considerable storage space and compilation time requirements (on the first compilation). See [#Troubleshooting](#troubleshooting) for possible errors.


<!-- ## Advanced Installation 

### Requirements

- ROS: Arena-Rosnav is developed for Ubuntu 20.04. That means, for running Arena Benchmark you need to have [ROS Noetic](http://wiki.ros.org/noetic/Installation) installed.
- [Poetry](https://python-poetry.org/) and [PyEnv](https://github.com/pyenv/pyenv): we recommend you to use a dependency manager for Python. We use Poetry because its easy to set up and easy to use, but you are open to use any other manager or virtual environment provider.
- We recommend using the zsh shell as it provides a bunch of useful pluggins such as autocompletion, which makes it easier to run the commands as some of the arena-rosnav commands takes in a large number of arguments. Here is a quick installation guide to install it with just two commands.

### Fast zsh-install & setup
We recommend using zsh. You can install it by just copy pasting this one command:

```sh
sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.1.2/zsh-in-docker.sh)" -- \
     -p git \
     -p ssh-agent \
     -p https://github.com/zsh-users/zsh-autosuggestions \
     -p https://github.com/zsh-users/zsh-completions## Fast zsh-install & setup
```
Afterwards
```sh
     git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions
     git clone --depth=1 https://github.com/romkatv/powerlevel10k.git ${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/themes/powerlevel10k
     vim $HOME/.zshrc
```
Set ZSH_THEME="powerlevel10k/powerlevel10k" in ~/.zshrc

Set plugins=(pluggin1 
    zsh-autosuggestions pluggin2 ... other pluggins
)

Note there is no comma to seperate between pluggins.

Finally,
```sh
source $HOME/.zshrc
```
and follow the setup wizard.

Note that you have to install VIM if you dont have it yet:

```sh
sudo apt-get update
sudo apt-get -y install vim
```
or

```sh
sudo apt update
sudo apt -y install vim
```


## Clone the repository

Clone the Arena Benchmark repository in any existing **catkin workspace** or [create a new workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace). You also need to have [catkin-tools](https://catkin-tools.readthedocs.io/en/latest/installing.html) installed.

```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
rosdep update && rosdep install --from-paths src --ignore-src -r -y
catkin build
cd src
```
then clone the arena-rosnav repo into the src folder

````sh
git clone https://github.com/Arena-Rosnav/arena-rosnav
````
## Update ROS workspace
For running Arena Benchmark you need a set of other packages. The majority of them can be installed and cloned directly with ROS. Therefore, you should navigate into the newly cloned repository and run following command:

```sh
cd arena-rosnav
rosws update
```

## Install required Python packages

You should also install the necessary Python packages. You can do so by first activating the poetry shell and then installing all packages listed in the _pyproject.toml_.

```sh
poetry shell
poetry install
```

## Install additional Packages

At last, you need to install a whole bunch of ros packages for running all planners and for other purposes.

```sh
rosdep update && rosdep install --from-paths src --ignore-src -r -y
sudo apt update && sudo apt install -y 
    libopencv-dev \ 
    liblua5.2-dev \ 
    ros-noetic-nlopt \ 
    libarmadillo-dev \
    
```

## Install stable-baselines3

In order to run the trainings process, you need to have our fork of the stable baselines 3 library installed.

```sh
pip install setuptools==57.1.0 psutil==5.9.4 wheel==0.41.2
cd ../utils-extern/misc/stable-baselines3/ && pip install -e .
```

## Build your workspace

```sh
cd ../../../.. && catkin build
```

!!! note

    If packages are missing during the build process, simply add them with `poetry add <package_name>`.

## Source the build

```sh
source devel/setup.zsh
```
## Install local planners
#### Please refer to our [planners overview](planners_overview.md) for installation steps of the local planners.

Finished! Check out the [Usage](usage.md) to start using Arena Rosnav.

Remember to always have the poetry shell active and the catkin workspace sourced when using Arena Rosnav. -->

## Testing
With the activated environment, test your installation by running the command

```sh
ros2 launch arena_bringup start_arena.launch.py
```

This should open gazebo and rviz successfully.

# Development
The workspace is built with symlinks, modifying existing python files automatically changes the run-time behavior (after relaunching).

For adding/removing files or recompiling C++, **compile only with**
```sh
. colcon_build
```
in the workspace root. This ensures that the python environment is activated and all build flags are set.
Arguments passed to `. colcon_build` are passed through to the underlying build command.


# Troubleshooting

## Missing Packages
If you encounter errors during the build process due to missing packages, add them to the workspace and pull request an updated installer.

## OOM Errors during Compilation
Gazebo may fail to compile on weaker devices. Limit the CPU and memory usage (for the first full build only) with
```sh
. colcon_build --executor sequential --parallel-workers 1
```

Alternatively, temporarily increase the size of your swapfile.
