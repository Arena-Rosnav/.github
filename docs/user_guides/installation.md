# Installation Guide

## Automatic Installation
To automatically install Arena-Rosnav and its dependencies, just run our install script:
```bash
curl https://raw.githubusercontent.com/Arena-Rosnav/arena-rosnav/master/install.sh | bash
```

## Advanced Installation

### Requirements

- ROS: Arena-Rosnav is developed for Ubuntu 20.04. That means, for running Arena Benchmark you need to have [ROS Noetic](http://wiki.ros.org/noetic/Installation) installed.
- [Poetry](https://python-poetry.org/) and [PyEnv](https://github.com/pyenv/pyenv): we recommend you to use a dependency manager for Python. We use Poetry because its easy to set up and easy to use, but you are open to use any other manager or virtual environment provider.
- We recommend using the zsh shell as it provides a bunch of useful pluggins such as autocompletion, which makes it easier to run the commands as some of the arena-rosnav commands takes in a large number of arguments. Here is a quick installation guide to install it with just two commands.

### Fast zsh-install & setup
We recommend using zsh. You can install it by just copy pasting this one command:

```bash
sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.1.2/zsh-in-docker.sh)" -- \
     -p git \
     -p ssh-agent \
     -p https://github.com/zsh-users/zsh-autosuggestions \
     -p https://github.com/zsh-users/zsh-completions## Fast zsh-install & setup
```
Afterwards
```bash
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
```bash
source $HOME/.zshrc
```
and follow the setup wizard.

Note that you have to install VIM if you dont have it yet:

```bash
sudo apt-get update
sudo apt-get -y install vim
```
or

```bash
sudo apt update
sudo apt -y install vim
```


## Clone the repository

Clone the Arena Benchmark repository in any existing **catkin workspace** or [create a new workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
cd src
```
then clone the arena-rosnav repo into the src folder

````bash
git clone https://github.com/Arena-Rosnav/arena-rosnav
````
## Update ROS workspace
For running Arena Benchmark you need a set of other packages. The majority of them can be installed and cloned directly with ROS. Therefore, you should navigate into the newly cloned repository and run following command:

```bash
cd arena-rosnav
rosws update
```

## Install required Python packages

You should also install the necessary Python packages. You can do so by first activating the poetry shell and then installing all packages listed in the _pyproject.toml_.

```bash
poetry shell
poetry install
```

## Install additional Packages

At last, you need to install a whole bunch of ros packages for running all planners and for other purposes.

```bash
sudo apt update && sudo apt install -y 
    libopencv-dev liblua5.2-dev \ 
    ros-noetic-navigation ros-noetic-teb-local-planner \
    ros-noetic-mpc-local-planner libarmadillo-dev ros-noetic-nlopt \
    ros-noetic-turtlebot3-description ros-noetic-turtlebot3-navigation \
    ros-noetic-lms1xx ros-noetic-velodyne-description ros-noetic-hector-gazebo \
    ros-noetic-ira-laser-tools \
    
```

## Install stable-baselines3

In order to run the trainings process, you need to have our fork of the stable baselines 3 library installed.

```
pip install setuptools==57.1.0 psutil==5.9.4 wheel==0.41.2
cd ../utils-extern/misc/stable-baselines3/ && pip install -e .
```

## Build your workspace

```
cd ../../../.. && catkin_make
```

!!! note

    If packages are missing during the build process, simply add them with `poetry add <package_name>`.

## Source the build

```
source devel/setup.zsh
```
## Install local planners
#### Please refer to our [planners overview](planners_overview.md) for installation steps of the local planners.

Finished! Check out the [Usage](usage.md) to start using Arena Rosnav.

Remember to always have the poetry shell active and the catkin workspace sourced when using Arena Rosnav.

## Testing
With the activated environment, test your installation by running the command

```
roslaunch arena_bringup start_arena.launch pedsim:=true simulator:=gazebo task_mode:=scenario scenario_file:=scenario_2.json map_file:=map_empty local_planner:=teb model:=jackal
```

This should open gazebo and rviz successfully.

# Troubleshouting
If you encounter errors during the build process due to missing packages, add them

# install lua
````
sudo apt install liblua5.1-0-dev 
````

# install Ipopt
````bash
git clone https://github.com/coin-or/Ipopt
cd Ipopt
./configure
make
sudo make install
````
