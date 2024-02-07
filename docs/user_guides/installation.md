# Installation Guide

## Automatic Installation
To use the automatic installation, you will need `curl`. If you don't have it already installed, you can install it via
```bash
sudo apt install curl
```
Once you have curl installed, start installing Arena-Rosnav by running:
```bash
curl https://raw.githubusercontent.com/Arena-Rosnav/arena-rosnav/master/install.sh | bash
```
After the script completes, open a **new** terminal and run:
```bash
curl https://raw.githubusercontent.com/Arena-Rosnav/arena-rosnav/master/install2.sh | bash
```

### Things to note
ROS and by extension Arena-Rosnav are *big*. You should have at least 30GB of storage available.
Also, this installation will probably take about half an hour, varying depending on the speed of your internet and machine.
Arena-Rosnav is only intended to be run on Ubuntu 20.04. If you are intending to run it on a VM, be aware that you need a GPU for certain functions.

<!-- ## Advanced Installation

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

Clone the Arena Benchmark repository in any existing **catkin workspace** or [create a new workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace). You also need to have [catkin-tools](https://catkin-tools.readthedocs.io/en/latest/installing.html) installed.

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
rosdep update && rosdep install --from-paths src --ignore-src -r -y
catkin build
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
rosdep update && rosdep install --from-paths src --ignore-src -r -y
sudo apt update && sudo apt install -y 
    libopencv-dev \ 
    liblua5.2-dev \ 
    ros-noetic-nlopt \ 
    libarmadillo-dev \
    
```

## Install stable-baselines3

In order to run the trainings process, you need to have our fork of the stable baselines 3 library installed.

```bash
pip install setuptools==57.1.0 psutil==5.9.4 wheel==0.41.2
cd ../utils-extern/misc/stable-baselines3/ && pip install -e .
```

## Build your workspace

```bash
cd ../../../.. && catkin build
```

!!! note

    If packages are missing during the build process, simply add them with `poetry add <package_name>`.

## Source the build

```bash
source devel/setup.zsh
```
## Install local planners
#### Please refer to our [planners overview](planners_overview.md) for installation steps of the local planners.

Finished! Check out the [Usage](usage.md) to start using Arena Rosnav.

Remember to always have the poetry shell active and the catkin workspace sourced when using Arena Rosnav. -->

## Testing
With the activated environment, test your installation by running the command

```bash
roslaunch arena_bringup start_arena.launch simulator:=gazebo
```

This should open gazebo and rviz successfully.

# Troubleshouting
If you encounter errors during the build process due to missing packages, add them

<!-- # install lua
````
sudo apt install liblua5.1-0-dev 
```` -->

## install Ipopt
````bash
git clone https://github.com/coin-or/Ipopt
cd Ipopt
./configure
make
sudo make install
````
