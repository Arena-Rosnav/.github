# Installation Guide

**Make absolutely sure that no python environment is currently sourced.**
**Make absolutely sure that ros2, if installed, is not sourced.**

We officially support native Ubuntu 22.04.
Ubuntu under WSL also works with limited support (i.e. only Gazebo).

## Automatic Installation
To use the automatic installation, you will need `curl` and nothing else. You can install it via
```sh
sudo apt install curl
```
Once you have curl installed, start installing Arena-Rosnav by running:
```sh
curl https://raw.githubusercontent.com/Arena-Rosnav/arena-rosnav/humble/installers/install.sh > install.sh
bash install.sh
```

The installer script is fault-tolerant (bad network, out of memory, etc.). If the installation fails, re-run the script and it will pick up from the last point.

The installation was successful when you get a 'installation finished' message.

### Optional Dependencies
By default, only ROS 2 and the Arena ecosystem are installed.

At the end of the installation, the installer will ask you if you want to install optional dependencies (currently Gazebo, Isaac, planners).
If you want to install these dependencies at a later point, run the in-place installer from the workspace root directory using `bash src/arena/arena-rosnav/installers/install.sh` and wait to be prompted.

### Things to note
We compile ROS 2 inside the workspace.
ROS 2 and the Arena ecosystem are *big*. You should have at least 80GB of storage available.
This installation will probably take over an hour.
Arena is only intended to be run on Ubuntu 22.04.

## Next Steps
Explore Arena's [usage](usage.md)
