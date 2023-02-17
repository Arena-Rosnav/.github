# cadrl-ros

## Installation

For CADRL, you need to create a virtual environment environment with python 3.6. and tensorflow 1.4.  
We recommend using virtualenv & virtualenvwrapper.

1. Install virtual environment and wrapper (as root or admin! with sudo) on your local pc (without conda activated. Deactivate conda env, if you have one active)

```bash
sudo apt install python3-pip
sudo pip3 install --upgrade pip
sudo pip3 install virtualenv
sudo pip3 install virtualenvwrapper
which virtualenv   # should output /usr/local/bin/virtualenv
```

2. Create venv folder inside your home directory

```bash
cd $HOME && mkdir python_env   # create a venv folder in your home directory
```

3. Add exports into your .bashrc or .zshrc:

```bash
echo "export WORKON_HOME=$HOME/python_env   #path to your venv folder
export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3   #path to your python3
export VIRTUALENVWRAPPER_VIRTUALENV=/usr/local/bin/virtualenv
source /usr/local/bin/virtualenvwrapper.sh" >> ~/.zshrc # Change to .bashrc if using bash
```

4. If you are on Ubuntu 20.04, install python3.6-dev
```bash
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt-get update
sudo apt-get install python3.6-dev
sudo apt-get install python3.6-distutils
```

Note: You might need to restart your terminal at this point.

5. Create virtual environment with python 3.6
```bash
source ~/.zshrc # Change to .bashrc if using bash
mkvirtualenv --python=python3.6 cadrl
```

6. Install packages inside your venv (venv always activated!):

```bash
pip install --extra-index-url https://rospypi.github.io/simple/ rospy rosbag tf tf2_ros --ignore-installed
pip install pyyaml catkin_pkg gym netifaces pathlib filelock pyqt5 mpi4py lxml scipy defusedxml matplotlib tensorflow==1.4
```

## Usage
Remember to activate the python3.6 virtual environment, before launching arena-bench.  
If you followed the installation guide, you can do that by typing:
```bash
workon cadrl
```
## This planner can be chosen using the local_planner argument like so:
```bash
roslaunch arena_bringup start_arena.launch local_planner:=cadrl # Make sure that your virtual env/poetry is activated
```
## For more details regarding usage, please refer to our [documentation](https://arena-rosnav.readthedocs.io/en/latest/user_guides/usage/)
## Original work
For more information about CADRL, please refer to the original publication [paper](https://arxiv.org/abs/1805.01956)
