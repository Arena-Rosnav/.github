# Get Started

## Quickstart Deployment 2D
To launch a simulation, run the following command in your terminal:

```
roslaunch arena_bringup start_arena_flatland.launch
```

!!! note
    The [poetry shell](./error_handling.md#activate-poetry-shell) needs to be activated all the time like a virtual environment.

Without any options provided, the default configuration will be loaded. To add options to the launch command, simply add `<option_key>:=<option_value>`. You can set multiple options at the same time.

```
roslaunch arena_bringup start_arena_flatland.launch <option_key>:=<option_value> ..
```

Here is a list of common options to configure the simulation:

- `local_planner` - local planning algorithm
- `map_file` - map to load in the environment
- `task_mode` - type of task for the algorithm to solve, {`random`,`manual`,`staged`,`scenario`}
- `scenario_file` - scenario file defining a scenario task
- `use_recorder` - default is `false`, option to record measurements during simulation

For further information on the deployment refer to the [Launch a Simulation](./core_functionalities/launch_simulation.md) page.

## Quickstart Training 2D
To start a training procedure you need two terminals.

In terminal 1:
```
roslaunch arena_bringup start_training.launch
```
In terminal 2:
```
cd arena-rosnav # navigate to the arena-rosnav directory
python training/scripts/train_agent.py --agent AGENT_22
```
This will start a single simulation environment with our custom, predefined network architecture "_AGENT_22_". As default robot model, the turtlebot3 model is used.

Here is a list of common options to configure the training:

- ```--agent```: initializes a predefined neural network architecture, the so-called "agent"
- ```--load```: loads an already existing, trained agent
- ```--custom-mlp```: initializes custom MLP according to given parameters

For further information on the training procedure refer to the [Training an Agent](./core_functionalities/training.md) page.