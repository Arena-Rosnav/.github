# Launch Arena-Rosnav Simulation
Arena-Rosnav works with ROS as its core. The [```roslaunch``` command](http://wiki.ros.org/roslaunch) is a tool to start ROS packages and utilizes ```.launch``` files to bring up multiple nodes at the same time. Our project stores the launch files in the [```arena-bringup``` directory/package](https://github.com/Arena-Rosnav/arena-rosnav/tree/dev/arena_bringup). For deployment in 2D use the ```start_arena_flatland.launch``` file. The simulation establishes and environment in which the robot is spawned and has to solve a navigation task. The properties of the task and the environment are specified in the launch file and launch command.

The quickest way to launch a simulation is via this command:
```
roslaunch arena_bringup start_arena_flatland.launch
```

This way the simulation will be launched with default settings. Additional configurations for the simulation can be set via options (also multiple at the same time). The options should be provided in this manner:

```
roslaunch arena_bringup start_arena_flatland.launch <option_key>:=<option_value> ..
```

Here is a list of the most common options:

- `local_planner` - local planning algorithm, all available local planners can be found in ```../catkin_ws/src/planners/```, provide to the directory name of the planner you want to deploy
- `map_file` - map to load in the environment, all availabe maps can be found in `../catkin_ws/src/forks/arena-simulation-setup/maps/`, provide the directory name of the map you want to use
- `task_mode` - type of task for the algorithm to solve, {`random`,`manual`,`staged`,`scenario`}, please refer to the [Task Generator](../overview/general_utils.md#task-generator) page for further information
- `scenario_file` - scenario file defining a `scenario` task, all available scenarios can be found in `../catkin_ws/src/forks/task_generator/scenarios/`, provide the `.json` filename to use the scenario
- `use_recorder` - default is `false`, option to record measurements during simulation

Example:
!!! warning
    TODO

!!! note
    The simulation will terminate itself after finishing all tasks.