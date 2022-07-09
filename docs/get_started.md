# Get Started

## Quickstart Deployment 2D
To launch a simulation run the following command in your terminal inside the arena-rosnav directory:

```
poetry run roslaunch arena_bringup start_arena_flatland.launch
```

Without any options provided, the default configuration will be loaded. To add options to the launch command, simply add `<option_key>:=<option_value>`. You can set multiple options at the same time.

```
poetry run roslaunch arena_bringup start_arena_flatland.launch <option_key>:=<option_value> ..
```

Here is a list of common options to configure the simulation:

- `local_planner` - local planning algorithm
- `map_file` - map to load in the environment
- `task_mode` - type of task for the algorithm to solve, {`random`,`manual`,`staged`,`scenario`}
- `scenario_file` - scenario file defining a scenario task
- `use_recorder` - default is `false`, option to record measurements during simulation
