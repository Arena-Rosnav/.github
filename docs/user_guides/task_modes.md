# Task Modes

These are the different task modes for the robots, obstacles and modules. Information about the task mode can be found by clicking on the desired task mode in the list below.

## Robot and Obstacle Task Modes

Robot and obstacle task modes are completely independent of each, they are selected using the `tm_<obstacles|robots>` launch args, e.g.

```sh
roslaunch arena_bringup start_arena.launch tm_obstacles:=random tm_robots:=scenario
```

You can dynamically change the task modes at run-time using `rosparam set /tm_<obstacles|robots>=...`. The new task modes are activated upon the next reset.

The available task modes are:

| Task Mode | Short Description | Robots | Obstacles |
| --- | --- | --- | --- |
| [`scenario`](task_modes/scenario.md) | load scenario file | ✓ | ✓ |
| [`random`](task_modes/random.md) | generate random positions | ✓ | ✓ |
| [`parametrized`](task_modes/parametrized.md) | more fine-tuned random | | ✓ |
| [`guided`](task_modes/guided.md) | waypoint sequence | ✓ | |
| [`explore`](task_modes/explore.md) | explore map | ✓ | |


## Task Modules

There available task modules are

- [Clear Forbidden Zones](task_modes/clear_forbidden_zones.md) (Always loaded),
- [RViz UI](task_modes/rviz_ui.md) (Always loaded),
- [Dynamic Map](task_modes/dynamic_map.md),
- [Staged](task_modes/staged.md),
- [Benchmark](task_modes/benchmark.md).

They are activated by providing a comma-delimited list as a value to the `tm_modules:=` roslaunch argument. Example:

```sh
roslaunch arena_bringup start_arena.launch tm_modules:=dynamic_map,staged
```