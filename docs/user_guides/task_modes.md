# Task Modes

These are the different task modes for the robots, obstacles and modules. Information about the task mode can be found by clicking on the desired task mode in the list below.

## Robot and Obstacle Task Modes

Robots and obstacles can be assigned the following task modes:
- [Scenario](task_modes/scenario.md)
- [Random](task_modes/random.md)

Only robots:
- [Guided](task_modes/guided.md)
- [Explore](task_modes/explore.md)

Only obstacles:
- [Parametrized](task_modes/parametrized.md)

You can dynamically change the task modes using `rosparam set /tm_<obstacles|robots>=...`

## Module Task Modes

There are four module task modes:

- [Clear Forbidden Zones](task_modes/clear_forbidden_zones.md) (Always loaded)
- [Dynamic Map](task_modes/dynamic_map.md)
- [Rviz UI](task_modes/rviz_ui.md) (Always loaded)
- [Staged](task_modes/staged.md)

