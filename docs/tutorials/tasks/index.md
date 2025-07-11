# Task Modes

These are the different task modes for the robots, obstacles and modules. Information about the task mode can be found by clicking on the desired task mode in the list below.

## Robot and Obstacle Task Modes

Robot and obstacle task modes are completely independent of each other, they are selected using the `tm_<obstacles|robots>` launch args.

You can dynamically change the task modes at run-time by setting the parameters `/task_generator_node/tm_<obstacles|robots>`. The new task modes are activated upon the next reset.

The available task modes are:

| Task Mode | Short Description | Robots | Obstacles |
| --- | --- | --- | --- |
| [`scenario`](scenario.md) | load scenario file | ✓ | ✓ |
| [`random`](random.md) | generate random positions | ✓ | ✓ |
| [`parametrized`](parametrized.md) | more fine-tuned random | | ✓ |
| [`explore`](explore.md) | explore map | ✓ | |
| [`environment`](environment.md) | fill environment | ✓ | ✓ |
<!-- | [`zones`](zones.md) | generate positions in zones | ✓ | ✓ | -->


## Task Modules

There available task modules are

- [Benchmark](benchmark.md).

They are activated by providing a comma-delimited list as a value to the `tm_modules:=`launch argument. Example

```sh
ros2 launch arena_bringup arena.launch.py tm_modules:=benchmark
```

## Add a new task mode

Here is a guide for [adding new task modes](adding_task_modes.md).
