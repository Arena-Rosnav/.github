# Arena 3.0 Playground

## Installation

Follow the automatic installation in [the documentation](https://arena-rosnav.readthedocs.io/en/latest/user_guides/installation/).

## First Steps

Try out your first simulation by running

```sh
roslaunch arena_bringup start_arena.launch
```

Answer the following questions:
1. Which simulator is running?
2. Which robot was spawned?
3. How are the obstacle locations determined?

<details>
    <summary>Solution</summary>
    
    1. flatland
    2. burger
    3. randomly
</details>

## Launch Arguments

Learn about the available launch arguments by reviewing [the relevant documentation](https://arena-rosnav.readthedocs.io/en/latest/user_guides/usage/).

Launch a simulation with the following properties:
- the simulator is `flatland`,
- the robot is `jackal` is spawned in `map_empty`,
- the local planner is `dwa`,
- the robot task mode and obstacle task mode are both `scenario`.

<details>
    <summary>Solution</summary>
    
    ```sh
    roslaunch arena_bringup start_arena.launch model:=jackal local_planner:=dwa map_file:=map_empty tm_robots:=scenario tm_obstacles:=scenario
    ```
</details>

## Manipulating the Simulation

### RViz

The most direct way to manipulate the simulation is by using [the `RViz` UI](https://arena-rosnav.readthedocs.io/en/latest/user_guides/task_modes/rviz_ui/). Utilize all three tools and try to achieve the following:
- Make the robot not reach its goal for a whole 60 seconds.

### rosparam

Some launch arguments can be dynamically updated at run time without having to start the simulation. Read [the task mode documenation](https://arena-rosnav.readthedocs.io/en/latest/user_guides/task_modes/#robot-and-obstacle-task-modes) carefully and dynamically set `tm_obstacles` to `random`.

<details>
    <summary>Solution</summary>
    
    ```sh
    rosparam set tm_obstacles random
    ```
</details>

Verify that the obstacles are indeed spawned randomly upon each subsequent reset.

### task_generator.yaml

All essential simulation parameters are specified in a central configuration file `task_generator.yaml`.
Locate this file in `arena_bringup/configs/task_generator.yaml` and open it for editing.

_Helpful tip: in order to directly get the exact location of `arena_bringup`, run `rospack find arena_bringup`_

#### Global Configuration

Within the configuration you will find the parameter `timeout` which specifies the maximum time in seconds that the robot can take to reach the goal. Try lowering this limit so that the robot doesn't reach its goal in time.

#### Task Configuration
Since we are running `tm_robots:=scenario` and `tm_obstacles:=random` at this point, the subsections `SCENARIO` and `RANDOM` are relevant to our simulation. We can manipulate their parameters at run time.

Change the scenario by setting `SCENARIO/file` to an applicable scenario you found in `arena_bringup/configs/scenarios`.

<details>
    <summary>Solution</summary>
    
    Any of `map_empty_X.json` are fine.
</details>

Change the number of obstacles spawned by setting `RANDOM/<type>/<min|max>` to higher or lower values. Go crazy with this one.


## Starting a new Simulation

Some simulation properties cannot be changed at runtime and require a full restart of the simulation. Abort the simulation by sending a keyboard interrupt to the console (`Ctrl+C`) and wait for everything to stop.

Change the scenario to `arena_hospital_scmall_1.json` in `task_generator.yaml`.

Run a new simulation with the updated properties:
- the simulator is `gazebo`,
- the robot `burger` is spawned on map `arena_hospital_small`,
- the local planner is `teb`,
- the robot task mode and obstacle task mode are both `scenario`.

<details>
    <summary>Solution</summary>
    
    ```sh
    roslaunch arena_bringup start_arena.launch model:=burger local_planner:=teb map_file:=arena_hospital_small tm_robots:=scenario tm_obstacles:=scenario
    ```
</details>

Wait for Gazebo to fully open (the first model indexing might take a while) and verify that both a Gazebo and an RViz window have opened.
All the functionalities taught in the previous section still apply to the current simulation. Apply them to the current simulation and furthermore familiarize yourself with the [Gazebo controls](https://classic.gazebosim.org/tutorials?cat=guided_b&tut=guided_b2).

### Headless

While using the Gazebo simulator, you might not necessarily want the added overhead of opening the Gazebo UI. Try starting a semi-headless simulation by adding the launch argument `headless:=1`.

## Benchmark

A special mode is provided by the module `benchmark`. It is used to automatically start a series of simulations and record data along the way for the purpose of benchmarking planners against each other.

### Configuration

Navigate to `arena_bringup/configs/benchmark` and open the file `config.yaml` for editing. A benchmark consists of a unique combination of
1. contest - a list of planners (_contestants_) competing against each other
2. suite - a list of challenges (_stages_) that each contestant is subjected to

Select a contest and a suite from the respective subdirectories and write them to the config.

### Launch

Launch the benchmark by running
```sh
arena_bringup start_arena.launch tm_modules:=benchmark
```

The rest is handled fully automatically. Simulators will open and close multiple times, this can lead to many error messages in the console. This is normal and no cause for concern. If you do not need the simulator UIs at all, add the launch argument `headless:=2` to your future benchmark launches.

You may notice that the `task_generator.yaml` is changed - a backup can be found in `task_generator.yaml.bkup`. Do not touch either file during the benchmark. Once the benchmark finishes, the initial state will automatically be restored.

Follow the progress of the benchmark by reading the relevant log file in the subdirectory `logs`. Wait for the entire benchmark to finish.

### Postprocessing

The raw result of the benchmark, identified by its `run_id` from the log file, is located in `arena_evaluation/data/<run_id>` (find using `rospack find arena_evaluation`). Further postprocessing of the data is provided by the `arena_evaluation` package and is covered in its own tutorial.