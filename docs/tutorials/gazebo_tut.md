## Gazebo as Simulator in Deployment Stage


Gazebo is a 3D simulation tool that enables the creation of realistic scenarios. Due to its popularity, there are already many custom robots and worlds available. We offer a variety of robots, such as Jackal, TurtleBot3, Robotino, and more. Additionally, we provide different environments, including hospitals, warehouses, airport terminals, etc. To simulate the movement of humans, we use PedSim as dynamic obstacles.
More information can be found in our [Arena-Rosnav](https://arena-rosnav.readthedocs.io/en/latest/) documentation. 

**Generic Call**

<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/clipboard.js/2.0.8/clipboard.min.css">

```python
roslaunch arena_bringup start_arena.launch [task_mode] [simulator] [desired_resets] [map_file] [local_planner] [model] [more_flags]
```

| Programm Call                              | Flags           | Description                                                                        |
|----------                                  |----------       |----------                                                                          |
| roslaunch arena_bringup start_arena.launch | task_mode       | random, staged, scenario, parametrized, dynamic_map_random, dynamic_map_staged     |
|                                            | simulator       | gazebo                                                                             |
|                                            | desired_resets  | 20                                                                                 |
|                                            | map_file        | hospital, small_warehouse, aws_house,                                              |
|                                            | local_planner   | dwa, teb, mpc                                                                      |
|                                            | model           | jackal, burger                                                                     |

**Task Generator Config File**

We create a task generator configuration file to modify various task mode settings.   
This file can be found in the following directory:

```python
/arena-rosnav/arena_bringup/configs/task_generator.yaml
```

| Task Mode   | Settings        | Description      |
|----------   |----------       |----------        |
| random      | static, interactive, dynamic       | The random task mode creates random static and dynamic obstacles in the world. In the static settings, you can set the maximum and minimum numbers of randomly spawned static objects. The task generator will then create a random number between the minimum and maximum value. In the dynamic settings, you can set maximum and minimum numbers of randomly spawned dynamic agents. The task generator will then create a random number between the minimum and maximum value.|
| scenario    | scenario_file                      | The random task mode is designed for evaluation in the first place. In the scenario file dynamic and static obstacles as well as the start and goal position of the robot are defined.                                                                              |
| staged      | curriculum, starting_index         | The staged task mode is designed for training processes. In general, it behaves like the random task mode. You can set the number of randomly spawned dynamic and static objects. The difference from the random task mode is that there are multiple stages. In each stage, a different number of dynamic and static objects can be set. If the robot successfully completes the first stage, the next stage will be triggered. The amount of obstacles is defined in a curriculum file |



### Aws Hospital World

| Programm Call                                    | Flags           | Description |
|----------                                        |----------       |----------|
| roslaunch arena_bringup start_arena.launch       | task_mode       | random   |
|                                                  | simulator       | gazebo   |
|                                                  | desired_resets  | 20       |
|                                                  | map_file        | hospital |
|                                                  | local_planner   | teb      |
|                                                  | model           | jackal   |

| Task Mode | Settings |
|-----------|----------|
| random    | static:       <br> min: 0 <br> max: 0           |
|           | interactive:  <br> min: 0 <br> max: 0           |
|           | dynamic:      <br> min: 10 <br> max: 20         |


<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/clipboard.js/2.0.8/clipboard.min.css">

```python
roslaunch arena_bringup start_arena.launch task_mode:=random simulator:=gazebo desired_reset:=20 map_file:=hospital local_planner:=teb model:=jackal
```

### AWS Small Warehouse World

| Programm Call                              | Flags           | Description |
|----------                                  |----------       |----------|
| roslaunch arena_bringup start_arena.launch | task_mode       | random           |
|                                            | simulator       | gazebo           |
|                                            | desired_resets  | 20               |
|                                            | map_file        | small_warehouse  |
|                                            | local_planner   | teb              |
|                                            | model           | jackal           |

| Task Mode | Settings |
|-----------|----------|
| random    | static:       <br> min: 0 <br> max: 0           |
|           | interactive:  <br> min: 0 <br> max: 0           |
|           | dynamic:      <br> min: 10 <br> max: 20         |

<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/clipboard.js/2.0.8/clipboard.min.css">

```python
roslaunch arena_bringup start_arena.launch task_mode:=random simulator:=gazebo desired_reset:=20 map_file:=small_warehouse local_planner:=teb model:=jackal
```


### AWS Small House World

| Programm Call                              | Flags           | Description |
|----------                                  |----------       |----------|
| roslaunch arena_bringup start_arena.launch | task_mode       | random   |
|                                            | simulator       | gazebo   |
|                                            | desired_resets  | 20       |
|                                            | map_file        | hospital |
|                                            | local_planner   | teb      |
|                                            | model           | jackal   |
| Task Mode | Settings |
|-----------|----------|
| random    | static:       <br> min: 0 <br> max: 0           |
|           | interactive:  <br> min: 0 <br> max: 0           |
|           | dynamic:      <br> min: 10 <br> max: 20         |

<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/clipboard.js/2.0.8/clipboard.min.css">

```python
roslaunch arena_bringup start_arena.launch task_mode:=random simulator:=gazebo desired_reset:=20 map_file:=aws_house local_planner:=teb model:=jackal
```



### Factory World

### RMF Airport Terminal


