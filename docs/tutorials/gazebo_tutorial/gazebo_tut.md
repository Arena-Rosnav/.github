## Deploy A Robot In Different Worlds Using Gazebo As Simulator


Gazebo is an open-source 3D simulation environment that enables the creation of realistic scenarios, including simulating sensors, robot dynamics, and the interaction with objects in a virtual environment. Due to its popularity in the field of robotics, there are already many custom worlds available. For simulation, we provide different environments, including hospitals, warehouses, airport terminals, etc. Additionally , we offer a variety of robots, such as Jackal, TurtleBot3, Robotino, and more. To simulate moving humans, we use PedSim as dynamic obstacles.
More information can be found in our User Guides section. 

**Generic Call**

<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/clipboard.js/2.0.8/clipboard.min.css">

```python
roslaunch arena_bringup start_arena.launch [task_mode] [simulator] [desired_resets] [map_file] [local_planner] [model] [more_flags]
```

| Programm Call                              | Flags           | Setting Options                                                                         | Description |
|----------                                  |----------       |----------                                                                               | ----------  |
| roslaunch arena_bringup start_arena.launch | task_mode       | random, , scenario, staged, <br> (parametrized, dynamic_map_random, dynamic_map_staged)   | Random task ist designed for deployment. Scenario task is designed for evaluation. Staged task is designed for training.            |
|                                            | simulator       | gazebo, flatland                                                                        | Flatland a performance centric 2D robot simulator or Gazebo a 3D simulation environment.             |
|                                            | desired_resets  | 10                                                                                      | Number of desire resets             |
|                                            | map_file        | hospital, small_warehouse, aws_house,                                                   | World to use in Gazebo              |
|                                            | local_planner   | dwa, teb, mpc                                                                           | Local Planner                       |
|                                            | model           | jackal, burger                                                                          | Robot model to deploy in the gazebo world.             |

**Task Generator Config File**

We create a task generator configuration file to modify various task mode settings.   
This file can be found in the following directory:

```python
/arena-rosnav/arena_bringup/configs/task_generator.yaml
```

| Task Mode in Task Generator Config File   | Settings        | Description      |
|----------   |----------       |----------        |
| random      | static, interactive, dynamic       | The random task mode creates static and dynamic obstacles randomly in the world. In the static settings, inside the task generator config file, you can set the maximum and minimum numbers of randomly spawned static objects. The task generator will then create a random number between the minimum and maximum value. In the dynamic settings, you can set maximum and minimum numbers of randomly spawned dynamic agents. The task generator will then create a random number between the minimum and maximum value. In the random task mode, the robot is assigned a random start and goal position. After the robot reaches the goal, a new start and goal pose will be assigned and new static and dynamic obstacles will be placed randomly in the world. The number of desired resets can be set with the desired_resets flag. |
| scenario    | scenario_file                      | The random task mode is designed for evaluation in the first place. In the scenario file dynamic and static obstacles as well as the start and goal position of the robot are defined.                                                                              |
| staged      | curriculum, starting_index         | The staged task mode is designed for training processes. In general, it behaves like the random task mode. You can set the number of randomly spawned dynamic and static objects. The difference from the random task mode is that there are multiple stages. In each stage, a different number of dynamic and static objects can be set. If the robot successfully completes the first stage, the next stage will be triggered. The amount of obstacles is defined in a curriculum file. |



### Aws Hospital World

| Programm Call                                    | Flags           | Setted   |
|----------                                        |----------       |----------|
| roslaunch arena_bringup start_arena.launch       | task_mode       | random   |
|                                                  | simulator       | gazebo   |
|                                                  | desired_resets  | 20       |
|                                                  | map_file        | hospital |
|                                                  | local_planner   | teb      |
|                                                  | model           | jackal   |

| Task Mode in Task Generator Config File | Settings          |
|-----------|----------                                       |
| random    | static:       <br> min: 0 <br> max: 0           |
|           | interactive:  <br> min: 0 <br> max: 0           |
|           | dynamic:      <br> min: 10 <br> max: 40         |


<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/clipboard.js/2.0.8/clipboard.min.css">

```python
roslaunch arena_bringup start_arena.launch task_mode:=random simulator:=gazebo desired_reset:=20 map_file:=hospital local_planner:=teb model:=jackal
```

![aws_hospital_world](./aws_hospital_world.gif)



### AWS Small Warehouse World

| Programm Call                              | Flags           | Setted           |
|----------                                  |----------       |----------        |
| roslaunch arena_bringup start_arena.launch | task_mode       | random           |
|                                            | simulator       | gazebo           |
|                                            | desired_resets  | 20               |
|                                            | map_file        | small_warehouse  |
|                                            | local_planner   | teb              |
|                                            | model           | jackal           |

| Task Mode in Task Generator Config File | Settings          |
|-----------|----------                                       |
| random    | static:       <br> min: 0 <br> max: 0           |
|           | interactive:  <br> min: 0 <br> max: 0           |
|           | dynamic:      <br> min: 10 <br> max: 40         |

<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/clipboard.js/2.0.8/clipboard.min.css">

```python
roslaunch arena_bringup start_arena.launch task_mode:=random simulator:=gazebo desired_reset:=20 map_file:=small_warehouse local_planner:=teb model:=jackal
```

![aws_small_warehouse_world](./aws_small_warehouse_world.gif)



### AWS Small House World

| Programm Call                              | Flags           | Setted   |
|----------                                  |----------       |----------|
| roslaunch arena_bringup start_arena.launch | task_mode       | random   |
|                                            | simulator       | gazebo   |
|                                            | desired_resets  | 20       |
|                                            | map_file        | hospital |
|                                            | local_planner   | teb      |
|                                            | model           | jackal   |

| Task Mode in Task Generator Config File | Settings          |
|-----------|----------                                       |
| random    | static:       <br> min: 0 <br> max: 0           |
|           | interactive:  <br> min: 0 <br> max: 0           |
|           | dynamic:      <br> min: 10 <br> max: 20         |

<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/clipboard.js/2.0.8/clipboard.min.css">

```python
roslaunch arena_bringup start_arena.launch task_mode:=random simulator:=gazebo desired_reset:=20 map_file:=aws_house local_planner:=teb model:=jackal
```

![aws_small_house_world](./aws_small_house_world.gif)



### Factory World

### RMF Airport Terminal World

### Book Store World

### Baylands Park World

### City World

### Police Station World

### School World