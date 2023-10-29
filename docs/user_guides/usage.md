# Usage

## Evaluation

To start a simulation just launch the `start_arena.launch` file in the _arena_bringup_ package.

```bash
roslaunch arena_bringup start_arena.launch
```
e.g.
```bash
roslaunch arena_bringup start_arena.launch simulator:=gazebo task_mode:=scenario model:=jackal map_file:=map_empty pedsim_waypoint_plugin:=spinny
```

The launch file takes following parameters:

| Name             | Default                 | Type             | Description                                                                                                                                                                                                         |
| ---------------- | ----------------------- | ---------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| model            | burger                  | ---              | Name of the robot you want to simulate. currently [burger, jackal, ridgeback, agvota, rto, cob3, youbot, dingo, waffle, tiago,... ]                                                                                                                                                                             |
| local_planner    | teb                     |                  | Name of the planner you want to use. currently available: [teb, dwa, mpc, cadrl, crowdnav, rlca, aio, applr, cohan, dragon, lflh, sarl, trail, arena (ours), rosnav(ours)]                                                                                                                                                                                |
| simulator      | flatland                | flatland, gazebo | Name of the simulator                                                                                                                                                                                            |
| robot_setup_file |                         | string           | Name of the robot setup file you<br /> want to use. The file should be<br /> located in /task-generator/robot_setup/.<br /> This is only used if you want to <br /> start a simulation with multiple <br /> robots. |
| pedsim_waypoint_plugin      | pedsim                   | bool             | name of your plugin <br /> to manipulate pedsim movements                                        |
| agent_name       | value of model argument | string           |                                                                                                                                                                                                                     |
| task_mode        | random                  | random, scenario,<br /> staged, parametrized,<br /> dynamic_map, random_scenario, random_staged |                                       If scenario: specify scenario path, if staged: specify stages config path                                                                                                                                                                              |
| visualization    | rviz                    | flatland, rviz   | wether rviz or flatland should be used<br /> for visualization. Using flatland is<br /> only possible when flatland<br /> is selected as environment                                                                |
| show_rviz        | true                    | bool             | Activates rviz when using gazebo                                                                                                                                                                                    |
| scenario_file    | test_scenario.json      | string           | Name of the scenario file.<br /> Must be located in<br /> /task-generator/scenarios                                                                                                                                 |
| map_file         | map0                    | string           | Name of the map. The map must <br />be located in<br /> /arena_simulation_setup/maps                                                                                                                                |
| record_data      | false                   | bool             | Wether you want to record the<br /> robot data during the run for <br />later evaluations with the <br />[arena_evaluation](../packages/arena_evaluation.md) package.                                               |

## Training

To start a training procedure you need two terminals.

In terminal 1:

```
roslaunch arena_bringup start_training.launch
```

In terminal 2:

```
cd arena-rosnav # navigate to the arena-rosnav directory
python training/scripts/train_agent.py
```

Remember to active the poetry shell in both terminals.

You can find more informations about the trainings process [here](training.md).
