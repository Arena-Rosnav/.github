### Scenario

The scenario task will spawn the static and dynamic obstacles defined in the scenario file. All scenario files are in `src/arena-rosnav/arena_bringup/configs/scenarios`.

A scenario consists of 
```json
{
  "obstacles": {
    "dynamic": ["<PEDSIM AGENT DESCRIPTION>"],
    "static": [
      {
        "name": "<object name>",
        "pos": ["X", "Y", "theta"],
        "model": "<model name>"
      }
    ]
  },
  "robots": [
    {
      "start": ["X", "Y", "theta"],
      "goal": ["X", "Y", "theta"]
    }
  ],
  "map": "<NAME OF THE MAP>",
  "resets": "<AMOUNT OF RESETS>"
}
```

To use the scenario file set the parameter `task_mode/scenario/scenario_file` in `src/arena-rosnav/arena_bringup/configs/task_generator.yaml` and then enter in the terminal:

```bash
roslaunch arena_bringup start_arena.launch model:=jackal tm_robots:=scenario tm_obstacles:=scenario
```

If the robot reached its goal defined in the scenario file the map will be resetted. 

This is the Scenario task mode with the scenario file `default.json`:

![Scenario{}](./gifs/scenario.gif) 