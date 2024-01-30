## Create New Scenarios

In the scenario file dynamic and static obstacles as well as the start and goal position of the robot are defined. You can find more informations in the **User Guides (Task Modes)** section.
<br>
<br>
Create a new scenario file (`.json`) in the following directory:
<br>
`../arena-rosnav/arena_bringup/configs/scenario`

This is the general structure of the scenario .json file (default.json):

```json
{
  "robots": [
    {
      "start": [24.0, 20.0, 0.7],
      "goal": [2.0, 2.0, 0.0]
    }
  ],
  "obstacles": {
    "static": [
      {
        "name": "shelf1",
        "model": "shelf",
        "pos": [13.0, 11.0, -0.5]
      },
      {
        "name": "shelf2",
        "model": "shelf",
        "pos": [12.0, 12.0, -0.5]
      },
      {
        "name": "shelf3",
        "model": "shelf",
        "pos": [14.0, 10.0, -0.5]
      }
    ],

    "dynamic": [
      {
        "name": "1",
        "pos": [14.0, 2.0, 0.0],
        "type": "adult",
        "model": "actor1",
        "waypoints": [
            [2.0, 14.0, 0.0],
            [14.0, 2.0, 0.0]
        ],
        "waypoint_mode": 1
      },
      {
        "name": "2",
        "pos": [2.0, 8.0, 0.0],
        "type": "adult",
        "model": "actor1",
        "waypoints": [
            [8.0, 2.0, 0.0],
            [8.0, 2.0, 0.0]
        ],
        "waypoint_mode": 1
      },
      {
        "name": "3",
        "pos": [8.0, 2.0, 0.0],
        "type": "adult",
        "model": "actor1",
        "waypoints": [
          [2.0, 8.0, 0.0],
          [8.0, 2.0, 0.0]
        ],
        "waypoint_mode": 1
      }
    ],
    "interactive": []
  },
  "map": "map_empty"
}

```

In the *robots* section, you can choose the start `"start": [24.0, 20.0, 0.7]` and goal `"goal": [2.0, 2.0, 0.0]` position of the robot. The first entry is the x position, the second is the y position and the third entry is the yaw orientation. 
<br>
<br>
In the *obstacles/static* section, you can define different static obstacles such as shelves, tables etc. Additionally you can provide different names for each static obstacle. 
<br>
<br>
In the *obstacles/dynamic* section, you can create dynamic pedestrians, such as women, men, and/or children. The `"pos": [14.0, 2.0, 0.0]` argument provides the start position of the robot. The `"waypoints": [2.0, 14.0, 0.0], [14.0, 2.0, 0.0]` argument provides different waypoints for the pedestrians. Always include the start position as a waypoint. Either the pedestrians will follow the waypoints in the given order or they will do it randomly or in a cyclic manner. It is not necessary to provide many waypoints because movements such as turning in aisles will be determined by the selected social force model. 
Therefore, you can add the waypoints in a random order. Additionally you can provide different names for each pedestrian. 
<br>
<br>
Note: The number of episodes/resets of the scenario can be define in the `task_generator.yaml` file.


**Testing**

Command:
```python
roslaunch arena_bringup start_arena.launch map_file:=map_empty model:=jackal simulator:=gazebo tm_robots:=scenario tm_obstacles:=scenario
```

After the robot reaches the goal, the scenario will reset. 

<br>
<br>
![]() 



