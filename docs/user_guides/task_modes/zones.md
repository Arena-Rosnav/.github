### Zones

The zones task mode will create waypoints for pedestrians and robots in specific zones. The zones are defined in a separate file with the following parameters for each zone:

| Key             | Type          | Description                                                                     |
|-----------------|---------------|---------------------------------------------------------------------------------|
| label           | String        | Name for the zone                                                               |
| category        | [String]      | List of categories of the zone                                                  |
| polygon         | [Polygon]     | List of polygons (each polygon is a list of points [[float, float]])            |

The `.yaml` file should be located in the map directory of the respective world. Zone files can be easily created using the Zones Editor in `arena-tools`.

Additionally a zone scenario file is required for the usage of this task mode. The zone scenario file describes the different pedestrians and robot roles.

The zone and zone scenario files used for the zone task mode can be specified in `task_mode/zones` parameter in `src/arena-rosnav/arena_bringup/configs/task_generator.yaml`.

#### Pedestrians

The zones task spawns a variety of pedestrians upon each reset, specified within the zone scenario file. Each type of pedestrian has these attributes:

| Key             | Type          | Description                                                                     |
|-----------------|---------------|---------------------------------------------------------------------------------|
| name            | String        | Name for the pedestrians                                                        |
| type            | String        | Type of pedestrian e.g. human/adult                                             |
| model           | String        | Model used for pedestrian                                                       |
| waypoint in     | [String]      | List of categories in which the waypoints should spawn                          |
| number of waypoints| [int. int] | min, max amount of waypoints to be created                                      |
| amount          | int           | Amount of pedestrians                                                           |


#### Robots

The robot will be randomly assigned a role. The roles are also defined in the zone scenario file:

| Key             | Type          | Description                                                                     |
|-----------------|---------------|---------------------------------------------------------------------------------|
| name            | String        | Name for the role                                                               |
| waypoint in     | [String]      | List of categories in which the waypoints should spawn                          |

The robots behave like the [Explore](explore.md) task mode but their waypoints only spawn in specific zones, which has the specified categories.

#### Usage

```sh
roslaunch arena_bringup start_arena.launch tm_obstacles:=zones tm_robots:=zones
```
