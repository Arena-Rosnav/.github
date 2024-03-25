# Dynamic Map

This task mode creates a dynamic map. To use this mode add the argument `map_file` with value `dynamic_map`.

One very basic example to run the Task Mode:
```sh
roslaunch arena_bringup start_arena.launch map_file:=dynamic_map
```

![Dynamic Map in RVIZ](./gifs/dynamic_map.gif)

## Configuration

The Task Mode can be configured via editing `arena-rosnav/arena_bringup/params/map_generator.yaml`. The most relevant configurations include:
- `algorithm`: Choose between different kinds of algorithms. A list of possible options can be seen [here](/docs/packages/map_generator.md)
- `map_type`: Choose between different kinds of map generation types of the choposen algorithm. A list of possible options can be seen [here](/docs/packages/map_generator.md)
- `width`/`height`: sets the general size of the map
- `resolution`: the map is defined by a grid. This parameter sets the grid size

There are also some more parameters, which are specific to each map type. Please refer to their respective [documentation](/docs/packages/map_generator.md).

## Example
An example configuration looks as follows


```yaml
algorithm: rosmap
episode_per_map: 1

map_properties:
  width: 40
  height: 40
  resolution: 0.5

algorithm_config:
  map_type: canteen
  obstacle_num: 15
  obstacle_extra_radius: 5

  rosmap:
    canteen:
      chair_chance: 0.6
```