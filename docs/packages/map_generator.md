# Map Generator

The Map Generator is a Ros Package to generate dynamic maps for the [`dynamic_map` Task Mode](/user_guides/task_modes/dynamic_map/).

## System Design

![](/docs/images/system-design/map_generator-sys.png)

## Requesting a Map
 
1. The `Task Generator` send a signal via the `/request_new_map` topic
2. The `MapGeneratorNode` generates a new map and sends out the map data via two topics
   - `/map` contains an array describing the grid and position of walls. This data is then sent to `MapDistanceNode`
   - `/map_obstacles` describes the obstacles of the map. This data is then sent to `WorldManager` and stored until the distance map is ready
3. The `MapDistanceNode` computes a new distance map and sends the data to the `WorldManager` to store
4. Via `/signal_new_distance_map` a signal is sent that the new map is ready

## Topics

| Publisher        | on topic                   | Subscriber                    |
| ---------------- | -------------------------- | ----------------------------- |
| Task Generator   | "/request_new_map"         | MapGeneratorNode              |
| MapGeneratorNode | "/map"                     | MapDistanceNode               |
| MapDistanceNode  | "/signal_new_distance_map" | Task Generator                |
| Task Generator   | "/dynamic_map/task_reset"  | Task Generator                |
| MapGeneratorNode | "/map_obstacles"           | Task Generator > WorldManager |
