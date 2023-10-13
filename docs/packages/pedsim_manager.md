# Pedsim Manager

The Pedsim manager in the Arena Rosnav infrastructure is responsible for three primary tasks using the Pedsim ROS package and related services:

## Task Overview

1. **Creating Pedsim Obstacles**: This task involves generating Pedsim obstacles. In Random Mode, it uses the map manager to find random unoccupied positions and creates dynamic obstacles with path-defined waypoints. In Scenario Mode, it skips the random position finding step.

2. **Spawning Pedsim Obstacles**: The core function of the Pedsim manager is to spawn obstacles. For Static and Interactive Obstacles, it involves creating service objects and populating them with obstacle data. Dynamic Obstacles also include social states. Borders of the map are spawned to prevent dynamic obstacles from passing through walls.

3. **Removing Pedsim Obstacles**: Obstacle removal is achieved through a Pedsim service call.
