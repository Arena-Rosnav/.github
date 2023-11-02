
# Gazebo Simulator

The Gazebo simulator shares tasks with the Pedsim manager, including obstacle creation, spawning, and removal. However, it operates within Gazebo, in contrast to Pedsim. 

### Task Synchronization

Both the Pedsim and Gazebo representations of the simulation must run in parallel and remain synchronized. To achieve this, the Gazebo simulator subscribes to ROS topics used by Pedsim, namely "simulated agents" and "simulated waypoints." Callbacks are triggered when data is published to these topics. When the Pedsim manager spawns obstacles, the information is published to these topics, initiating a callback in the Gazebo simulator. The callback reads the topic and uses the data to spawn identical obstacles in Gazebo through the Gazebo's spawn model service.

## Features

- Moving entities in Gazebo.
- Spawning new entities in Gazebo.
- Deleting entities from Gazebo.
- Handling simulation pause and resume.


# Create an instance of GazeboSimulator
simulator = GazeboSimulator(namespace)

# Example: Move an entity in Gazebo
simulator.move_entity(name, pos)

# Example: Spawn a new entity in Gazebo
entity = ...
simulator.spawn_entity(entity)

# Example: Delete an entity from Gazebo
simulator.delete_entity(name)


Requirements
ROS (Robot Operating System)
Gazebo simulator
Python 3.x
