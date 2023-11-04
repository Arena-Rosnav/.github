
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

# Move an Entity in Gazebo

-name = "your_entity_name"  # Replace with the name of the entity
-pos = (x, y, angle)  # Replace with the desired target position and orientation
- The `simulator.move_entity(name, pos)` function is called to move the specified entity to the target position.
- This function internally communicates with Gazebo's services to update the entity's position and orientation.

# Spawn a New Entity in Gazebo

-entity = Entity()
-entity.name = "new_entity"  # Replace with the desired entity name
-entity.position = (x, y, angle)  # Replace with the initial position and orientation
-entity.model = Model()  # Create a Model object and configure it as needed, defining the type of model (URDF or SDF) and the model description
- The `simulator.spawn_entity(entity)` function is called to spawn the new entity in Gazebo.
- This function internally communicates with Gazebo's services to add the entity to the simulation environment. The function returns `True` if the entity was successfully spawned.

# Delete an Entity from Gazebo

- name = "entity_to_delete"  # Replace with the name of the entity to be deleted
- The `simulator.delete_entity(name)` function is called to delete the specified entity from the Gazebo simulation environment.
- This function internally communicates with Gazebo's services to remove the entity.
- The function returns `True` if the entity was successfully deleted from the simulation.

Requirements
ROS (Robot Operating System)
Gazebo simulator
Python 3.x
