
# Gazebo Simulator

The Gazebo simulator shares tasks with the Pedsim manager, including obstacle creation, spawning, and removal. However, it operates within Gazebo, in contrast to Pedsim. 

### Task Synchronization

Both the Pedsim and Gazebo representations of the simulation must run in parallel and remain synchronized. To achieve this, the Gazebo simulator subscribes to ROS topics used by Pedsim, namely "simulated agents" and "simulated waypoints." Callbacks are triggered when data is published to these topics. When the Pedsim manager spawns obstacles, the information is published to these topics, initiating a callback in the Gazebo simulator. The callback reads the topic and uses the data to spawn identical obstacles in Gazebo through the Gazebo's spawn model service.
