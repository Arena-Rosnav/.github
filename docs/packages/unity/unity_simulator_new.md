
# Unity Simulator - Arena Unity

Arena Unity, also called Unity Simulator, is our own integration of the Unity Engine into Arena Rosnav. The Unity Simulator has the same core functionalities as Flatland and Gazebo. Among other things, we are able to load maps from Arena Rosnav into a Unity scene where we can spawn pedestrians, obstacles and most importantly the robot itself. This robot in Arena Unity is configured dynamically according to configuration files within Arena Rosnav.  \
This integration was done to train with an RBG-D sensor within the photo-realistic simulation. Next to the photo-realism, it also holds other advantages for training over Gazebo. 

## Tutorials

Get started with these tutorials. If you want to develop Arena Unity, you need to do both tutorials.

- [**How to Use Arena Unity**](how_to_use_arena_unity.md)
- [**How to Develop Arena Unity**](how_to_develop_arena_unity.md)

## Work in progress from here on

## Features

Click on the feature names to get more documentation.

- **Robot Drive**: The Arena Rosnav navigation stack is publishing cmd_vel messages that are applied to the robot in Arena Unity to move the robot.
- **Dynamic Map Loading**: For every map of Arena Rosnav, the distance map file is used to dynamically generate the walls and borders in Arena Unity.
- **Laser Scan and Dynamic Configuration**: The laser scan for each robot is supported and all the parameters (angles, scan rate, resolution, etc.) are configured dynamically.
- **RGB-D Sensor**: **(Not implemented)** 
- **Dynamic Robot Loading**: We use the URDF-Importer of the Unity Robotics Hub to load robots from URDF files but with modifications to the URDF-Importer.
- **Multi-Agent Simulation**: **(Not implemented)** The way the robot sensors generate data and we control the robots, we are able to support mutli-agent simulations.
- **Pedestrian Integration**:  

## System Design

![Arena Unity System Design](../../images/Arena-Unity-Integration.drawio.png)

