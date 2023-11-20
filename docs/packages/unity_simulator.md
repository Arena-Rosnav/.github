
# Unity Simulator

The Unity Simulator is currently being integrated using the [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub/tree/main).  
During development it's important to have Unity (2022.3.11f1) set up and open the [seperate Repository](https://github.com/Arena-Rosnav/arena-unity) in Unity. The current implementation for Arena-Core is on [this branch](https://github.com/TheZomb/arena-rosnav/tree/unity-simulator). 

## Starting
To start the process you first need to start Arena Rosnav with 

```bash
roslaunch arena_bringup start_arena.launch simulator:=unity model:=burger
```
Right after that you need to start the Unity Simulator in the Editor. (The order isn't really relevant but you need to start both processes shortly after each other).  
If everything is alright you should be able to see the following in the Game Preview:  

![](../images/packages/Unity-Connection-Success.png)

## Structure (Unity)
In this section the structure of the project is described as of 20. Nov 2023.

### ServiceController
This Script should work as the **Endpoint** to our ROS project. To make the process clear, all Services and topics should be created in here.  
Any scene that should be connected to Arena Rosnav should have a component with this Script as a Component.

### ROSClockPublisher
This Script published to `/clock` to synchronize both endpoints.    
Any scene that should be connected to Arena Rosnav should have a component with this Script as a Component.

### Robot
The Robot is instantiated from a prefab in the `ServiceController.cs` script upon calling the `/unity/spawn_model` service for the robot. (*Should be implemented with dynamic loading from URDF string from [arena-simulation-setup](https://github.com/voshch/arena-simulation-setup) instead* ).  
After instantiating at the initial position the following scripts are added:  
- **Drive.cs**: Used to handle `/<robot_namespace>/cmd_vel` commands and handling robot movement accordingly  
- **ROSTransformTreePublisher**: used to publish to `/tf` the transform data of each robots component   
- **LaserScanSensor**: added directly to the Scan component and publishes the data collected from the laser to `/<robot_namespace>/scan`

#### Hint
* Teleporting the robot is not possible. Instead the position needs to be set when Instantiating. One solution for teleporting (when calling `set_model_state`) is to respawn the robot at given position.

## Dev Notes
* Currently the Unity simulator is only implemented for the `burger` robot (20. Nov. 2023)
* The components of the burger robot prefab were renamed (base_scan -> burger/base_scan) for a temporary fix of a ROS publishing destionation error. This should be fixed in the code to dynamically use the robot namespace (20. Nov. 2023)