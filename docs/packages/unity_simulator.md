
# Unity Simulator

The Unity Simulator is currently being integrated using the [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub/tree/main).  
During development it's important to have Unity (2022.3.11f1) set up and open the [seperate Repository](https://github.com/Arena-Rosnav/arena-unity). The current implementation is on [this branch](https://github.com/TheZomb/arena-rosnav/tree/unity-simulator). 

### Starting
To start the process you first need to start Arena Rosnav with 

```bash
roslaunch arena_bringup start_arena.launch simulator:=unity
```

Right after that you need to start the Unity Simulator in the Editor.  
If everythin is alright you should be able to see the following in the Game Preview:  

![](../images/packages/Unity-Connection-Success.png)