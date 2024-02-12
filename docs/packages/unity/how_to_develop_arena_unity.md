## How to Develop and Extend Arena Unity
Only running the Unity Simulator without making changes to it is easier since this only requires the correct arena-rosnav branch and the executable build of Arena Unity. But to develop it you need the correct Unity Editor version. For this you will also need to create a Unity account in order to have a Unity license.

**Getting Unity Hub and a Unity license**:  
After doing the steps in How to use Arena Unity, you need to install Unity Hub by following the steps in [this manual](https://docs.unity3d.com/hub/manual/InstallHub.html#install-hub-linux) but do not install a Unity Editor yet. You will also need to create a Unity  Account, if you don't already have one, for the Unity license.

**Installing the correct Unity Editor**:  
For this project, we use the Unity Editor version 2022.3.11f1. To install this version, execute the install-unity-version.sh script  in *\[catkin_ws\]/src/arena-unity* :
```sh
./install-unity-version.sh
```

Now, you can start developing, by opening the project path *\[catkin_ws\]/src/arena-unity* in the Unity Hub and in your favourite code editor.  
For development it's easier to test changes in Arena Unity by starting the Unity Simulator fromt the Unity Editor (i.e. simply pressing *play* in the Unity Editor, without having to build it first). If you want to start Arena Unity from within the Editor you can use the argument *development_mode* for the launch command:
```sh
roslaunch arena_bringup start_arena.launch simulator:=unity task_mode:=scenario model:=jackal development_mode:=true 
```
This will stop Arena Rosnav from running the executable build of Arena Unity but instead waits for you to start it yourself.  
You then start Arena Rosnav with the *roslaunch* command and start Arena Unity by pressing *play* in the Unity Editor (the order doesn't matter). It is working if you see a blue ROS connection icon in the left upper corner of the Game Preview:

![](../../images/packages/Unity-Connection-Success.png)
