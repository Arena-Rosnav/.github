## How to Use Arena Unity
If you want to use Arena Unity Simulator like Gazebo and Flatland, you need to install the Arena Unity executable.

### Installing Arena Unity

**Getting the Arena Unity Executable**:  
After installing Arena Rosnav via the manual installation or through the automatic installation scripts, execute the Arena Unity install script in *\[catkin_ws\]/src/arena/arena-rosnav* :
```sh
./install4_arena_unity.sh
```

**Running Arena Unity**:  
You can run Arena Unity simply by executing the normal launch command and specifying Unity as the *simulator*. E.g.:
```sh
roslaunch arena_bringup start_arena.launch simulator:=unity 
```
