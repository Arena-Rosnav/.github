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

**Troubleshooting**:  
If you see a pink screen in the Arena Unity window when using the example launch command, this is usually down to either of these reasons:

- Gaphics API
    - Arena Unity requires Vulkan as the Graphics API instead of OpenGL. Vulkan is the default Graphics API when installing Ubuntu. OpenGL is the default Graphics API for Windows. If you run Arena Rosnav in an emulator (e.g. VirtualBox) or WSL, then OpenGL is usually used in the Windows back-end. We are unaware  of a solution for this. We recommend using native Ubuntu.  
    - Make sure Vulkan is available by running:  
```sh
sudo apt install vulkan-tools && vulkaninfo | less 
```

- GPU Drivers
    - The automatic installation for Cuda often doesn't work or they are not installed in the Ubuntu installation if you disabled third-party drivers. Either way, we recommend using the [manual installation guide](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/) to install them.
