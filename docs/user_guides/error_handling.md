# Common Error Handling Measurements

## Activate poetry shell
The poetry shell needs to be activated all the time!
```
cd arena-rosnav # navigate to the arena-rosnav directory
poetry shell
```

## Building and sourcing the workspace
Building and sourcing the workspace anew can solve many errors, the most common one is the following

!!! error
    ```
    RLException: [start_arena_flatland.launch] is neither a launch file in package [arena_bringup] nor is [arena_bringup] a launch file name
    The traceback for the exception was written to the log file
    ```

```

cd catkin_ws # navigate to the catkin_ws directory
catkin_make
source devel/setup.zsh # if you use bash: source devel/setup.bash 
```

!!! note
    After any changes inside a subdirectory of the utils directory the workspace needs to be build and sourced again.

## Updating the utils of the workspace
Sometimes your utils directory is outdated or defective. In these cases, simply updating or deleting the utils folder and updating the workspace might do the trick.
```
cd arena-rosnav # navigate to the arena-rosnav
rosws update
```

!!! note
    Don't forget to [build and source the workspace](error_handling.md#building-and-sourcing-the-workspace) afterwards!

## Updating stable-baselines3
```
cd caktin_ws/src/utils/stable-baselines3 # navigate to the stable-baselines3 directory
pip install -e.
```