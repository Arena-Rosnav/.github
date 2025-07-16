# Usage

## Environment

The arena environment can be activated using `. arena.bash` while in the workspace root. This automatically activates the ros2 and python environments.

### (Optional) update.

Update arena using `ros2 run arena_bringup pull`.

Build your workspace using `. colcon_build`. This is a wrapper around `colcon_build` and only rebuilds packages with newer timestamps in the src folder. Specify `--packages-select` to ignore that. All arguments are passed through to `colcon_build`.

The build is performed as a symlink install and may lead to issues when moving/deleting files. In that case, manually run `rm -r build/<problematic_package> install/<problematic_package>` and rebuild.

## Starting

To start a simulation just launch the `arena.launch` file in the `arena_bringup` package.

```sh
ros2 launch arena_bringup arena.launch.py sim:=gazebo
```

The launch file takes following parameters:

| Name          | Default                 | Type   | Description |
| ------------- | ----------------------- | ------ | ----- |
| robot         | jackal                  | string | robot you want to simulate. <br/> Supports syntax `name[2]` for multiple instances of the same robot. <br/> Supports syntax `file.yaml` to reference the robot setup file `arena_bringup/configs/robot_setup/file.yaml` <br/> Supports comma-separated lists of the above, e.g. `jackal,turtlebot[2],demo.yaml` |
| world      | map_empty                    | string | Name of the world.|
| sim     | dummy                | string | Name of the simulator |
| human     | hunav                | string | Name of the human simulator  |
| global_planner | navfn    | string | Name of the nav2 planner you want to use.|
| local_planner | dwb   | string | Name of the nav2 controller you want to use.|
| tm_robots     | random | string | robots task mode |
| tm_obstacles  | random | string | obstacles task mode |
| tm_modules    | | string | modules task modes |
| record_data   | false                   | bool   | Wether you want to record the<br /> robot data during the run for <br />later evaluations with the `arena_evaluation` package. |
