## Robots
A robot named `<robot>` can be used in the simulation by integrating it into the `simulation_setup/entities/robots/` directory. Create a new subdirectory `<robot>` with the following structure:

```
<robot>/
    configs/
        costmaps/
            local_costmap_params.yaml
            global_costmap_params.yaml
        mbf/
            <planner>_params.yaml
    launch/
        control.launch
    urdf/
        <robot>.urdf.xacro
    yaml/
        <robot>.yaml
    <robot>.model.yaml -> yaml/<robot>.yaml
    model_params.yaml
```

These files are the bare necessity, they can (and for cleanliness should) be split up across smaller files. Robot `jackal` can be taken as an inspiration.

### costmaps
The `costmaps` folder contains at least the two required costmap configurations (local/global) which are loaded directly to the parameter server to be used by `costmap_2d`.
Additionaly, planner-dependent configurations can and should be defined in its respective `<planner>_local_planner_params.yaml` (e.g. `teb_local_planner_params.yaml`). Some planners don't load this configuration file at all, some planners share one configuration file. Always double check `arena_simulation_setup/launch/planners/<type>/mbf_<planner>.launch` if you deviate from the "basic" planners (`teb`, `dwa`, ...).

### launch/control.launch.py

`control.launch.py` is called from the `arena_bringup/launch/testing/robot.launch` launchfile, which itself is started indirectly by the robot manager. This launch process automatically starts commonly used processes, which means that `control.launch` _should not_:
- spawn the robot model in the simulator
- start a `move_base`-interfacing planner node
- publish a static transform
- load `model_params.yaml` to the parameter server
- start a data recorder node
- [Gazebo only] start a `robot_state_publisher` node

But usually _should_:
- spawn joint/motor controllers
- start an odometry publisher (e.g. using `odom_pub`)

To this end, the launchfile gets passed a `robot_namespace` argument, which contains the target namespace of this robot instance in ros. The robot is provided the following topics for interfacing:

#### Subscribe to
- `robot_namespace/cmd_vel`

#### Publish to
- `robot_namespace/joint_states`
- `robot_namespace/odom`

### urdf/<robot>.urdf.xacro

XACRO file compiled on-the-fly into a single `<robot>.urdf` file which is used for spawning the robot into Gazebo.

### model_params.yaml

Contains a robot configuration that is read by the planner node, and usually contains reference frame definitions, `move_base` actions, and sensor descriptions.

## Closing Notes

### Using the Robot
To use and test the robot, start the simulation with `model:=<robot>` set as a command line argument (or use the robot setup configuration file).

### Robot Repositories
Robot repositories and stand-alone packages can be integrated in the same way, but have to be added to `extern/` as specified by [Contributing](/docs/contribute.md).
