## Robots
A robot named `<robot>` can be used in the simulation by integrating it into the `arena_simulation_setup/entities/robots/` directory. Create a new subdirectory `<robot>` with the following structure:

```
<robot>/
    urdf/
        <robot>.urdf.xacro
    control.yaml
    mappings.yaml
    model_params.yaml
```

These files are the bare necessity, they can (and for cleanliness should) be split up across smaller files. Robot `turtlebot` can be taken as an inspiration.

### urdf/<robot>.urdf.xacro

XACRO file compiled on-the-fly into a single `<robot>.urdf` file at run-time.

### control.yaml

Standard ros2 control file.

### model_params.yaml

Contains a robot configuration that is used across many different nodes.
Include the following parameters and make sure they are correct:

```yaml
robot_base_frame: <relative frame to robot root>
robot_odom_frame: 'odom'

observation_sources_string: <same as observation_sources, just as a space-separated string>
observation_sources:
  - <list every key of observation_sources_dict here>
observation_sources_dict: # include as many as you need
  <observation_name>:
    topic: ${namespace}/<relative_topic>
    max_obstacle_height: 2.0
    clearing: True
    marking: True
    data_type: "LaserScan" # change this as needed

polygons:
  - StopPolygon
polygons_dict:
  StopPolygon:
    type: "polygon"
    points: "[ [0.1, 0.1], [0.1, -0.1], [-0.1, -0.1], [-0.1, 0.1] ]" # change this to your robot shape
    action_type: "stop"
    min_points: 4
    visualize: True
    polygon_pub_topic: "polygon_stop"
    enabled: True

footprint: "[ [0.1, 0.1], [0.1, -0.1], [-0.1, -0.1], [-0.1, 0.1] ]" # change this to your robot shape

actions:
  continuous:
    angular_range:
    - -1.9
    - 1.9
    linear_range:
    - -0.4
    - 0.4
  discrete:
  - angular: 0.0
    linear: 0.3
    name: move_forward
  - angular: 0.0
    linear: -0.15
    name: move_backward
  - angular: 0.35
    linear: 0.15
    name: turn_left
  - angular: -0.35
    linear: 0.15
    name: turn_right
  - angular: 0.75
    linear: 0.0
    name: turn_strong_left
  - angular: -0.75
    linear: 0.0
    name: turn_strong_right
  - angular: 0.0
    linear: 0.0
    name: stop
is_holonomic: false
```

### mappings.yaml

Contains topic mappings for the gazebo ros bridge. Copy this from turtlebot and modify as needed.
The absolute minimum, without any sensors, should be:

```yaml
[
  ## COMMON
  # Odometry (Gazebo -> ROS2)
  {
    "gz_topic": "/model/{robot_name}/odometry",
    "ros_topic": "odom",
    "ros_type": "nav_msgs/msg/Odometry",
    "gz_type": "gz.msgs.Odometry",
    "direction": "[",
  },
  # Velocity command (ROS2 -> Gazebo)
  {
    "gz_topic": "/model/{robot_name}/cmd_vel",
    "ros_topic": "cmd_vel",
    "ros_type": "geometry_msgs/msg/Twist",
    "gz_type": "gz.msgs.Twist",
    "direction": "]",
  },
  # TF Data (Gazebo -> ROS2)
  {
    "gz_topic": "/model/{robot_name}/tf",
    "ros_topic": "/tf",
    "ros_type": "tf2_msgs/msg/TFMessage",
    "gz_type": "gz.msgs.Pose_V",
    "direction": "[",
  },
  {
    "gz_topic": "/model/{robot_name}/tf_static",
    "ros_topic": "/tf_static",
    "ros_type": "tf2_msgs/msg/TFMessage",
    "gz_type": "gz.msgs.Pose_V",
    "direction": "[",
  },
  # Joint states
  {
    "gz_topic": "/model/{robot_name}/joint_states",
    "ros_topic": "/joint_states",
    "ros_type": "sensor_msgs/msg/JointState",
    "gz_type": "gz.msgs.Model",
    "direction": "[",
  },
]
```

## Closing Notes

Keep in mind that the current gazebo version does not fully support ros2 control. You probably need to add a differential drive plugin to the urdf.

### Using the Robot
To use and test the robot, start the simulation with `robot:=<robot>` set as a command line argument (or use the robot setup configuration file).