# Add new environment

To add a new environment to _Arena Benchmark_, you mainly have to do two seperate things. At first, you have to create a new environment adapter for the task manager. Additionally, you have to create and integrate a new launch file for your environment.

## New environment in task generator

In order to use a custom environment with _Arena Benchmark_, you have to create an interface between environment and our task generator. Therefore, you will implement your own environment class in [task_generator/task_generator/environments/](https://github.com/Arena-Rosnav/task-generator/tree/dev/task_generator/environments), deriving from `BaseEnvironemnt`. See the [`flatland_environment.py`](https://github.com/Arena-Rosnav/task-generator/tree/dev/task_generator/environments/flatland_environment.py) as an example.

In your environment interface you have to implement following methods:

#### before_reset_task

Function that is run before the task is resetted. This can be used to stop the simulation.

#### after_reset_task

Function that is run after the task is resetted. This can be used to start the simulation again.

#### remove_all_obstacles

Remove all obstacles from the current simulation.

#### spawn_random_dynamic_obstacle

Used to spawn new dynamic obstacles.

**Arguments:**

- position `[float, float, float]`: denoting x, y and angle
- min_radius `float`: minimal radius of the obstacle
- max_radius `float`: maximal radius of the obstacle
- linear_vel `float`: linear velocity
- angular_vel_max `float`: maximal angular velocity

#### spawn_random_static_obstacles

Used to spawn new static obstacles.

**Arguments:**

- position `[float, float, float]`: denoting x, y and angle
- min_radius `float`: minimal radius of the obstacle
- max_radius `float`: maximal radius of the obstacle

#### publish_goal

Publish the goal to desired subscribers.

**Arguments:**

- goal `[float, float]` denoting x, y

#### move_robot

Move a robot, speicified by its name, to a given position.

**Arguments:**

- position `[float, float]`: denoting x, y
- name `string`: denoting the name of the robot that should be moved

#### spawn_robot

Spawn a new robot in the environment.

**Arguments:**

- name `string`: Name the robot.
- robot_model `string`: The name of the robot you want to spawn. (rto, jackal, ...)
- namespace `string`: Namespace of the spawned robot.

#### spawn_obstacle

**TODO**

## Launch file for environment

To launch the environment with all other packages, you should create a custom launch file in the `arena_bringup` package. Refer to the [`flatland_environment.launch`](https://github.com/Arena-Rosnav/arena-bench/blob/dev/arena_bringup/launch/testing/flatland_simulator.launch) to see how we did this with flatland. You should then include your launch file into the main `start_arena.launch` and only launch your environment when the environment argument is equal to your environment name.

```xml
  <!-- Flatland -->
  <include file="$(find arena_bringup)/launch/testing//simulators/flatland.launch" if="$(eval arg('environment') == 'flatland')">
    <arg name="visualization" default="$(arg visualization)" />
    <arg name="rviz_file" value="$(arg rviz_file)" />
    <arg name="model" value="$(arg model)" />
  </include>

  <!-- Gazebo -->
  <include file="$(find arena_bringup)/launch/testing/simulators/gazebo.launch" if="$(eval arg('environment') == 'gazebo')">
    <arg name="model" value="$(arg model)" />
    <arg name="rviz_file" value="$(arg rviz_file)" />
    <arg name="show_rviz" default="$(arg show_rviz)" />
    <arg name="headless" default="false"/>
    <arg name="gui" default="true"/>
    <arg name="world" default="$(arg world_file)" />
  </include>

  <!-- YOUR NEW ENVIRONEMENT -->
  <include file="$(find arena_bringup)/launch/testing/simulators/gazebo.launch" if="$(eval arg('environment') == 'YOUR_ENVIRONMENT_NAME')">
    <!-- LIST OF ALL ARGUMENTS YOU NEED TO LAUNCH YOUR ENVIRONMENT APPROPRIATELY -->
  </include>
```
