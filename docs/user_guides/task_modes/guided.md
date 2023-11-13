### Guided

The guided task behaves identically to the [random task](random.md) except for the robots. In this task mode you can manually set a cyclical sequence of goal waypoints for the robots using rviz.

- Use the `2D Nav Goal` tool to append a goal position to the sequence
- Use the `Publish Point` tool anywhere to trigger a full reset

The robots stay still and wait until the first goal is published. The robots also share the list of waypoints, but track them individually.
The goal sequence is tracked in rosparam `/guided_waypoints`.
