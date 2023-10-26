# pedsim_waypoint_plugin

## Overview

The `pedsim_waypoint_plugin` provides an interface for overriding the default behavior of agents (also: pedestrians, peds, dynamic obstacles) with any other abstract behavior.

### Architecture

Overriding pedsim calculations is achieved by forwarding the output of the `pedsim_simulator/waypoint_plugin_feedback` service to an additional `pedsim_waypoint_plugin` node (plugin node), which acts as part of a feedback loop and sends physical forces for the actors back to `pedsim_simulator/waypoint_plugin_feedback`.

![architecture](./architecture.png "Architecture")

_note_: this is purely prescriptive for the feedback loop, any other communication between plugins/simulators can be implemented using additional rostopics

### Data Flow

In order to allow sophisticated calculations (equal-or-better philosophy against pedsim), the basic `simulated_agents` data is complemented by additional pre-calculation data and packaged into a `WaypointPluginDataframe` message (defined at `pedsim_ros/pedsim_msgs/msg/WaypointPluginDataframe.msg`).

The pre- and post-processing of the full messages coming in and out of the plugin node is handled automatically, and the raw data is handled inside a pre-selected `WaypointPlugin`.

![dataflow](./dataflow.png "Data Flow")

This results in an event-driven, control-loop architecture, with its abstract nature allowing provisions for different possible approaches to computational scalability challenges, e.g.:
- stateful computations
- output interpolation

### Usage
#### Internals

Data arriving at the `WaypointPlugin` is split into an `InputData` object with the following properties.

|field|type|properties (selection)|description|
|---|---|---|---|
|`header`|`std_msgs/Header`|`.stamp.secs`<br/>`.stamp.nsecs`|simulation time|
|`robots`|`List[pedsim_msgs/RobotState]`|`.pose.position`<br/>`.pose.orientation`|robot name, pose|
|`groups`|`List[pedsim_msgs/AgentGroup]`|-|currently unpopulated|
|`waypoints`|`List[pedsim_msgs/Waypoint]`|`.position`|static obstacles|
|`agents`|`List[pedsim_msgs/AgentState]`|`.direction`<br/>`.twist`<br/>`.forces`<br/>`.destination`|extended agent state|

The aim of the WaypointPlugin is to transform this data into a single `OutputData` object of type `List[pedsim_msgs/AgentFeedback]` which is then published to the simulator and into the pedsim feedback loop.

`AgentFeedback` contains the fields
- `uint64 id`: agent id, should be copied from the `pedsim_msgs/AgentState` messages
- `std_msgs/Vector3 force`: force vector to override simulation with
- `bool unforce`: if true, unforce the vector and let pedsim_simulator fully take back over  

When an `AgentFeedback` is sent, it latches and overrides the force vector until it is overridden by a new feedback or unforced. This does not (directly) affect the recalculation of forces for the `InputData`. Actors will always face the current movement direction (`.twist.linear`).

### Writing a WaypointPlugin

1. Initialize your project structure inside the `arena-rosnav` repository by opening `arena-rosnav/utils/ros/rosnodes/pedsim_waypoint_plugin/pedsim_waypoint_plugin` as a workspace.

2. Create a subdirectory `<your_plugin_name>` in `plugins`.

3. Create a file `<your_plugin_name>/main.py`.

4. Add the line `from .plugins.<your_plugin_name> import main # noqa` to `__init__.py`.

5. Add a unique identifier to the `WaypointPluginName` type in `pedsim_waypoint_plugin.py`. (This identifier will be used in the launchfile argument).

6. Add the following skeleton to your `main.py`:

```python
from pedsim_waypoint_plugin.pedsim_waypoint_generator import OutputData, PedsimWaypointGenerator, InputData, WaypointPluginName, WaypointPlugin
import pedsim_msgs.msg

@PedsimWaypointGenerator.register(WaypointPluginName.<YOUR_PLUGIN_NAME>)
class Plugin_<your_plugin_name>(WaypointPlugin):
    def __init__(self):
        ...

    def callback(self, data) -> OutputData:
        return [pedsim_msgs.msg.AgentFeedback(unforce=True) for agent in data.agents]
```

You can now use your plugin by setting the roslaunch argument `pedsim_waypoint_plugin:=<your_plugin_name>`.

Example launch:
```sh
roslaunch arena_bringup start_arena.launch simulator:=gazebo task_mode:=scenario model:=jackal map_file:=map_empty pedsim_waypoint_plugin:=spinny
```

Apart from this basic structure, the content of your plugin's folder is free to modify. It is recommended to practice by implementing these rudimentary plugins:

1. Make each actor walk in a circle (stateless),
2. Make each actor walk in a "figure 8" (stateful).

A reference implementation for 1. is provided in the `spinny` plugin.
