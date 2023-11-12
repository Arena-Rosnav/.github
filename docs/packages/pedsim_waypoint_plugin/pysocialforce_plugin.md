# PySocialForce Plugin

## Overview
The **PySocialForce** plugin is based on the social force model for python created by [Yuxiang Gao](https://github.com/yuxiang-gao/PySocialForce). 

### Social Force Model
The social force model uses pedsim ros inspired forces to compute the forces of the agents. There is an array of different forces that can be taken into account like desired force, obstacle force & social force.  
Most importantly, it can model group behavior well. It includes different forces specifically for groups, making them look into the same direction, stick to each over, etc..  

## Extending the Plugin
Another great property is the modularity and the extendability of this plugin. This means, the effect of different forces can be easily made smaller or bigger, subsets of all forces can be used to control the computation of the overall force and new forces can be added. To extend the plugin by a new force, do the following:

1. Initialize your project structure inside the `arena-rosnav` repository by opening `arena-rosnav/utils/ros/rosnodes/pedsim_waypoint_plugin/pedsim_waypoint_plugin` as a workspace.

2. Create a new force class in `plugins/pysocial1/pysocialforce/forces.py` with the following structure:

```python
class <your_force_name>Force(Force):
    def _get_force(self):
        force = np.zeros((self.peds.size(), 2))
        return force
```

3.  Now, you need to add this force to the simultor. In the simulator class in `plugins/pysocial1/pysocialforce/simulator.py`, add `forces.<your_force_name>Force` as a list element to `force_list` in the `make_forces` method.

Now your new force will be taken into account and the force will be calculated through your `_get_force` method every time new forces for all pedestrians will be requested. To work on the way the force is computed, edit your `_get_force` method.  
About `_get_force`:
- This method is expected to return a `nx2` numpy ndarray which includes the x (col 0) and y (col 1) values in row `i` for the force vector of agent `i`. This force will be sumed up with all other forces.
- You can access the postion, velocity and destination of every agent by using `self.peds.pos()`, `self.peds.vel()` and `self.peds.goal()`
- If you are missing information for your computation (e.g. robot position) you can add them as attributes to the simulator.ped object and update them with the ped.update() method.