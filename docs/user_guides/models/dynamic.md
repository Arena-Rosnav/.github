## Dynamic Obstacles

A dynamic obstacle named `<model_name>` can be added by first acquiring
- yaml model file: 2D representation
- sdf model file: 3D representation (not required by flatland)

These files need to be saved to the folder `src/arena-simulation-setup/obstacles/dynamic_obstacles` with the following structure

```
<model_name>/
    sdf/
        <model_name>.sdf
    yaml/
        <model_name>.yaml
```

The model can now be used under `<model_name>` in task configurations.

### Gazebo

To properly integrate the dynmaic obstacle into gazebo the following two plugins need to be added
- libActorCollisionsPlugin
- libPedsimGazeboActorPlugin

The libPedsimGazeboActorPlugin has the following plugin parameters:
- animation_factor
- model_height

The libActorCollisionsPlugin can have multiple scaling parameters with:
- collision 
- scale
