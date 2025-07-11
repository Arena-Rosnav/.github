## Static Obstacles

A static obstacle named `<model_name>` can be added by first acquiring
- sdf model file: 3D representation (required for gazebo)
- usd model file: 3D representation (optional, automatically converted from sdf otherwise)

These files need to be saved to `simulation_setup/entities/obstacles/static/` with the following structure

```
<model_name>/
    sdf/
        <model_name>.sdf
    usd/
        <model_name>.usd
```

The model can now be used under `<model_name>` in task configurations.