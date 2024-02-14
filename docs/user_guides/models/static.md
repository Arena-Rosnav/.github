## Static Obstacles

A static obstacle named `<model_name>` can be added by first acquiring
- yaml model file: 2D representation
- sdf model file: 3D representation (not required by flatland)

These files need to be saved to `simulation_setup/entities/obstacles/static/` with the following structure

```
<model_name>/
    sdf/
        <model_name>.sdf
    yaml/
        <model_name>.yaml
```

The model can now be used under `<model_name>` in task configurations.