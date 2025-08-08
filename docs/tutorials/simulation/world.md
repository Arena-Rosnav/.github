### Worlds

Add a new world under `arena_simulation_setup/worlds`.

A world folder must contain the following structure:

```
<world_name>/
    map/
        map.png
        map.yaml
    scenarios/
        default.json
    world.yaml
```

#### Generating Worlds

You can automatically generato worlds using `arena-tools`:

`ros2 run arena_simulation_setup generate_world`

You can implement custom world generators in `arena_simulation_setup/arena_simulation_setup/utils/generative`.

#### Editing Scenarios
You can modify and add scenarios using `arena-tools`:

`ros2 run arena_tools scenario_editor`
