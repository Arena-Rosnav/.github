### Random

#### Obstacles

The random task spawns a variety of static and dynamic obstacles upon each reset, with models chosen randomly. To adjust the types and quantities of obstacles, modify the `task_mode/random` parameter in `src/arena-rosnav/arena_bringup/configs/task_generator.yaml`.

e.g.

```yaml
RANDOM:
  seed: -1
  static:
    min: 0
    max: 0
    models: ["shelf"]
  interactive:
    min: 1
    max: 5
    models: ["shelf"]
  dynamic:
    min: 1
    max: 5
    models: ["actor1"]
```

For each obstacle type, define the range of spawnable instances (minimum and maximum) and the available models. To specify the amount of obstacles for a specific model, refer to the [Parametrized](parametrized.md) task mode.

#### Robot

The random task mode generates new start and goal positions for the robot with each task reset.

#### Seed

The seed parameter controls the random seed used for obstacle, robot start, and goal position generation. Providing a seed allows for stable reproduction of tasks. A value of -1 indicates a random seed will be used.

#### Usage

```bash
roslaunch arena_bringup start_arena.launch model:=jackal
```

![Random](./gifs/random.gif)