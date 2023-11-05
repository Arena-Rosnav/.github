### Staged

The staged task mode is designed for the trainings process of arena-rosnav. In general, it behaves like the [random task mode](random.md) but there are multiple stages between one can switch. Between the stages, the amount of static and dynamic obstacles changes. The amount of obstacles is defined in a curriculum
file, the path to said file is a key in the `paths` parameter. The curriculum files are in `src/arena-rosnav/arena_bringup/training/training_curriculums`. The different models that can be spawned are defined in the parameter `task_mode/random` in `src/arena-rosnav/arena_bringup/configs/task_generator.yaml`.

The curriculum file consist of multiple defined stages. One stage can be defined like this:

```yaml
-
  goal_radius: 1.0
  static: 0
  interactive: 0
  dynamic: 0
```

To use the staged file set the parameter `task_mode/scenario/staged` in `src/arena-rosnav/arena_bringup/configs/task_generator.yaml`.