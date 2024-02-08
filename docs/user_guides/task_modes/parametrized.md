### Parametrized

The parametrized task acts like the [random task](random.md) but for each obstacle a different amount can be set. The parameters are configured in a xml file in `arena_bringup/configs/parametrized`.

The parameter file looks like this:

```xml
<?xml version="1.0" ?>
<random>
  <static>
    <obstacle name="shelf">
      <min>2</min>
      <max>7</max>
      <type>shelf</type>
      <model>"shelf"</model>
    </obstacle>
  </static>
  <dynamic>
    [...]
  </dynamic>
</random>

```

To use the parametrized file set the parameter `task_mode/scenario/parameterized` in `arena_bringup/configs/task_generator.yaml`.