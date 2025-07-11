# Planners

You can easily add custom nav2 planners for selection by adding them to one of the subdirectories in `arena_simulation_setup/configs/nav2`.

E.g. a controller `my_controller` defined in `arena_simulation_setup/configs/nav2/my_controller/controller_config.yaml` can be loaded using `local_planner:=my_controller`.