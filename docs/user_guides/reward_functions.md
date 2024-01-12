# Reward Functions

## RewardFunction and RewardUnit Class

The reward function class contains the logic for calculating the intended reward function. It is the centralized instance keeping track of the current steps' reward and other environment related information. Its main component is the list of `RewardUnits` each taking up observations and returning a certain reward value. In order to retrieve the current reward, they are applied each after another, given some observation, and by that principle, they form a reward function.

Source Code:

- [RewardFunction](https://github.com/Arena-Rosnav/arena-rosnav/blob/master/utils/misc/rl_utils/rl_utils/utils/rewards/reward_function.py)
- [RewardUnit](https://github.com/Arena-Rosnav/arena-rosnav/blob/master/utils/misc/rl_utils/rl_utils/utils/rewards/reward_units/base_reward_units.py)

## Custom RewardUnit

RewardUnits are modular components of our reward function which apply a certain logic on given observation in order to retrieve a reward value. Based on the purpose, it can give a positive or negative feedback on some aspect of the observation space.

To implement a new RewardUnit, you need to define two methods:

1. Inheret from base RewardUnit class

   - Implement these functions:

     - \_\_init\_\_: This method should initialize all configurable parameters of the reward unit. Whether the RewardUnit should be applied when the safe distance is violated, depends on the specific use case. You should decide based on the requirements of your mechanism.
     - \_\_call\_\_: This method should implement the main logic for the RewardUnit. Essentially, the logic should alter the reward value of the wrapping RewardFunction and optionally its info dicitonary which in turn can be used in the Gymnasium environment. The base class already implements methods you can fallback to.

2. Add the RewardUnit to the Registry

   - Use the @RewardUnitFactory.register("\*unit_name\*") decorator to add your new unit to the registry. The name should be unique and self-explanatory as it is the identifier for the unit.
   - Add the classname to the \_\_all\_\_ list on the top of the file.

**Note**: If you need to access information that isn’t provided by the [ObservationManager](https://github.com/Arena-Rosnav/arena-rosnav/blob/master/utils/misc/rl_utils/rl_utils/utils/observation_collector/observation_manager.py), you can access the `internal_state_info` of the RewardFunction class. There are already methods to add/remove data to/from the internal state dictionary.

For more detailed information about the class structure, please refer to the source code.

## Implemented RewardUnits

| Reward Unit              | Link                                                                                                                                                                             |
| ------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| "goal_reached"           | [click](https://github.com/Arena-Rosnav/arena-rosnav/blob/8389266de1476ff6769ed3c0c282915f7e465d12/utils/misc/rl_utils/rl_utils/utils/rewards/reward_units/reward_units.py#L28)  |
| "collision"              | [click](https://github.com/Arena-Rosnav/arena-rosnav/blob/8389266de1476ff6769ed3c0c282915f7e465d12/utils/misc/rl_utils/rl_utils/utils/rewards/reward_units/reward_units.py#L208) |
| "safe_distance"          | [click](https://github.com/Arena-Rosnav/arena-rosnav/blob/8389266de1476ff6769ed3c0c282915f7e465d12/utils/misc/rl_utils/rl_utils/utils/rewards/reward_units/reward_units.py#L78)  |
| "no_movement"            | [click](https://github.com/Arena-Rosnav/arena-rosnav/blob/8389266de1476ff6769ed3c0c282915f7e465d12/utils/misc/rl_utils/rl_utils/utils/rewards/reward_units/reward_units.py#L117) |
| "approach_goal"          | [click](https://github.com/Arena-Rosnav/arena-rosnav/blob/8389266de1476ff6769ed3c0c282915f7e465d12/utils/misc/rl_utils/rl_utils/utils/rewards/reward_units/reward_units.py#L155) |
| "distance_travelled"     | [click](https://github.com/Arena-Rosnav/arena-rosnav/blob/8389266de1476ff6769ed3c0c282915f7e465d12/utils/misc/rl_utils/rl_utils/utils/rewards/reward_units/reward_units.py#L248) |
| "reverse_drive"          | [click](https://github.com/Arena-Rosnav/arena-rosnav/blob/8389266de1476ff6769ed3c0c282915f7e465d12/utils/misc/rl_utils/rl_utils/utils/rewards/reward_units/reward_units.py#L391) |
| "approach_globalplan"    | [click](https://github.com/Arena-Rosnav/arena-rosnav/blob/8389266de1476ff6769ed3c0c282915f7e465d12/utils/misc/rl_utils/rl_utils/utils/rewards/reward_units/reward_units.py#L284) |
| "follow_globalplan"      | [click](https://github.com/Arena-Rosnav/arena-rosnav/blob/8389266de1476ff6769ed3c0c282915f7e465d12/utils/misc/rl_utils/rl_utils/utils/rewards/reward_units/reward_units.py#L349) |
| "abrupt_velocity_change" | [click](https://github.com/Arena-Rosnav/arena-rosnav/blob/8389266de1476ff6769ed3c0c282915f7e465d12/utils/misc/rl_utils/rl_utils/utils/rewards/reward_units/reward_units.py#L427) |

## Yaml File Format

The reward functions are specified in yaml files. Here, the **key** represents the name of the desired RewardUnit and the key its' arguments.

```yaml
goal_reached:
  reward: 15

safe_distance:
  reward: -0.2

collision:
  reward: -10

no_movement:
  reward: -0.01
  _on_safe_dist_violation: true

approach_goal:
  pos_factor: 0.5
  neg_factor: 0.6
  _on_safe_dist_violation: true

distance_travelled:
  consumption_factor: 0.005
  lin_vel_scalar: 1.0
  ang_vel_scalar: 0.001
  _on_safe_dist_violation: false

reverse_drive:
  reward: -0.0001
  _on_safe_dist_violation: true

approach_globalplan:
  pos_factor: 0.3
  neg_factor: 0.5
  _on_safe_dist_violation: false

follow_globalplan:
  min_dist_to_path: 0.5
  reward_factor: 0.1
  _on_safe_dist_violation: false

abrupt_velocity_change:
  vel_factors:
    "0": 0.3
    "1": 0.07
    "2": 0.07
  _on_safe_dist_violation: true
```

## Custom RewardFunction

Custom reward functions can be simply created by combining arbitrary RewardUnits.

- Create a new yaml file or copy an existing one in the reward_functions sub-directory.
- Add the desired RewardUnits by specifying their names and parameters.
- Add the reward function in the training config file to use the newly created reward function.

It is recommended to check out the [reward_units.py](https://github.com/Arena-Rosnav/arena-rosnav/blob/master/utils/misc/rl_utils/rl_utils/utils/rewards/reward_units/reward_units.py).
