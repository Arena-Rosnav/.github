# Arena-Rosnav Reward Functions

## Reward Functions in Reinforcement Learning

Reward functions play a crucial role in reinforcement learning, particularly in the context of robotic navigation. In reinforcement learning, an agent learns to make decisions by interacting with an environment and receiving feedback in the form of rewards. These rewards serve as a signal to guide the agent towards achieving its goals.

In the domain of robotic navigation, reward functions define the objectives and priorities for the robot's behavior. They provide a quantitative measure of how well the robot is performing a given task, such as reaching a target location or avoiding obstacles. By carefully designing reward functions, we can shape the learning process and guide the robot towards desired behaviors.

The design of reward functions in robotic navigation involves striking a balance between different objectives. For example, we may want the robot to reach the goal quickly while avoiding collisions with obstacles. This requires assigning appropriate rewards to different actions and states, considering factors such as distance to the goal, proximity to obstacles, and energy consumption.

By leveraging reinforcement learning techniques and well-designed reward functions, we can enable robots to autonomously navigate complex environments, adapt to changing conditions, and optimize their behavior over time. This has significant implications for various applications, including autonomous vehicles, warehouse robots, and search and rescue missions.

In the following sections, we will introduce you to the reward functions used in the Arena-Rosnav project. We will walk through the different components of the reward functions and explain how they are implemented in the code. We will also discuss the design choices and trade-offs involved in the process.

## Reward Functions in Arena-Rosnav

#### Reward Function Components

The smallest unit of a reward function is a reward unit. A reward unit is a function that computes a reward value based on the current state of the robot and the simulation. It can be as simple as a constant value or as complex as a multi-objective function. In the Arena-Rosnav project, we use reward units to define the different objectives and priorities for the robot's behavior.
The RewardUnit ćlass should be inherited by all reward units. It provides methods for adding rewards and information, checking parameters, and resetting the reward unit. It also contains an abstract method for calculating the reward.

```python
class RewardUnit(ABC):
    """
    Reward Unit Base Class

    This class represents a reward unit and is an abstract base class (ABC).
    It provides methods for adding rewards and information, checking parameters, and resetting the reward unit.
    It also contains an abstract method for calculating the reward.

    Attributes:
        _reward_function (RewardFunction): The RewardFunction instance holding this unit.
        _on_safe_dist_violation (bool): Whether the unit is applied on safe distance violation.
    """

    def __init__(
        self,
        reward_function: RewardFunction,
        _on_safe_dist_violation: bool = True,
        *args,
        **kwargs
    ) -> None:
        """Initializes the RewardUnit.

        Args:
            reward_function (RewardFunction): The RewardFunction instance holding this unit.
            _on_safe_dist_violation (bool, optional): Whether the unit is applied on safe distance violation. Defaults to True.
        """
        self._reward_function = reward_function
        self._on_safe_dist_violation = _on_safe_dist_violation

    @property
    def robot_radius(self):
        return self._reward_function.robot_radius

    @property
    def on_safe_dist_violation(self):
        return self._on_safe_dist_violation

    def get_internal_state_info(self, key: str) -> Any:
        """
        Retrieves internal state information from the RewardFunction wrapper based on the provided key.

        Args:
            key (str): The key to identify the internal state information.

        Returns:
            Any: The internal state information associated with the provided key.
        """
        return self._reward_function.get_internal_state_info(key=key)

    def add_reward(self, value: float):
        """Adds the given value to the episode's reward.

        Args:
            value (float): _description_
        """
        self._reward_function.add_reward(value=value)

    def add_info(self, info: dict):
        """Adds the given information to the episode's info dict.

        Args:
            info (dict): _description_
        """
        self._reward_function.add_info(info=info)

    def check_parameters(self, *args, **kwargs):
        """Method to check the parsed unit parameters. Send warning if params were chosen inappropriately."""
        pass

    def reset(self):
        """Method to reset the unit state after each episode."""
        pass

    @abstractmethod
    def __call__(self, *args: Any, **kwargs: Any) -> Any:
        """
        Placeholder method for calling the reward unit. It should alter the reward and possibly the info dict.
        This method should be overridden in the derived classes.
        """
        raise NotImplementedError()
```

##### Implemented Reward Units

For technical details on the implemented reward units, please refer to the [source code](https://github.com/Arena-Rosnav/arena-rosnav/blob/master/utils/misc/rl_utils/rl_utils/utils/rewards/reward_units/reward_units.py).

| Reward Unit Name                                                                                                                                                                                                  | Registry Name                      | Arguments                                                                                                                                                                                                                                                                                                                                                                                                                              | Explanation                                                        |
| ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------ |
| [RewardGoalReached](https://github.com/Arena-Rosnav/arena-rosnav/blob/5e87e3d9b59c72280d0b92e622abb4787643c20d/utils/misc/rl_utils/rl_utils/utils/rewards/reward_units/reward_units.py#L34)                       | **goal_reached**                   | **reward** (_float_): The reward value for reaching the goal.                                                                                                                                                                                                                                                                                                                                                                          | Calculates the reward when the goal is reached.                    |
| [RewardSafeDistance](https://github.com/Arena-Rosnav/arena-rosnav/blob/5e87e3d9b59c72280d0b92e622abb4787643c20d/utils/misc/rl_utils/rl_utils/utils/rewards/reward_units/reward_units.py#L88)                      | **safe_distance**                  | **reward** (_float_): The reward value for violating the safe distance.                                                                                                                                                                                                                                                                                                                                                                | Calculates the reward when violating the safe distance.            |
| [RewardNoMovement](https://github.com/Arena-Rosnav/arena-rosnav/blob/5e87e3d9b59c72280d0b92e622abb4787643c20d/utils/misc/rl_utils/rl_utils/utils/rewards/reward_units/reward_units.py#L132)                       | **no_movement**                    | **reward** (_float_): The reward value for no movement.                                                                                                                                                                                                                                                                                                                                                                                | Calculates the reward when there is no movement.                   |
| [RewardApproachGoal](https://github.com/Arena-Rosnav/arena-rosnav/blob/5e87e3d9b59c72280d0b92e622abb4787643c20d/utils/misc/rl_utils/rl_utils/utils/rewards/reward_units/reward_units.py#L170)                     | **approach_goal**                  | **pos_factor** (_float_): Positive factor for approaching the goal<br>**neg_factor** (_float_): Negative factor for distancing from the goal.                                                                                                                                                                                                                                                                                          | Calculates the reward when approaching the goal.                   |
| [RewardCollision](https://github.com/Arena-Rosnav/arena-rosnav/blob/5e87e3d9b59c72280d0b92e622abb4787643c20d/utils/misc/rl_utils/rl_utils/utils/rewards/reward_units/reward_units.py#L223)                        | **collision**                      | **reward** (_float_): The reward value for colliding with an obstacle.                                                                                                                                                                                                                                                                                                                                                                 | Calculates the reward when a collision occurs.                     |
| [RewardDistanceTravelled](https://github.com/Arena-Rosnav/arena-rosnav/blob/5e87e3d9b59c72280d0b92e622abb4787643c20d/utils/misc/rl_utils/rl_utils/utils/rewards/reward_units/reward_units.py#L268)                | **distance_travelled**             | **consumption_factor** (_float_): Negative consumption factor<br>**lin_vel_scalar** (_float_): Scalar for the linear velocity<br>**ang_vel_scalar** (_float_): Scalar for the angular velocity                                                                                                                                                                                                                                         | Calculates the reward based on the distance travelled.             |
| [RewardApproachGlobalplan](https://github.com/Arena-Rosnav/arena-rosnav/blob/5e87e3d9b59c72280d0b92e622abb4787643c20d/utils/misc/rl_utils/rl_utils/utils/rewards/reward_units/reward_units.py#L304)               | **approach_globalplan**            | **pos_factor** (_float_): Positive factor for approaching the goal<br>**neg_factor** (_float_): Negative factor for distancing from the goal                                                                                                                                                                                                                                                                                           | Calculates the reward when approaching the global plan.            |
| [RewardFollowGlobalplan](https://github.com/Arena-Rosnav/arena-rosnav/blob/5e87e3d9b59c72280d0b92e622abb4787643c20d/utils/misc/rl_utils/rl_utils/utils/rewards/reward_units/reward_units.py#L369)                 | **follow_globalplan**              | **min_dist_to_path** (_float_): The minimum distance to the global plan.<br>**reward_factor** (_float_): The reward factor to multiply the action by                                                                                                                                                                                                                                                                                   | Calculates the reward when following the global plan.              |
| [RewardReverseDrive](https://github.com/Arena-Rosnav/arena-rosnav/blob/5e87e3d9b59c72280d0b92e622abb4787643c20d/utils/misc/rl_utils/rl_utils/utils/rewards/reward_units/reward_units.py#L436)                     | **reverse_drive**                  | **reward** (_float_): The reward value for driving in reverse.                                                                                                                                                                                                                                                                                                                                                                         | Calculates the reward when driving in reverse.                     |
| [RewardAbruptVelocityChange](https://github.com/Arena-Rosnav/arena-rosnav/blob/5e87e3d9b59c72280d0b92e622abb4787643c20d/utils/misc/rl_utils/rl_utils/utils/rewards/reward_units/reward_units.py#L494)             | **abrupt_velocity_change**         | **vel_factors** (_Dict[str, float]_): Velocity factors for each dimension                                                                                                                                                                                                                                                                                                                                                              | Calculates the reward when there is an abrupt velocity change.     |
| [RewardRootVelocityDifference](https://github.com/Arena-Rosnav/arena-rosnav/blob/5e87e3d9b59c72280d0b92e622abb4787643c20d/utils/misc/rl_utils/rl_utils/utils/rewards/reward_units/reward_units.py#L557)           | **root_velocity_difference**       | **k** (_float_): The scaling factor for the velocity difference.                                                                                                                                                                                                                                                                                                                                                                       | Calculates the reward based on the root velocity difference.       |
| [RewardTwoFactorVelocityDifference](https://github.com/Arena-Rosnav/arena-rosnav/blob/5e87e3d9b59c72280d0b92e622abb4787643c20d/utils/misc/rl_utils/rl_utils/utils/rewards/reward_units/reward_units.py#L613)      | **two_factor_velocity_difference** | **alpha** (_float_): The weight for the squared difference in the first dimension of the action.<br>**beta** (_float_): The weight for the squared difference in the last dimension of the action.                                                                                                                                                                                                                                     | Calculates the reward based on the two-factor velocity difference. |
| [RewardActiveHeadingDirection](https://github.com/Arena-Rosnav/arena-rosnav/blob/5e87e3d9b59c72280d0b92e622abb4787643c20d/utils/misc/rl_utils/rl_utils/utils/rewards/reward_units/reward_units.py#L662C7-L662C35) | **active_heading_direction**       | **r_angle** (_float_): Weight for difference between max deviation of heading direction and desired heading direction.<br>**theta_m** (_float_): Maximum allowable deviation of the heading direction.<br>**theta_min** (_int_): Minimum allowable deviation of the heading direction.<br>**ped_min_dist** (_float_): Minimum distance to pedestrians.<br>**iters** (_int_): Number of iterations to find a reachable available theta. | Calculates the reward based on the active heading direction.       |

#### Reward Function Wrapper

The RewardFunction class is a wrapper for reward units. It provides the API for calculating the reward given the current state of the robot and the simulation.
It should be instantiated and utilized by the Gym environment. By passing different reward units to the wrapper, we can define different reward functions for the robot's behavior.

#### Reward Function Configuration

The reward function is configured using a YAML file. The configuration contains the registry name of the reward units and their parameters. The reward units are applied in the order they are listed in the configuration file.
When a parameter is not specified in the configuration file, the default value is used. The default values are defined in the [constants file](https://github.com/Arena-Rosnav/arena-rosnav/blob/5e87e3d9b59c72280d0b92e622abb4787643c20d/utils/misc/rl_utils/rl_utils/utils/rewards/constants.py).
Target directory for the configuration files is: **`../arena-rosnav/arena_bringup/configs/training/reward_functions`**.

The following is an example of a reward function configuration file:

```yaml
goal_reached:
  reward: 20

safe_distance:
  reward: -0.15

collision:
  reward: -15

approach_goal:
  pos_factor: 0.8
  neg_factor: 0.9
  _on_safe_dist_violation: true

reverse_drive:
  reward: -0.0001
  _on_safe_dist_violation: true

approach_globalplan:
  pos_factor: 0.1
  neg_factor: 0.15
  _on_safe_dist_violation: false

two_factor_velocity_difference:
  _on_safe_dist_violation: true

active_heading_direction:
  r_angle: 0.6
  _on_safe_dist_violation: true
```
