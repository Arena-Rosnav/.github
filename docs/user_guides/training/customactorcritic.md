# Actor-Critic Policies in Proximal Policy Optimization (PPO)

Actor-critic policies are a class of reinforcement learning algorithms that combine the strengths of both policy-based and value-based methods. In PPO, the actor-critic architecture is used to learn a policy that maximizes the expected cumulative reward.

The actor component of the policy, also known as the policy network, is responsible for selecting actions based on the current state. It takes the state as input and outputs a probability distribution over the available actions. The critic component, also known as the value network, estimates the value function, which represents the expected cumulative reward from a given state.

PPO is an algorithm that aims to optimize the policy by iteratively updating the actor and critic networks. It uses a surrogate objective function to ensure that the policy updates are performed in a stable and conservative manner. PPO has been shown to achieve state-of-the-art performance in various reinforcement learning tasks.

To implement actor-critic policies in PPO, you would typically define the actor and critic networks, initialize them with random weights, and then use gradient-based optimization techniques to update the weights based on the observed rewards and states.
Overall, actor-critic policies in PPO provide a powerful framework for learning policies that can effectively balance exploration and exploitation in reinforcement learning tasks.

Conveniently, the `stable-baselines3` library provides a high-level implementation of PPO that allows you to easily train and evaluate actor-critic policies in various environments. The library also includes pre-defined actor and critic networks that can be used out-of-the-box or customized to suit your specific needs.

## Custom Actor-Critic Networks in Arena-Rosnav

The Rosnav-RL repository provides a `BaseAgent` class that serves as a base class for defining an Actor-Critic-Agent in a reinforcement learning environment.
The `BaseAgent` class includes several properties and methods for specifying the architecture of the actor and critic networks, as well as the observation space and other relevant parameters concerning the agent.

Essentially, the `BaseAgent` class provides a convenient interface for defining custom actor-critic networks in the Arena framework. The resulting agent is then parsed to the `PPO` algorithm in the `stable-baselines3`.

There are several properties that need to be implemented in a custom agent class, such as `observation_spaces`, `type`, `features_extractor_class`, `features_extractor_kwargs`, `net_arch`, and `activation_fn`. These properties and methods allow you to specify the architecture of the actor and critic networks,
as well as the observation space and other relevant parameters regarding the agent.

## BaseAgent Properties

### Agent Registry

When defining a custom agent, you need to register it in the agent factory using the `@AgentFactory.register` decorator. This allows the agent to be instantiated by the agent factory during training initialization.

#### `space_encoder_class`

- **Description**:
  The class of the space encoder used by the agent.
- **Type**:
  `Type[BaseSpaceEncoder]`
- **Optional**:
  Yes
- **Default**:
  DefaultEncoder

#### `observation_spaces`

- **Description**:
  List of observation space units which are required for the agent.
- **Type**:
  `List[BaseObservationSpace]`
- **Optional**:
  No
- **Available Values**:
  [Click here](https://github.com/Arena-Rosnav/rosnav-rl/tree/dev/rosnav/utils/observation_space/spaces)

#### `observation_space_kwargs`

- **Description**:
  The keyword arguments for the observation spaces.
- **Type**:
  `Dict[str, Any]`
- **Optional**:
  Yes (Default values are used)

#### `type`

- **Description**:
  The type of the agent.
- **Type**:
  `PolicyType`
- **Optional**:
  No
- **Available Values**:
  [Click here](https://github.com/Arena-Rosnav/rosnav-rl/blob/6242cb60b3c51a80beb1b2ac672d8944b2371959/rosnav/model/constants.py#L11)

#### `features_extractor_class`

- **Description**:
  The class of the features extractor used by the agent.
- **Type**:
  `Type[BaseFeaturesExtractor]`
- **Optional**:
  Yes
- **Default**:
  `NatureCNN` (for `CnnPolicy`)
- **Available Values**:
  [Click here](https://github.com/Arena-Rosnav/rosnav-rl/tree/dev/rosnav/model/feature_extractors)

#### `features_extractor_kwargs`

- **Description**:
  The keyword arguments for the features extractor.
- **Type**:
  `Dict[str, Any]`
- **Optional**:
  Yes (Default values are used)
- **Default**:
  `None`

#### `net_arch`

- **Description**:
  The architecture of the actor and critic. Most commonly, it is a list of integers, where each integer represents the number of units in a hidden layer of the MLP.
- **Type**:
  `List[Union[int, Dict[str, Any]]]`
- **Optional**:
  Yes
- **Default**:
  `None` (Just the output layer for the actor and critic networks)

#### `activation_fn`

- **Description**:
  The activation function used in the actor and critic networks.
- **Type**:
  `nn.Module`
- **Optional**:
  Yes
- **Default**:
  `nn.tanh`

### Custom Agent Example

```python
@AgentFactory.register("RosnavResNet_6")
class RosnavResNet_6(BaseAgent):
    """
    Custom policy class using ResNet-based CNN.

    Attributes:
        type (PolicyType): The type of the policy.
        space_encoder_class (class): The class for encoding the observation space.
        observation_spaces (list): The list of observation spaces.
        observation_space_kwargs (dict): The keyword arguments for the observation space.
        features_extractor_class (class): The class for extracting features.
        features_extractor_kwargs (dict): The keyword arguments for the features extractor.
        net_arch (list): The architecture of the neural network.
        activation_fn (function): The activation function used in the neural network.
    """

    type = PolicyType.CNN
    space_encoder_class = DefaultEncoder
    observation_spaces = [
        SPACE_INDEX.STACKED_LASER_MAP,
        SPACE_INDEX.PEDESTRIAN_VEL_X,
        SPACE_INDEX.PEDESTRIAN_VEL_Y,
        SPACE_INDEX.PEDESTRIAN_LOCATION,
        SPACE_INDEX.PEDESTRIAN_TYPE,
        SPACE_INDEX.GOAL,
        SPACE_INDEX.LAST_ACTION,
    ]
    observation_space_kwargs = {
        "roi_in_m": 30,
        "feature_map_size": 80,
        "laser_stack_size": 10,
    }
    features_extractor_class = RESNET_MID_FUSION_EXTRACTOR_6
    features_extractor_kwargs = {
        "features_dim": 256,
        "width_per_group": 128,
    }
    net_arch = dict(pi=[256, 64], vf=[256])
    activation_fn = nn.ReLU
```
