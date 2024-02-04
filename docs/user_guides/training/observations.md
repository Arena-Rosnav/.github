# Observation Spaces in Reinforcement Learning for Robotic Navigation

In reinforcement learning, observation spaces play a crucial role in robotic navigation tasks. An observation space represents the information that an agent perceives from its environment. It provides the necessary input for the agent to make decisions and take actions.

In the context of robotic navigation, observation spaces typically include sensor data such as camera images, lidar scans, or proprioceptive information from the robot's sensors. These observations capture the state of the environment and enable the agent to understand its surroundings.

The choice of observation space is critical as it directly impacts the agent's ability to learn and navigate effectively. A well-designed observation space should provide sufficient information for the agent to perceive relevant features of the environment, while also being compact and computationally efficient.

By carefully designing the observation space, reinforcement learning agents can effectively perceive and understand their environment, leading to improved navigation performance in robotic systems.

# Observations in Arena-Rosnav

## Observation Spaces and Observation Encoding

### Data Flow

To provide data to the observation spaces in the Arena-Rosnav framework, the simulation and its plugins are responsible for supplying the necessary information. If there is a need to introduce new observation spaces, it may be necessary to introduce new data types. This can involve modifying existing code or creating new plugins that can supply the new information effectively. This ensures that the observation spaces have access to all the relevant information from the environment for effective learning and navigation.

In order to collect the data from the simulation, we introduced the `ObservationManager` that wraps all the observation collector entities, in our framework called `ObservationUnits`. The `ObservationUnits` are responsible for collecting the data from the simulation and passing it to the `ObservationManager`.
The observation dictionary is the central container that stores all the information that the agent needs to perceive from the environment. It is passed to the `RewardFunction` and eventually to the observation encoder infrastructure to encode and concatenate the data into model compliant format.

### ObservationSpace Classes

In the Arena-Rosnav framework, we use the gym library to define observation spaces of the DRL agent. The `gym` library provides a flexible and standardized way to define observation spaces, making it easy to integrate with reinforcement learning algorithms and libraries.
Now, in order to define the observation spaces, we have created a base class `BaseObservationSpace` that provides a standardized interface for defining observation spaces. This base class is then extended to create specific observation spaces for example for different types of sensor data, such as camera images, lidar scans, or proprioceptive information.
Moreover, the observation spaces are responsible for encoding dedicated information from the observation dictionary into a format that is compatible with the reinforcement learning model. This involves for example converting the assigned observations into a numpy array that can be used as input to the model.

```python
class BaseObservationSpace(ABC):
    """
    Base class for defining observation spaces in reinforcement learning environments.
    """

    def __init__(self, *args, **kwargs) -> None:
        self._space = self.get_gym_space()

    @property
    def space(self) -> spaces.Space:
        """
        Get the gym.Space object representing the observation space.
        """
        return self._space

    @property
    def shape(self):
        """
        Get the shape of the observation space.
        """
        return self._space.shape

    @abstractmethod
    def get_gym_space(self) -> spaces.Space:
        """
        Abstract method to define and return the gym.Space object representing the observation space.
        """
        raise NotImplementedError()

    @abstractmethod
    def encode_observation(self, observation: dict, *args, **kwargs) -> np.ndarray:
        """
        Abstract method to encode the observation into a numpy array.
        """
        raise NotImplementedError()

```

[Click here to see a list of the implemented observation spaces.](https://github.com/Arena-Rosnav/rosnav-rl/tree/52a86833bd807c9db3040dfc8e5fd633a3243281/rosnav/utils/observation_space/spaces)

### Observation Encoding

```python

class ObservationSpaceManager:
    """
    A class that manages the observation spaces for a reinforcement learning agent.

    Args:
        space_list (List[Union[str, SPACE_INDEX]]): A list of space names or space indices.
        space_kwargs (Dict[str, Any]): Keyword arguments for configuring the observation spaces.
        frame_stacking (bool, optional): Whether to enable frame stacking. Defaults to False.
        flatten (bool, optional): Whether to flatten the observation spaces. Defaults to True.

    Attributes:
        space_list: The list of space names or space indices.
        observation_space: The combined observation space of all the individual spaces.

    Methods:
        add_observation_space: Add a new observation space to the manager.
        add_multiple_observation_spaces: Add multiple observation spaces to the manager.
        encode_observation: Encode an observation into a numpy array.
        get_space_container: Get the space container for a specific space.
        get_space_index: Get the space index for a specific space name.

    """

    def __init__(
        self,
        space_list: List[Union[str, SPACE_INDEX]],
        space_kwargs: Dict[str, Any],
        frame_stacking: bool = False,
        flatten: bool = True,
    ) -> None:
        ...

    ...

    def encode_observation(self, observation: dict, *args, **kwargs) -> np.ndarray:
        """
        Encode an observation into a numpy array.

        Args:
            observation (dict): The observation to encode.
            *args: Additional positional arguments.
            **kwargs: Additional keyword arguments.

        Returns:
            np.ndarray: The encoded observation.

        """
        _concatenated = np.concatenate(
            [
                self._space_containers[space.name].encode_observation(
                    observation, **kwargs
                )
                for space in self._spacelist
            ],
        )
        return (
            _concatenated
            if not self._frame_stacking
            else np.expand_dims(_concatenated, axis=0)
        )

```

The observation space classes are also responsible for encoding specific information from the observation dictionary into a format that is compatible with the reinforcement learning model. Given the specific parameters of the observation space,
the resulting encoded observation should comply with the constructed `gym` observation space. For example, the returned numpy array needs to have the same shape as the observation space defined as `gym` space.

As with the `ObservationManager`, we also introduced the `ObservationSpaceManager` that wraps all the observation spaces and provides a standardized interface for encoding observations. The `ObservationSpaceManager` is responsible for combining
the individual observation spaces into a single observation space and encoding the observations into a single numpy array that can be used as input to the model.
As per default, all the encoded observations are flattened and then optionally restructured on the model side (e.g. in the forward pass method of the model).

### Workflow when Introducing New Observation Spaces

When introducing new observation spaces, the following steps should be taken:

- Make sure, that the necessary data is provided by the simulation.
- Create a new `ObservationUnit` [<span style="font-size:0.5em;">[Source Code]</span>](https://github.com/Arena-Rosnav/arena-rosnav/blob/bd41d4cebdcd3504b6cbc1111bcaaaa3ec351a20/utils/misc/rl_utils/rl_utils/utils/observation_collector/observation_units/collector_unit.py) retrieving that new particular observation and adding it to the observation dictionary. Add the observation unit to the `ObservationManager` [<span style="font-size:0.5em;">[Source Code]</span>](https://github.com/Arena-Rosnav/arena-rosnav/blob/master/utils/misc/rl_utils/rl_utils/utils/observation_collector/observation_manager.py).
- Create a new observation space class that extends the `BaseObservationSpace` [<span style="font-size:0.5em;">[Source Code]</span>](https://github.com/Arena-Rosnav/rosnav-rl/blob/dev/rosnav/utils/observation_space/spaces/base_observation_space.py) class.
  - Implement the `get_gym_space` method to define the observation space using the `gym` library.
  - Implement the `encode_observation` method to encode the observation into a numpy array with intended content.
- Make sure that the `RosnavSpaceManager`[<span style="font-size:0.5em;">[Source Code]</span>](https://github.com/Arena-Rosnav/rosnav-rl/blob/dev/rosnav/rosnav_space_manager/rosnav_space_manager.py) retrieves the necessary agent information for your new observation space and provides it to the encoder class.
- When specifying a StableBaselines3 model, add the new observation space class to the `observation_space` parameter of the model.
