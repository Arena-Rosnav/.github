
## Rosnav - ROS Navigation

**Rosnav** is the official package for ROS Navigation. It includes pretrained neural networks for collision avoidance and infrastructure to run these networks within a ROS context. Rosnav is primarily designed for the **arena-rosnav** environment, where it is both trained and evaluated in 2D and 3D variants.

### Structure

The Rosnav package encompasses multiple pretrained neural networks for various robots, along with encoders for easy integration into existing systems. Note that it does not provide the full infrastructure for training models; **arena-rosnav** can be used for that purpose.

### Architectures

Two primary architectures are available:

1. **Robot Specific Architecture**: Each robot has its own dedicated network with distinct observation and action spaces.

   - *Observation Space*: Combines current laser scan, goal distance and angle, and previous action.
   - *Action Space*: 1D array with two or three elements, depending on robot holonomy.

2. **Unified Architecture** (Work in Progress): A single model can run on multiple robots with a standardized observation space.

   - *Observation Space*: Includes manipulated laser scan, goal information, previous action, and velocity limits.
   - *Action Space*: A 1D array with three entries in the interval [-1, 1].

### Rosnav Space Encoder

The **Rosnav Space Encoder** serves as middleware to encode observations for the different architectures, supporting both robot-specific and unified spaces. It provides functions to manage observation and action spaces, decode actions, and encode observations.

### Rosnav Space Manager

For simplicity, a separate **Rosnav Space Manager** is available. It instantiates the encoder and selects the appropriate encoder based on the `space_encoder` parameter.

### Usage

#### For Training

- Integrate Rosnav Space Manager in your training environment to manage action and observation spaces and encode/decode observations and actions as needed.

#### For Testing

- A provided node offers a service call (`rosnav/get_action`) to obtain the next action from a given observation. The service requires observations as described in the parameters mentioned.

### Required Parameters

- Various parameters must be set for the Rosnav Space Encoder to work correctly, including laser specifications, robot radius, encoder selection, action space type, and action definitions.

---
