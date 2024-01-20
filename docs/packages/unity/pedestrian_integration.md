
# Pedestrian Integration

## Character Models
For visualizing pedestrians in Unity, we make use of **two character models** as of January 2024, a female and a male character. The respective character models are taken from Mixamo.org, which offer free character models for any use. We used those character models to create our own blueprints (called prefabs in Unity) that have added functionality (e.g. collider functionality, animation component, etc.). The blueprints are then spawned by the *PedController* whenever a spawn request handled.

## Social States and Animations
### Social States in Unity
The **social state** changes and corresponding animations are handled in the *PedController*. It is subscribed to the */pedsim_simulator/simulated_agents* topic via the ROSTCPConnector and has a callback function that handles every publish on this topic. The Controller then accesses the animator component of each pedestrian to set the social state number according to the social state that is published.

### Animator Component
The **animator component** is part of every pedestrian game object in Unity. It defines the transitions that can occur between every animation and what animation is played at the moment. It does so primarly by watching the social_state variable, also called *parameter*, for changes. Based on the value of this parameter it then triggers an animation.

