
# Pedestrian Integration

## Character Models
For visualizing pedestrians in Unity, we make use of **two character models** as of January 2024, a female and a male character. The respective character models are taken from Mixamo.org, which offer free character models for any use. We used those character models to create our own blueprints (called prefabs in Unity) that have added functionality (e.g. collider functionality, animation component, etc.). The blueprints are then spawned by the *PedController* whenever a spawn request handled.

## Social States and Animations
### Social States in Unity
The **social state** changes and corresponding animations are handled in the *PedController*. It is subscribed to the */pedsim_simulator/simulated_agents* topic via the ROSTCPConnector and has a callback function that handles every publish on this topic. The Controller then accesses the animator component of each pedestrian to set the social state number according to the social state that is published.
### Velocity in Unity
The **velocity** of each pedestrian also plays a crucial role in determining the animation. It is calculated by the PedController from the linear velocity that is published for every pedestrian. After calculating the forward speed, the Controller sets the velocity in the Animator Component for the corresponding pedestrian.

### Animator Component
The **animator component** is part of every pedestrian game object in Unity. It defines the transitions that can occur between every animation and what animation is played at the moment. It does so primarly by watching the velocity and social_state variables, also called *parameters*, for changes. Based on the value of those parameters it then triggers the corresponding animation.  
The supported animations (as of January 2024) are the following:  
1. Idle  
2. Walking/Running (animation speed proportional to velocity)  
3. Talking/Listening  
4. Texting  
5. Interested (currently named LookAround)  
6. Talking on the Phone (actual phone will be added shortly)  
