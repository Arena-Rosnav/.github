

# Arena Unity Training Integration

This page includes information about the integration of Arena Unity into the training pipeline, including feature overviews and technical details. If you want to use Arena Unity for training, refer to this [guide]().  
Arena Rosnav previously only supported training of deep reinforcement learning agents in the Flatland simulator.  
Arena Rosnav has a sophisticated training pipeline in place (its main documentation can be found [here](user_guides/training/training.md)). To integrate Arena Unity as an additional simulator for training DRL agents we needed to:

- Extend the existing training pipeline by 'Arena Unity'-specific features, like 'Arena Unity'-specific observations and reward functions 
- Create a completely new training environment for the training pipeline to interact with Arena Unity
- Find a way to synchronize the training step rate with the Arena Unity continuous-time simulation
- Integrate the launch of Arena Unity in the training launch workflow 
- Create a new feature extractor network to support feature extraction of RGBD data 
- Integrate a "simulation speed" variable to fully leverage compute ressources

![Arena Unity Training Integration](../../images/Arena-Unity-Training.drawio.png)
<sub>Area Unity Training Integration Graphic</sub>  
This image shows all completely 'Arena Unity'-specific components (in green) within the Arena Rosnav DRL training pipeline.

## 'Arena Unity'-specific Observations & Rewards

#### Image
![RGBD Image Visualization](../../images/RGBD-camera-Arena-Unity.png)
<sub>RGB and Depth Image Visualization in Rviz.</sub>  
![Depth Image Zoom-In](../../images/depth-image-Arena-Unity.png)
<sub>Same image as above but zoom-in on depth image.</sub>  
Most important, we implemented a RGBD sensor in Arena Unity to leverage the photo-realsim of Unity with its HD-Renderpipeline. RGB and depth act as independent observations and processing logic was implemented in the Arena Rosnav training pipeline.

#### Enhanced Collision Detection
Based on our Arena Unity CollisionSensor (see the CollisionSensor documentation [here]()), we are able to efficiently and accurately detect collisions. During training, we previously relied on an additional designated laser scan to detect whether a collision of the robot with another object took place. Now we can accurately and efficiently detect collisions with the Unity collider component in the back-end. This observation acts as an enhancement to previous collision logic.

#### Pedestrian-specific Safety Distance
A new observation which can detect safety distance violations specifically with pedestrians. This is also based on the Arena Unity CollisionSensor. We added a reward function specifically for pedestrian safety distance where reward value and safety distance can be set.

#### Obstacle-specific Safety Distance
Similiar to pedestrian-specific safety distance, obstacle-specific safety distance can be set to a certain distance and observations will be generated accordingly. It also uses the Arena Unity CollisionSensor and is used for a new corresponding reward.