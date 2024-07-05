
# Collision Sensor

![Collision Sensor](../../images/Collision-Sensor-Arena-Unity.png)
<sub>3 Collision and Safe Dist Sensors Visually Represented by Spheres</sub>

### Overview
The `CollisionSensor` is an Arena Unity component integrated with ROS to detect collisions and manage safety distances for a robot. It is important for training DRL robot navigation agents.

### Functionality

- **Collision Detection**
    - Detects collisions between the robot and obstacles or pedestrians to prevent accidents during navigation.

- **Safety Distance Monitoring**
    - Configurable to detect specific safety distances for pedestrians and obstacles, enabling distinct handling in navigation logic and reward functions during DRL training.

### Configuration
- **Collision Detection**:
    - The configuration of the collision sensor resides in the `unity_params.yaml` file in the `arena-simulation-setup`repo which is unique to every robot.
        - Use the jackal `unity_params.yaml` configuration as an example if you want to extend to other robots.
    - Example:
```yaml
components:
  collider:
    type: ColliderSensor
    height: 0.92
    radius: 0.32
    position: [0, 0, 0]
```

- **Safety Distance Monitoring**:
    - A safety distance sensor can be attached via the `unity/attach_safe_dist_sensor` service which is provided by Arena Unity.
    - Note that the safety distance sensor (collsions sensor) is configured based on the existing collision sensor for normal collision detection.
        - If there is no normal collision sensor, no safety distance sensor can be attached.
    - Configure the safety distance sensor to react to pedestrians or obstacles by setting the respective flag (ped_safe_dist and obs_safe_dist) to true in the request.
        - Both flags can be set to true.
    - Service request format:
```
string robot_name
string safe_dist_topic
bool ped_safe_dist
bool obs_safe_dist
float64 safe_dist
```

### Implementation
- **Attachment and Configuration**
    - **Normal Collisions Detection:**
        - The `HandleCollider` method:
            - Creates a `CollisionSensor` object.
            - Attaches a `CapsuleCollider` set as a trigger.
            - Configures the collider using a provided dictionary.
            - Attaches the sensor to the robot.
    - **Safety Distance Detection**
        - The `HandleAttachSafeDistSensor` method configures and attaches a safety distance sensor to the robot based on the request parameters.
        - The collision sensor is configured based on the existing collision sensor for normal collision detection.
    - The collider is configured using the `ConfigureCollider` method, which uses the configuration from `unity_params.yaml` with values for `height`, `radius`, and `position`.

- **Initialization**
    - The `CollisionSensor` class initializes by establishing a ROS connection and registering a publisher for collision messages in the `Start` method.

- **Collision Detection**
    - **OnTriggerEnter**: Increments the collision count when an object with a designated layer ("Ped" for pedestrians or "Obs" for obstacles) enters the collider.
    - **OnTriggerExit**: Decrements the collision count when an object exits the collider.

- **Periodic Publishing**
    - The `Update` method periodically publishes collision messages based on a defined publishing rate.

- **Publishing Messages**
    - The `PublishMessage` method sends a `CollisionMsg` to the ROS topic, tracking continued contact time and resetting the collision count if necessary.

- **Collision Verification**
    - The `ActuallyInContact` method verifies the presence of collisions, ensuring the accuracy of the collision count.
    - This method is periodically executed to increase stability during training because during very high load, the counter might become incorect.

- **Configuration**
    - The `ConfigureCollider` method configures the collider's height, radius, and position using a provided dictionary.
