# Planners

Planners, often used instead of local planners, are the planning algorithms that control the navigation of the mobile robot. Local planning deals with obstacle-avoidance while following the precalculated global path. Each planner is maintained in its own repository.

We offer the among others following DRL planners:

- **Rosnav**: Our self-developed DRL planner. [Paper](https://arxiv.org/abs/2104.03616)
- **TRAIL**: DRL approach with velocity obstacles for the Jackal. 3rd place at the BARN challenge, created at [TRAIL Lab](https://sites.google.com/site/zhantengxie/home?authuser=0).
- **APPLR**: RL policy for online adjustment of navigation parameters of DWA for the Jackal. Produced by [UTexas](https://www.cs.utexas.edu/~xiao/Research/APPL/APPL.html#applr)
- **RLCA-ROS**: RL navigation policy trained on and intended for multi-agent systems. [Paper](https://arxiv.org/abs/1709.10082).
- **CADRL**: DEEPRL policy, works good across different robots, given that external information about environment's obstacles can be provided. [Paper](https://arxiv.org/abs/1805.01956).
- **SARL-Star**: Pedestrian-aware approach for indoor environments. [Paper](https://ieeexplore.ieee.org/abstract/document/8961764)
- **Crowdnav-ROS**: Used for mobile navigation in crowds by predicting the natural human behavior. [Paper](https://arxiv.org/abs/1809.08835)

As well as the following non-DRL planners:

- **Dragon**: Finds path by evaluating the gaps between obstacles in the environment. Suitable for static scenarios. Developed by [AMRLab](https://www.bezzorobotics.com/) for the BARN challenge.
- **TEB**
- **DWA**
- **MPC**

!!! note
    All available planners can be found in ```../catkin_ws/src/planners/```.