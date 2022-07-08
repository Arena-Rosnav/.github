# Introduction
Arena-rosnav is a flexible, high-performance 2D simulator with configurable agents, multiple sensors, and benchmark scenarios for testing robotic navigation. Arena-Rosnav uses [Flatland](https://github.com/avidbots/flatland) as the core simulator and is a modular high-level library for end-to-end experiments in embodied AI -- defining embodied AI tasks (e.g. navigation, obstacle avoidance, behavior cloning), training agents (via imitation or reinforcement learning, or no learning at all using conventional approaches like DWA, TEB or MPC), and benchmarking their performance on the defined tasks using standard metrics.

### What is arena-rosnav for?
Train DRL agents on ROS compatible simulations for autonomous navigation in highly dynamic environments. Flatland-DRL integration is inspired by Ronja Gueldenring's work: [drl_local_planner_ros_stable_baselines](https://github.com/RGring/drl_local_planner_ros_stable_baselines.git). Test state of the art local and global planners in ROS environments both in simulation and on real hardware. Following features are included:

- Setup to train a local planner with reinforcement learning approaches from [stable baselines3](https://github.com/DLR-RM/stable-baselines3.git)
- Training in simulator [Flatland](https://github.com/avidbots/flatland) in train mode
- Include realistic behavior patterns and semantic states of obstacles (speaking, running, etc.)
- Include different obstacles classes (other robots, vehicles, types of persons, etc.)
- Implementation of intermediate planner classes to combine local DRL planner with global map-based planning of ROS Navigation stack
- Testing a variety of planners (learning based and model based) within specific scenarios in test mode
- Modular structure for extension of new functionalities and approaches