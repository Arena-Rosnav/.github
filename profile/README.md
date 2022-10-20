# Arena Benchmark

[Arena Benchmark](https://github.com/Arena-Rosnav/arena-bench) is a set of tools for training and evaluating of navigational algorithms for obstacle avoidance. Arena Benchmark is modular and flexible and allows to easily integrate new environments and planners, as well as running existing planners and models in predefined benchmark scenarios and evaluating the performance with ease. Next to the planners and environments we offer a wide variety of different robots and worlds for your use.

Arena Benchmark offers a complete evaluation pipeline for benchmarking the performance of robots and planners based on standard metrics, and a trainings pipeline for navigational models based on DRL and PPO. With this pipeline our own DRL planner [ROSNav](packages/rosnav.md) was created.

> Please refer to the official documentation to find detailed instructions on how to install and use the _Arena Benchmark_ project. You can find the documentation by following [this](#TODO) link.

### Features

- Integration of [Flatland](#TODO) for training new agents and [Flatland]() and [Gazebo]() for evaluating existing approaches.
- Variety of planners, robots and worlds
- Pipeline for training planner agents based on reinforcement learning approaches from [stable baselines3](https://github.com/DLR-RM/stable-baselines3.git)
- [Task manager]() for managing highly dynamic and custom environments.
- Our own DRL planner [ROSNav]()
- Pipeline for evaluating approaches and analysing them based on standard metrics with our [Arena Evaluation](packages/arena_evaluation.md) package.
- Modular structure for extension of new functionalities and approaches
- Evaluation of multiple robots and planners in the same simulation

|         Multiple agents in one simulation         |             Random task mode with one robot             |                        Simulation in Gazebo                        |
| :-----------------------------------------------: | :-----------------------------------------------------: | :----------------------------------------------------------------: |
| <img width="250" src="docs/images/gifs/marl.gif"> | <img width="250" src="docs/imagesgifs/random_task.gif"> | <img width="250" src="docs/images/gifs/random-mode-warehouse.gif"> |

## Supported Robots

|                       _turtlebot3-burger_                       |                       _jackal_                       |                  _ridgeback_                  |                 _agv-ota_                  |                 _tiago_                  |
| :-------------------------------------------------------------: | :--------------------------------------------------: | :-------------------------------------------: | :----------------------------------------: | :--------------------------------------: |
| <img width="250" src="docs/imagesrobots/turtlebot3-burger.jpg"> | <img width="250" src="docs/imagesrobots/jackal.jpg"> | <img width="250"  src="robots/ridgeback.jpg"> | <img width="250" src="robots/agv-ota.png"> | <img width="250" src="robots/tiago.jpg"> |

|                  _Robotino(rto)_                  |                       _youbot_                       |                       _turtlebot3_waffle_pi_                        |                _Car-O-Bot4 (cob4)_                 |                       _dingo_                       |
| :-----------------------------------------------: | :--------------------------------------------------: | :-----------------------------------------------------------------: | :------------------------------------------------: | :-------------------------------------------------: |
| <img width="250" src="docs/imagesrobots/rto.jpg"> | <img width="250" src="docs/imagesrobots/youbot.jpg"> | <img width="250"  src="docs/imagesrobots/turtlebot3_waffle_pi.jpg"> | <img width="250" src="docs/imagesrobots/cob4.jpg"> | <img width="250" src="docs/imagesrobots/dingo.jpg"> |

## Recent Publications

- [Arena-Bench](https://arxiv.org/abs/2206.05728): A Benchmarking Suite for Obstacle Avoidance Approaches in Highly Dynamic Environments
- [Arena-Rosnav](https://ieeexplore.ieee.org/document/9636226/authors#authors): Towards Deployment of Deep-Reinforcement-Learning-Based Obstacle Avoidance into Conventional Autonomous Navigation Systems
- [All-in-One](https://ieeexplore.ieee.org/document/9811797): A DRL-based Control Switch Combining State-of-the-art Navigation Planners
