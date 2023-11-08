# DRL Training

<!--
- How to implement NN
- How to implement RF
 -->

- [DRL Training](#drl-training)
  - [Features](#features)
  - [Start Training](#start-training)
    - [Simulation Environment](#simulation-environment)
    - [Training Script](#training-script)
    - [Visualization](#visualization)
  - [Components](#components)
  - [Config Parameters](#config-parameters)
    - [General](#general)
    - [Training Monitoring](#training-monitoring)
    - [Reinforcement Learning Agent](#reinforcement-learning-agent)
      - [Neural Networks](#neural-networks)
      - [Observation Space](#observation-space)
        - [Laser](#laser)
          - [Full Angle Laser](#full-angle-laser)
          - [Reduced Number of Laser Beams](#reduced-number-of-laser-beams)
        - [Frame Stacking](#frame-stacking)
      - [Action Space](#action-space)
        - [Custom Action Space Discretization](#custom-action-space-discretization)
      - [Reward Functions](#reward-functions)
      - [Proximal Policy Optimizaton](#proximal-policy-optimizaton)
    - [Callbacks](#callbacks)
      - [Periodic Evaluation](#periodic-evaluation)
      - [Training Curriculum](#training-curriculum)
      - [Stop Training Mechanism](#stop-training-mechanism)
    - [Example Config](#example-config)

## Features

- Simple handling of the training through a single config file
- Choose between different robot models
- Define your own DNN or choose between predefined
- Networks will be trained, evaluated and saved
- Load your trained agent to continue training
- Optionally log training and evaluation data
- Enable and modify a custom training curriculum
- Multiprocessed rollout collection for training with debug mode

## Start Training

To start a training procedure you need at least two terminals.

> Note: Don't forget to source the catkin workspace and activating the poetry environment before prompting these commands.

### Simulation Environment

**Training Simulation Launch Arguments**:

- _model_: Robot model
- _num_envs_: Number of parallel environments
- _map_folder_name_: Name of the map

```bash
roslaunch arena_bringup start_training.launch model:=jackal num_envs:=1 map_folder_name:=map_empty
```

### Training Script

**Training Script Arguments**:

- _config_: Name of the config file (has to be in the subdirectory "_arena-rosnav/training/configs_").

```bash
# navigate to the arena-rosnav directory
cd arena-rosnav
python training/scripts/train_agent.py
```

### Visualization

**Training Visualization Arguments**:

- _ns_: Namespace of the simulation instance to be visualized.
- _rviz_file_: Name of the rviz file "_arena_bringup/rviz_". Defaults to "nav_LP".

```bash
roslaunch arena_bringup visualization_training.launch ns:=sim_1
```

## Components

![components](../images/system-design/training-sys.jpg)

## Config Parameters

<!------------------------------------------------------------>

### General

| Parameter             | Type  | Description                                                                                                |
| --------------------- | ----- | ---------------------------------------------------------------------------------------------------------- |
| debug_mode            | bool  | Debug mode simulates the multiprocessing in a single process and thus allows for clearer error traceback.  |
| n_envs                | int   | Number of environments to collect rollouts from.                                                           |
| no_gpu                | bool  | Don't use the gpu.                                                                                         |
| task_mode             | str   | Name of the navigation task mode. An overview of the task modes can be found [here](planners_overview.md). |
| n_timesteps           | int   | Number of timesteps to be simulated. A timestep is regarded as an environment step.                        |
| max_num_moves_per_eps | int   | Maximum number of steps per episode.                                                                       |
| goal_radius           | float | Radius of the navigation goal.                                                                             |

<!------------------------------------------------------------>

### Training Monitoring

| Parameter | Type | Description                                                              |
| --------- | ---- | ------------------------------------------------------------------------ |
| use_wandb | bool | Enables Weights and Biases logging.                                      |
| eval_log  | bool | Saves periodic evaluation statistics during training in designated file. |

<!------------------------------------------------------------>

### Reinforcement Learning Agent

#### Neural Networks

| Parameter         | Type | Description                                                                                                         |
| ----------------- | ---- | ------------------------------------------------------------------------------------------------------------------- |
| architecture_name | str  | Name of the predefined neural network architecture according to the network registry.                               |
| resume            | str  | Name of the agent to resume training with. The agent has to be located in the "_planners/rosnav/agents_" directory. |

> For a tutorial how to implement new neural network architectures refer to [here](planners_overview.md).

#### Observation Space

| Parameter | Type | Description                                                                                                                                                                                                                 |
| --------- | ---- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| normalize | bool | Enables the moving average normalization for the observations. Technical details can be found [here](https://stable-baselines3.readthedocs.io/en/master/guide/vec_envs.html#stable_baselines3.common.vec_env.VecNormalize). |

##### Laser

###### Full Angle Laser

| Parameter        | Type | Description                                                                                          |
| ---------------- | ---- | ---------------------------------------------------------------------------------------------------- |
| full_range_laser | bool | Enables an additional laser covering 360° in order to detect collisions in blind spots of the robot. |

###### Reduced Number of Laser Beams

| Parameter | Type | Description                                     |
| --------- | ---- | ----------------------------------------------- |
| enabled   | bool | Enables the reduction of number of laser beams. |
| num_beams | int  | Desired number of laser beams.                  |

> Note: Laser is downsampled by keeping only a subset of size _num_beams_ of its elements that are evenly distributed along its length.

##### Frame Stacking

| Parameter  | Type | Description                                                                                                                                                                                                                                                                                                                                          |
| ---------- | ---- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| enabled    | bool | Enables frame stacking. Further technical details can be found [here](https://stable-baselines3.readthedocs.io/en/master/guide/vec_envs.html#stable_baselines3.common.vec_env.stacked_observations.StackedObservations). The time period of a frame is equivalent to an environment step. The latter is determined by _step-size_ of the simulation. |
| stack_size | int  | Number of frames to be stacked.                                                                                                                                                                                                                                                                                                                      |

#### Action Space

| Parameter | Type | Description                          |
| --------- | ---- | ------------------------------------ |
| discrete  | bool | Discrete actions will be considered. |

##### Custom Action Space Discretization

| Parameter           | Type | Description                                                                                                       |
| ------------------- | ---- | ----------------------------------------------------------------------------------------------------------------- |
| enabled             | bool | Enables the custom discretization of the action space.                                                            |
| buckets_linear_vel  | int  | Number of buckets to divide the linear velocity in. The continuous linear velocity range is taken as reference.   |
| buckets_angular_vel | int  | Number of buckets to divide the angular velocity in. The continuous angular velocity range is taken as reference. |

> Note:
>
> - Eventually, there are a combination of _buckets_linear_vel \* buckets_angular_vel_ discrete actions for the agent.
> - Only non-holonomic robots (with 2D action space) are supported for custom action space discretization currently.

#### Reward Functions

| Parameter  | Type | Description                                                                                                             |
| ---------- | ---- | ----------------------------------------------------------------------------------------------------------------------- |
| reward_fnc | str  | Name of the file containing the reward function. File has to lie in "_arena-rosnav/training/configs/reward_functions_". |

> For a tutorial how to implement new reward functions refer to [here](planners_overview.md).

#### Proximal Policy Optimizaton

| Parameter     | Type  | Description                                                                                                                                                                                    |
| ------------- | ----- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| batch_size    | int   | Batch size - number of experiences considered for parameter update.                                                                                                                            |
| gamma         | float | Discount factor                                                                                                                                                                                |
| n_steps       | int   | The number of steps to run for each environment per rollout collection (set automatically by training script depending on _batch_size_ and _n_envs_)                                           |
| ent_coef      | float | Entropy coefficient for the loss calculation                                                                                                                                                   |
| learning_rate | float | The learning rate, it can be a function of the current progress remaining (from 1 to 0) (i.e. batch size is n_steps and n_env where n_env is number of environment copies running in parallel) |
| vf_coef       | float | Value function coefficient for the loss calculation                                                                                                                                            |
| max_grad_norm | float | The maximum value for the gradient clipping                                                                                                                                                    |
| gae_lambda    | float | Factor for trade-off of bias vs variance for Generalized Advantage Estimator                                                                                                                   |
| m_batch_size  | int   | Mini batch size of the training                                                                                                                                                                |
| n_epochs      | int   | Number of epoch when optimizing the surrogate loss                                                                                                                                             |
| clip_range    | float | Clipping parameter, it can be a function of the current progress remaining (from 1 to 0).                                                                                                      |

<!------------------------------------------------------------>

### Callbacks

#### Periodic Evaluation

| Parameter             | Type | Description                                                      |
| --------------------- | ---- | ---------------------------------------------------------------- |
| max_num_moves_per_eps | int  | Maximum number of steps per evaluation episode.                  |
| n_eval_episodes       | int  | Number of episode per evaluation.                                |
| eval_freq             | int  | Periodic evaluation after every _n_envs \* eval_freq_ timesteps. |

#### Training Curriculum

| Parameter                | Type  | Description                                                                                                                                             |
| ------------------------ | ----- | ------------------------------------------------------------------------------------------------------------------------------------------------------- |
| training_curriculum_file | str   | Name of the training curriculum file. File has to be located at "_arena-rosnav/training/configs/training_curriculums_".                                 |
| curr_stage               | int   | Number of the current curriculum stage. For newly initialized agents, this depicts the start stage.                                                     |
| threshold_type           | str   | Threshold type to be considered at the end of each evaluation cycle for the training curriculum. Can be either "succ" (success rate) or "rew" (reward). |
| upper_threshold          | float | Upper threshold to be reached to trigger the next stage.                                                                                                |
| lower_threshold          | float | Lower threshold to be reached to trigger the previous stage.                                                                                            |

#### Stop Training Mechanism

| Parameter      | Type  | Description                                                                                                                                             |
| -------------- | ----- | ------------------------------------------------------------------------------------------------------------------------------------------------------- |
| threshold_type | str   | Threshold type to be considered at the end of each evaluation cycle for the training curriculum. Can be either "succ" (success rate) or "rew" (reward). |
| threshold      | float | Threshold to be reached in order to end the training. In staged mode this only applies when the last staged was reached.                                |

<!------------------------------------------------------------>

### Example Config

<details open>
<summary>Click to open</summary>

```yaml
### General
debug_mode: true
n_envs: 4
no_gpu: false

### General Training
task_mode: "dynamic_map_staged"
n_timesteps: 20000000
max_num_moves_per_eps: 150
goal_radius: 0.2

### Training Monitoring
monitoring:
  use_wandb: true
  eval_log: false

### Agent Specs: Training Hyperparameter and Network Architecture
rl_agent:
  architecture_name: "AGENT_54"
  resume: null

  frame_stacking:
    enabled: true
    stack_size: 8

  normalize: false
  laser:
    full_range_laser: true
    reduce_num_beams:
      enabled: true
      num_beams: 200

  reward_fnc: "rule_13"

  action_space:
    discrete: false
    custom_discretization:
      enabled: true
      buckets_linear_vel: 8
      buckets_angular_vel: 11

  ppo:
    batch_size: 120
    gamma: 0.99
    n_steps: 1200
    ent_coef: 0.005
    learning_rate: 0.0003
    vf_coef: 0.22
    max_grad_norm: 0.5
    gae_lambda: 0.95
    m_batch_size: 40
    n_epochs: 3
    clip_range: 0.22

callbacks:
  ### Periodic Eval
  periodic_eval:
    max_num_moves_per_eps: 250
    n_eval_episodes: 50
    eval_freq: 20000

  ### Training Curriculum
  training_curriculum:
    training_curriculum_file: "barn.yaml"
    curr_stage: 3
    threshold_type: "succ"
    upper_threshold: 0.95
    lower_threshold: 0.65

  ### Stop Training on Threshold
  stop_training:
    threshold_type: "succ"
    threshold: 0.95
```

</details>
