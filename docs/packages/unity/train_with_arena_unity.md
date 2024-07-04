
# Arena Unity Training Guide

## Introduction

This guide provides instructions on how to start an example training run with Arena Unity and explains how to configure the training with Arena Unity. If you want to extend Arena Unity or design your own agent with a custom observation space and feature extractor, afterwards, refer to the technical integration docs [here](training_integration.md).

## Starting an Example Training with Arena Unity

Follow the guide below to start an exmaple training run with Arena Unity as the simulator using an 'Arena Unity'-specific training configuration.

### Steps

1. **In a poetry shell, in `src/arena/arena-rosnav`, run:**

```bash
roslaunch arena_bringup start_training.launch model:=jackal num_envs:=4 simulator:=unity headless:=false
```

2. **In another poetry shell, in `src/arena/arena-rosnav`, run:**

```bash
python training/scripts/train_agent.py --config unity_rgbd_training_config.yaml
```

## Configuring the Training and Agent with Arena Unity

The training process is configured via a config file (e.g., `unity_rgbd_training_config.yaml`) and launch arguments. The agent description is set up in the Rosnav-rl repo within `custom_sb3_policy.py`.

### Configuring Your DRL Agent

- **Agent Description**: Use an existing agent description or create a new one in `custom_sb3_policy.py` located in the `rosnav-rl` repo (cloned in `src/planners/rosnav`).
    - Example: `ArenaUnityResNet_1`

    - **Observation Spaces**:
        - Configure observation spaces used as input to the agent with `observation_spaces`.
        - Example: Use `SPACE.RGBDSpace` for RGBD input.
        - `RGBDSpace` requires spatial dimensions as input, which can be specified in `observation_space_kwargs`.

    - **Feature Extractor**:
        - Specified in the agent description via `features_extractor_class`.
        - Currently implemented extractor: `RESNET_RGBD_FUSION_EXTRACTOR_1`.
        - To create a custom feature extractor, extend the RGBD feature extractor.

### Configuring the Training

- **Training Config File**:
    - Use `unity_rgbd_training_config.yaml` as a starting point.
    - Modify the agent description in `custom_sb3_policy.py` by changing `architecture_name`.
    - Customize reward functions by creating a reward config and specifying it in `reward_fnc`.
    - Customize the training curriculum (number of static and dynamic obstacles, etc.) by creating a config and specifying it in `training_curriculum_file`.

- **Launch Arguments**:
    - `headless`: Set to false for testing to see what is happening. When set to true, the main camera will be removed, greatly improving performance.
    - `num_envs`: Ensure this matches the number set in the training config.

By following these steps, you can effectively start and configure a DRL training run with Arena Unity. For further customization and advanced configurations, refer to the detailed technical documentation provided in this training section.

