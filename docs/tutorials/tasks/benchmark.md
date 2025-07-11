### Benchmark Task Mode

The Benchmark Mode is for benchmarking multiple planners (_contestants_) against each other on a predefined benchmarking _suite_.

#### Configuration

The central configuration file can be found in `arena_bringup/configs/benchmark/config.yaml`.

An example configuration can look like the following:

| Key        | Field           | Type             | Description                                                                     |
|------------|-----------------|------------------|---------------------------------------------------------------------------------|
| suite      | config          | String           | suite configuration yaml file to be used                                        |
|            | scale_episodes  | int              | amount of suite simulation                                                      |
| contest    | config          | String           | contest configuration yaml file to be used                                      |
| general    | simulator       | flatland, gazebo | name of the simulator                                                           |

The configuration references a _suite_ and a _contest_, and also specifies general configuration parameters.

##### Suite

Suite configuration files are located in the subfolder `suites`.\
They contain a list of stage configurations: a sequence of benchmarking test cases that each contestant is subjected to.

Each stage is comprised of the following data:

| Key             | Type          | Description                                                                     |
|-----------------|---------------|---------------------------------------------------------------------------------|
| name            | String        | unique name for the stage                                                       |
| episodes        | int           | amount of episodes (resets)                                                     |
| robot           | String        | name of the robot you want to simulate. e.g.: [burger, jackal, ...]             |
| map             | String        | name of the map. The map must be located in  /arena_simulation_setup/maps       |
| tm_robots       | random, scenario,<br /> explore, guided | robot task mode you want to use                       |
| tm_obstacles    | random, scenario,<br /> parametrized    | obstacle task mode you want to use                    |
| config          | Object        | `config` is used to dynamically override the contents of `task_generator.yaml`. |

##### Contests

Contest configuration files are located in the subfolder `contests`.\
They contain a list of planner configurations (contestants) which are benchmarked against each other.

Each contestant is comprised of the following data:

| Key             | Type          | Description                                                                     |
|-----------------|---------------|---------------------------------------------------------------------------------|
| name            | String        | unique name for the stage                                                       |
| local_planner   | String        | name of the local planner you want to use. e.g.: [teb, dwa, mpc,...]            |



#### Execution

The benchmarking process is started using the following command:

```sh
ros2 launch arena_bringup arena.launch tm_modules:=benchmark
```