### Benchmark Task Mode

The Benchmark Mode is for benchmarking multiple planners (_contestants_) against each other on a predefined benchmarking _suite_.

#### Configuration

The central configuration file can be found in `arena_bringup/configs/benchmark/config.yaml`.

An example configuration can look like the following:

```yaml
suite:
  config: basic.yaml
  scale_episodes: 1

contest:
  config: inter.yaml

general:
  simulator: flatland
```

The configuration references a _suite_ and a _contest_, and also specifies general configuration parameters.

##### Suite

Suite configuration files are located in the subfolder `suites`.\
They contain a list of stage configurations: a sequence of benchmarking test cases that each contestant is subjected to.

Each stage is comprised of the following data:

```yaml
name: <unique name>
episodes: <number of episodes>
robot: <robot model>
map: <map file>
tm_robots: <robot task mode>
tm_obstacles: <obstacle task mode>
config: <task generator config object>
```

`config` is used to dynamically override the contents of `task_generator.yaml`.

##### Contests

Contest configuration files are located in the subfolder `contests`.\
They contain a list of planner configurations (contestants) which are benchmarked against each other.

Each contestant is comprised of the following data:

```yaml
name: <unique name>
local_planner: <local planner name>
inter_planner: <inter planner name>
```

#### Execution

The benchmarking process is started using the following command:

```bash
roslaunch arena_bringup start_arena.launch tm_modules:=benchmark
```

The simulator will now autonomously start, shut down, and reload. It is imperative that you do not interrupt this process – or else.
Should the process break unexpectedly, you can find a backup file of `task_generator.yaml` in the same directory.

Each contestant goes through the suite and records data while doing so. You will know that the process is finished once the file `resume.lock` is removed from the `benchmark` directory.\
You can now find your recorded data in `arena-evaluation/data`.