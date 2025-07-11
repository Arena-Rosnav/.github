## Adding your own task modes

> Context: task_generator packge

This tutorial will show how to add your own task modes. There are three different kind of task modes (Obstacle, Robot and Module):

- Obstacle task modes manage the obstacle types and positions,
- Robot task modes manage the start and goal positions for the robots,
- Module task modes are responsible for additional features.

### 1. Creating the Implementation Class

To add your new task mode, add a new .py file named `<your_task_mode>.py` in the respective folder:

- `task_generator/task_generator/tasks/obstacles/`
- `task_generator/task_generator/tasks/robots/`
- `task_generator/task_generator/tasks/modules/`

Your class must implement the respective base class, and your .py file should start with this template:

For obstacle task modes:

```py
from task_generator.tasks.obstacles import Obstacles, TM_Obstacles

class TM_<YOUR_TASK_MODE>(TM_Obstacles):
    ...
```

For robot task modes:

```py
from task_generator.tasks.robots import TM_Robots

class TM_<YOUR_TASK_MODE>(TM_Robots):
    ...
```

For module task modes:

```py
from task_generator.tasks.modules import TM_Module

class TM_<YOUR_TASK_MODE>(TM_Module):
    ...
```

### 2. Registration of your task mode

Now you should add a unique identifier in the `TaskMode` class `task_generator/task_generator/constants/__init__.py` (This identifier will be used in the launchfile argument). Append it in the respective Enum class.

Import your task mode in the file `task_generator/task_generator/tasks/__init__.py`. Add the following line in the respective section at the end of the module.

e.g. for obstacle task modes:

```py
@TaskFactory.register_obstacles(Constants.TaskMode.TM_Obstacles.<YOUR_TASK_MODE>)
    def _random():
        from .obstacles.<YOUR_TASK_MODE> import TM_<YOUR_TASK_MODE>
        return TM_YOUR_TASK_MODE
```

### 3. Implementation

Override the methods of the base classes as needed.