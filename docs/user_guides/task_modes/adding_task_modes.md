## Adding your own task modes

This tutorial will show how to add your own task modes. There are three different kind of task modes (Obstacle, Robot and Module):

- Obstacle task modes manage the obstacle types and positions,
- Robot task modes manage the start and goal positions for the robots,
- Module task modes are responsible for additional features.

### 1. Creating a .py file for your task mode

To add your new task mode, add a new .py file named `<your_task_mode>.py` in the respective folder:

- `task_generator/task_generator/tasks/obstacles`
- `task_generator/task_generator/tasks/robots`
- `task_generator/task_generator/tasks/modules`

Your .py file should look like the following examples depending on your type of task mode:

For obstacle task modes:

```py
from task_generator.constants import Constants
from task_generator.tasks.obstacles import Obstacles, TM_Obstacles
from task_generator.tasks.task_factory import TaskFactory

@TaskFactory.register_obstacles(Constants.TaskMode.TM_Obstacles.<YOUR_TASK_MODE>)
class <YOUR_TASK_MODE>(TM_Obstacles):

    def reset(self, **kwargs) -> Obstacles:
        ...

```

For robot task modes:

```py
from task_generator.constants import Constants
from task_generator.tasks.robots import TM_Robots
from task_generator.tasks.task_factory import TaskFactory

@TaskFactory.register_robots(Constants.TaskMode.TM_Robots.<YOUR_TASK_MODE>)
class <YOUR_TASK_MODE>(TM_Robots):

    def reset(self, **kwargs):
        ...

    @property
    def done(self):
        ...
```

For module task modes:

```py
from task_generator.constants import Constants
from task_generator.tasks import Reconfigurable, Task
from task_generator.tasks.task_factory import TaskFactory

@TaskFactory.register_module(Constants.TaskMode.TM_Module.<YOUR_TASK_MODE>)
class <YOUR_TASK_MODE>(TM_Modules):

    def before_reset(self):
        ...

    def after_reset(self):
        ...
```

### 2. Registration of your task mode

Now you should add a unique identifier in the `TaskMode` class `src/arena-rosnav/task_generator/task_generator/constants.py` (This identifier will be used in the launchfile argument). Append it in the respective Enum class.

e.g. for obstacle task modes:

```py
class TaskMode:
        @enum.unique
        class TM_Obstacles(enum.Enum):
            ...
            <YOUR_TASK_MODE> = "your_task_mode"
        ...
```

Import your task mode in the file `src/arena-rosnav/task_generator/task_generator/tasks/task_factory.py`. Add the following line at the end of the document.

e.g. for obstacle task modes:

```py
from .obstacles.<your_task_mode> import <YOUR_TASK_MODE>
```

### 3. Implementation

Obstacle and robot task modes require a `reset` method. These will be called during a reset. 

For obstacle task modes, it also has to return `Obstacles` which is a tuple of two lists containing `Static Obstacles` and `Dynamic Obstacles`.

The robot task mode has a property `done` that can be overridden. This property should return `True` if the robots are done with their tasks else `False`. If the tasks are done, a reset will be triggered.

The module task mode does not have a `reset` method but rather two methods `before_reset` and `after_reset`. These methods will be called before and after a reset occurs.



