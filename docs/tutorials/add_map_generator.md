# Create a Map Generator

To add a new Map Generator first add a file `arena-rosnav/utils/misc/map_generator/map_generator/example/map_gen.py`, create a class that inherits from `BaseMapGenerator` and register it as a Map Generator.


Then implement following functions:

- `update_params()`
- `retrieve_params()`
- `generate_grid_map()`


``` python
from typing import Tuple
import numpy as np

@MapGeneratorFactory.register("example")
class ExampleMapGenerator(BaseMapGenerator):
  def update_params(self, *args, **kwargs):
    # ...
  def retrieve_params(self) -> Tuple:
    #...
  def generate_grid_map(self) -> (np.ndarray, dict):
    #...

```

#### update_params()
Updates the map parameters. Initially already takes arguments

- `height: int`
- `width: int`
- `map_resolution: int`

Should be extended by additional configurations desired for algorithm.

#### retrieve_params()
Retrieves the map parameters from ROS. 

Initially returns `Iterable[Union[int, float, str]]`. A tuple containing the height, width, and map resolution.

Should be extended by additional configurations desired for algorithm.

#### generate_grid_map()
Updates parameters (retrieved from ROS) and generates a grid map and obstacles.

Should return:
- `np.ndarray`: The generated gridmap as a 2D Array of values in [0,1]
- `dict`: In the form of {`obstacles`: Array of Obstacles of the map, `occupancy`: spaces blocked by obstacles in the gridmap}
