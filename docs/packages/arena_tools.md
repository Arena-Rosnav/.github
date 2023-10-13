## Arena-Tools

**Arena-Tools** is a collection of tools designed to simplify working with **Arena-Rosnav** and **Arena-Rosnav-3D**. It currently comprises the following components:

### Prerequisites

To use Arena-Tools, ensure you have Python 3.6 or higher. Install the required Python packages in your virtual environment:

```bash
pip3 install pyqt5 numpy pyyaml lxml scikit-image Pillow scikit-image opencv-python matplotlib
pip install PyQt5 --upgrade
```

For 2D map to 3D Gazebo world functionality, follow these additional steps:

1. Install `potrace` and `imagemagick` to convert png/pgm images to SVG format:

   ```bash
   sudo apt-get install potrace imagemagick
   ```

2. Install Blender according to the documentation, ensuring it can be run from the terminal. Note: This feature was tested with Blender v3.1.

### Running Arena-Tools

To start the Arena-Tools GUI and select your desired task, run:

```bash
roscd arena-tools && python arena_tools.py
```

### Map Generator

- **Map Generator** is a tool for generating random ROS maps, suitable for both indoor and outdoor settings. You can customize maps by adjusting parameters such as corridor width, iterations, obstacles, and obstacle extra radius.

- If you wish to convert the generated maps into Gazebo worlds, a button automates the process if you've followed the installation steps.

### Scenario Editor

- **Scenario Editor** is used to create scenarios for Arena-Rosnav. It allows you to interact with the scene, set maps, define robot positions and goals, and add Pedsim agents and flatland objects (static obstacles).

- Scenarios can be saved in YAML or JSON format.

### Flatland Model Editor

- **Flatland Model Editor** is used to create models for Flatland, which is a component of Arena-Rosnav. It enables you to edit basic properties of a body, add polygon and circle footprints, and save models.

- You can load and save models, add bodies, and edit footprints with ease using this tool.
