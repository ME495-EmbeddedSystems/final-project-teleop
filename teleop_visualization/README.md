# `teleop_visualization` Package

### Running the Code
You can run the launch file, which launches a visualization of the avatar workspace for the ring task, using the following command:

```bash
ros2 launch teleop_visualization rings.launch.xml
```

__Default View__:
Upon starting only this node, you will see a scene which looks like this:
![Default Visualization Screen](https://github.com/ME495-EmbeddedSystems/final-project-teleop/assets/122302059/607f3553-d389-4377-9223-9e869b37e54c)

### Getting Live Ring Positions
In order to get live positions of rings, the sensing node (found in the `teleop_sensing` package) needs to be running. You can launch this node, along with the visualization node, using the following command:

```bash
ros2 launch teleop_visualization live_rings.launch.xml
```

### Visualizing The Actual Avatar Robot Configuration
The visualizer can also show the current configuration of the avatar robot. For this to happen, the avatar robot's nodes must be running and publishing the current joint angles of the robots.