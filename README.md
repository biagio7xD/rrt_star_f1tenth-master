# RRT* Implementation for Autonomous Driving

This repository contains an implementation of the RRT* (Rapidly-exploring Random Tree Star) algorithm for autonomous driving. The RRT* algorithm is used for path planning in dynamic environments, enabling an autonomous vehicle to navigate safely and efficiently towards a desired destination.

## Features

- RRT* algorithm for global path planning.
- Integration with ROS (Robot Operating System) for communication and control.
- Sensor integration for obstacle detection and avoidance.
- Visualization of the planned path in a simulated environment.

## Requirements

- ROS
- C++
- Python
- Unity
- Unity F1/10 autonomous racing car simulator

## Installation

1. Clone the repository:

   ```bash
   git clone https://github.com/biagiolicari/rrt-star-autonomous-driving.git](https://github.com/biagiolicari/F1Tenth-Rapidly-exploring-random-tree-STAR.git)
   ```

2. Install the required dependencies. Please refer to the individual packages' documentation for installation instructions. |

3. Set up the Unity F1/10 autonomous racing car simulator. Follow the instructions provided by the simulator's documentation to set up and launch the simulator.

## Usage
1. Launch the simulator and load the desired scene.
2. Run the following commands in separate terminals to start the necessary ROS nodes: |
```bash 
 cd path/to/rrt_star/map
```
```bash
 rosrun map_server map_server map.yaml
```

3. Start the RRT* path planning algorithm:
```bash
  roslaunch rrt rrt_node.launch
```

4. Start the controller node to execute the planned path:
```bash
  cd path/to/rrt_star/purepursuit
```

```bash
  python ros_pp_local.py
```

```bash
  rostopic pub /commands/stop std_msgs/Bool "data: false"
```

5. Monitor the visualization of the planned path and the vehicle's movement in the simulator.

## Configuration
- Modify the parameters in the `*.yaml` file to adjust the behavior of the RRT* algorithm, such as maximum iterations, step size, and goal bias.

## License
This project is licensed under the [MIT License](LICENSE).
