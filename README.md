# Running the Simulation ROS2 Native System

## Prerequisites
Ensure you have the following installed and set up:
- [Webots](https://cyberbotics.com/)
- [ROS2](https://docs.ros.org/en/)
- `slam_toolbox` package for ROS2
- `rviz2` for visualization

## Steps to Run the Simulation
Open four terminal windows and execute the following commands in order:

### 1: Start Webots
```sh
webots
```

Open the SLAM world `SLAM.wbt`

### 2: Run the Webots Controller
```sh
<path_to_webots>/webots-controller <path_to_your_controller>/epuck_controller.py
```

Refer to [Webots documentation](https://cyberbotics.com/doc/guide/running-extern-robot-controllers) for help on running an external controller.

### 3: Launch SLAM Toolbox
```sh
ros2 launch slam_toolbox online_async_launch.py
```

### 4: Start RViz for Visualization
```sh
ros2 run rviz2 rviz2
```

Add by topic /map, /pose, or /scan for (Occupancy grid, estimated pose, LiDAR scans)

## Notes
- Ensure Webots is properly installed and accessible.
- Modify paths as needed if your setup differs.
- SLAM Toolbox is used for real-time mapping.
- RViz2 is required for visualizing the generated map and robot state.
- The controller uses packages from Webots that should might not initially be available in your default python environment.

For further configuration and troubleshooting, refer to the respective documentation of Webots and ROS2.

