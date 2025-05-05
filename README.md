# TBD project title
Ensure you have the following installed and set up:
- This project has been tested and build on Ubuntu 24.04 LTS (but the software used is also supported on Windows 10 or macOS.) 
- [Webots](https://cyberbotics.com/)
    - Troubleshooting tip: If you are unable to connect the controllers to Webots, refer to the [Webots documentation](https://cyberbotics.com/doc/guide/running-extern-robot-controllers) for help on running an external controller.
- [ROS2](https://docs.ros.org/en/) Jazzy and the following packages:
    - `Navigation2` for autonomous navigation
    - `rviz2` for visualisation
    - `SLAM Toolbox` specifically the modified 'jazzy'-branch from [our forked repository](https://github.com/Stender98/slam_toolbox) should available in your ros2_ws/src/slam_toolbox
- Python3

## Guide to run the simulation
### 1: Build the workspace
If not already build, source the setup.bash, .sh or .zsh and build the ROS2 workspace.
```sh
source /opt/ros/jazzy/setup.bash 
cd ros2_ws
colcon build
```

Troubleshooting help can be found in the [ROS2 documentation.](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)


### 2: Setup the run script.
During your first time running the `run.sh` script you will be prompted to provide the path to your Webots executable and Webots controller executeable. Refer to [Webots documentation](https://cyberbotics.com/doc/guide/running-extern-robot-controllers) for help on running an external controller.

A config.cfg file will hold these paths in the root folder of this repository, where if needed, changes can be applied to these paths. 

An example config.cfg file is the following:
```sh
WEBOTS_EXE="/snap/bin/webots"
WEBOTS_CONTROLLER="/snap/webots/27/usr/share/webots/webots-controller"
```

```sh
./run.sh
```

The run script will prompt you to choose between the centralised or decentralised approach and responds to the input `c` or `d`. Following that you select one of the available sizes of swarms presented. The corresponding Webots world will be launched followed by the robot controllers after a short delay.

## Notes
- The controller uses Python packages and packages from Webots that might not initially be available in your python environment.

For further configuration and troubleshooting, please refer to the respective documentation of Webots and ROS2.

