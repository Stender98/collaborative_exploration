#!/bin/bash

# Check if two arguments are provided: username and number of robots
if [ $# -ne 2 ]; then
    echo "Usage: $0 <username> <num_robots>"
    echo "Example: $0 markus 3"
    echo "Username must be 'markus' or 'stender'"
    exit 1
fi

# Set the username and number of robots from arguments
USER=$1
NUM_ROBOTS=$2

# Validate number of robots
if ! [[ "$NUM_ROBOTS" =~ ^[0-9]+$ ]] || [ "$NUM_ROBOTS" -lt 1 ]; then
    echo "Error: num_robots must be a positive integer."
    exit 1
fi

# Get the absolute path of the script's directory (repo root)
REPO_DIR="$(cd "$(dirname "$0")" && pwd)"

# Define paths based on the user
case $USER in
    "markus")
        WEBOTS_HOME="/usr/local/webots"
        WEBOTS_CMD="$WEBOTS_HOME/webots"
        WEBOTS_CONTROLLER="$WEBOTS_HOME/webots-controller"
        ;;
    "stender")
        WEBOTS_HOME="/snap/webots/27/usr/share/webots"
        WEBOTS_CMD="$WEBOTS_HOME/webots"
        WEBOTS_CONTROLLER="$WEBOTS_HOME/webots-controller"
        ;;
    *)
        echo "Error: Unknown user '$USER'. Please use 'markus' or 'stender'."
        exit 1
        ;;
esac

# Define world and controller paths
WORLD_PATH="$REPO_DIR/worlds/SLAM.wbt"
CONTROLLER_PATH="$REPO_DIR/ros2_ws/install/epuck_nav2_pkg/share/epuck_nav2_pkg/scripts/swarm_nav_controller.py"

# Ensure Webots paths exist
if [ ! -f "$WEBOTS_CMD" ]; then
    echo "Error: Webots executable not found at $WEBOTS_CMD."
    exit 1
fi
if [ ! -f "$WEBOTS_CONTROLLER" ]; then
    echo "Error: Webots controller executable not found at $WEBOTS_CONTROLLER."
    exit 1
fi
if [ ! -f "$WORLD_PATH" ]; then
    echo "Error: Webots world file not found at $WORLD_PATH."
    exit 1
fi
if [ ! -f "$CONTROLLER_PATH" ]; then
    echo "Error: Controller script not found at $CONTROLLER_PATH."
    exit 1
fi

# Ensure ROS 2 Jazzy is sourced
ROS_SETUP="/opt/ros/jazzy/setup.bash"
if [ ! -f "$ROS_SETUP" ]; then
    echo "Error: ROS 2 Jazzy setup file not found at $ROS_SETUP."
    exit 1
fi

# Source the ROS 2 workspace (build it if not already built)
WORKSPACE_SETUP="$REPO_DIR/ros2_ws/install/setup.bash"
if [ ! -f "$WORKSPACE_SETUP" ]; then
    echo "Building ROS 2 workspace..."
    cd "$REPO_DIR/ros2_ws"
    source "$ROS_SETUP"
    colcon build
    if [ $? -ne 0 ]; then
        echo "Error: Failed to build ROS 2 workspace."
        exit 1
    fi
fi

# Terminal command
TERMINAL="gnome-terminal"

# Step 1: Launch Webots with the specified world
echo "Launching Webots with world: $WORLD_PATH..."
$TERMINAL --tab --title="Webots" -- bash -c "$WEBOTS_CMD $WORLD_PATH; exec bash" &

# Give Webots a moment to start
sleep 2

# Step 2: Launch Webots controllers for each robot
#for ((i=0; i<NUM_ROBOTS; i++)); do
#    ROBOT_ID="robot$i"
#    echo "Launching controller for $ROBOT_ID..."
#    $TERMINAL --tab --title="Controller $ROBOT_ID" -- bash -c \
#       "source $ROS_SETUP && source $WORKSPACE_SETUP && \
#         $WEBOTS_CONTROLLER --robot-name=$ROBOT_ID $CONTROLLER_PATH --robot-id=$ROBOT_ID; exec bash" &
#    sleep 1  # Stagger controller launches
#done

# Step 3: Launch SLAM Toolbox for multi-robot map merging
echo "Launching SLAM Toolbox (online_async_multirobot_launch.py)..."
$TERMINAL --tab --title="SLAM Toolbox" -- bash -c \
    "source $ROS_SETUP && source $WORKSPACE_SETUP && \
     ros2 launch slam_toolbox online_async_multirobot_launch.py; exec bash" &

# Give SLAM Toolbox time to initialize
sleep 5

echo "Launching ROS 2 system with epuck_nav2_launch.py..."
$TERMINAL --tab --title="ROS 2 System" -- bash -c "source $ROS_SETUP && source $WORKSPACE_SETUP && ros2 launch epuck_nav2_pkg epuck_nav2_launch.py; exec bash" &

# Step 5: Launch RViz2 with custom configuration
#RVIZ_CONFIG="$REPO_DIR/ros2_ws/src/epuck_nav2_pkg/config/nav2_rviz_config.rviz"
#echo "Launching RViz2 with config: $RVIZ_CONFIG..."
#$TERMINAL --tab --title="RViz2" -- bash -c \
#    "source $ROS_SETUP && source $WORKSPACE_SETUP && \
#     ros2 run rviz2 rviz2 -d $RVIZ_CONFIG; exec bash" &

echo "All components launched! Check the terminal tabs for output."