#!/bin/bash

# Check if an argument is provided
if [ $# -ne 1 ]; then
    echo "Usage: $0 <username>"
    echo "Example: $0 markus  or  $0 stender"
    exit 1
fi

# Set the username from the argument
USER=$1

# Get the absolute path of the script's directory (repo root)
REPO_DIR="$(cd "$(dirname "$0")" && pwd)"

# Define paths based on the user
case $USER in
    "markus")
        WEBOTS_CMD="/usr/local/webots/webots"
        WORLD_PATH="$REPO_DIR/worlds/SLAM.wbt"
        ;;
    "stender")
        WEBOTS_CMD="/snap/webots/27/usr/share/webots/webots"
        WORLD_PATH="$REPO_DIR/worlds/SLAM.wbt"
        ;;
    *)
        echo "Error: Unknown user '$USER'. Please use 'markus' or 'stender'."
        exit 1
        ;;
esac

# Ensure ROS 2 Jazzy is sourced
ROS_SETUP="/opt/ros/jazzy/setup.bash"
if [ ! -f "$ROS_SETUP" ]; then
    echo "Error: ROS 2 Jazzy setup file not found at $ROS_SETUP. Please check your installation."
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

# Step 2: Launch the ROS 2 system (controller, SLAM Toolbox, Nav2)
echo "Launching ROS 2 system with epuck_nav2_launch.py..."
$TERMINAL --tab --title="ROS 2 System" -- bash -c "source $ROS_SETUP && source $WORKSPACE_SETUP && ros2 launch epuck_nav2_pkg epuck_nav2_launch.py; exec bash" &

# Step 3: Launch RViz2 with a Nav2 configuration
echo "Launching RViz2..."
$TERMINAL --tab --title="RViz2" -- bash -c "source $ROS_SETUP && ros2 run rviz2 rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rvz; exec bash" &

echo "All components launched! Check the terminal tabs for output."