#!/bin/bash

# Check if an argument is provided
if [ $# -ne 1 ]; then
    echo "Usage: $0 <username>"
    echo "Example: $0 markus  or  $0 stender"
    exit 1
fi

# Set the username from the argument
USER=$1

# Define paths based on the user
case $USER in
    "markus")
        CONTROLLER_PATH="/usr/local/webots/webots-controller ~/Desktop/Master/MasterThesis/controllers/epuck_controller/epuck_controller_keyboard.py"
        WORLD_PATH="~/Desktop/Master/MasterThesis/worlds/SLAM.wbt"
        ;;
    "stender")
        CONTROLLER_PATH="/snap/webots/27/usr/share/webots/webots-controller ~/MasterThesis/controllers/epuck_controller/epuck_controller_keyboard.py"
        WORLD_PATH="~/MasterThesis/worlds/SLAM.wbt"
        ;;
    *)
        echo "Error: Unknown user '$USER'. Please use 'markus' or 'stender'."
        exit 1
        ;;
esac

# Ensure ROS 2 Jazzy is sourced (assuming it's installed in /opt/ros/jazzy)
ROS_SETUP="/opt/ros/jazzy/setup.bash"
if [ ! -f "$ROS_SETUP" ]; then
    echo "Error: ROS 2 Jazzy setup file not found at $ROS_SETUP. Please check your installation."
    exit 1
fi

# Terminal command (using gnome-terminal; adjust if you use a different terminal)
TERMINAL="gnome-terminal"

# Step 1: Launch Webots with the specified world
echo "Launching Webots with world: $WORLD_PATH..."
$TERMINAL --tab --title="Webots" -- bash -c "webots $WORLD_PATH; exec bash" &

# Give Webots a moment to start
sleep 2

# Step 2: Launch the controller
echo "Launching controller: $CONTROLLER_PATH..."
$TERMINAL --tab --title="Controller" -- bash -c "$CONTROLLER_PATH; exec bash" &

# Step 3: Launch SLAM Toolbox
echo "Launching SLAM Toolbox..."
$TERMINAL --tab --title="SLAM Toolbox" -- bash -c "source $ROS_SETUP && ros2 launch slam_toolbox online_async_launch.py; exec bash" &

# Step 4: Launch RViz2
echo "Launching RViz2..."
$TERMINAL --tab --title="RViz2" -- bash -c "source $ROS_SETUP && ros2 run rviz2 rviz2; exec bash" &

echo "All components launched! Check the terminal tabs for output."
