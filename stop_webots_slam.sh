#!/bin/bash

# Function to kill a process by name and check if it worked
kill_process() {
    local process_name="$1"
    local pids=$(pgrep -f "$process_name")
    if [ -n "$pids" ]; then
        echo "Stopping $process_name (PIDs: $pids)..."
        pkill -f "$process_name"
        sleep 1  # Give it a moment to terminate
        if pgrep -f "$process_name" > /dev/null; then
            echo "Warning: $process_name did not terminate cleanly. Forcing kill..."
            pkill -9 -f "$process_name"
        else
            echo "$process_name stopped successfully."
        fi
    else
        echo "No $process_name process found."
    fi
}

# Source ROS 2 environment to use ros2 commands (adjust path if needed)
ROS_SETUP="/opt/ros/jazzy/setup.bash"
if [ -f "$ROS_SETUP" ]; then
    source "$ROS_SETUP"
else
    echo "Warning: ROS 2 setup file not found at $ROS_SETUP. Skipping ROS 2 node cleanup."
fi

# Stop Webots
kill_process "webots"

# Stop the Webots controller (match both Markus and Stender paths)
kill_process "webots-controller.*epuck_controller.py"

# Stop ROS 2 SLAM Toolbox (target the launch process and slam_toolbox nodes)
kill_process "ros2 launch slam_toolbox"
kill_process "slam_toolbox"

# Stop RViz2 (target both the run command and the binary)
kill_process "ros2 run rviz2"
kill_process "rviz2"

# Clean up any remaining ROS 2 nodes (if ROS 2 is sourced)
if command -v ros2 > /dev/null 2>&1; then
    echo "Checking for active ROS 2 nodes..."
    ros2 node list > /dev/null 2>&1
    if [ $? -eq 0 ]; then
        echo "Killing all ROS 2 nodes..."
        ros2 node list | xargs -I {} ros2 node kill {}
    else
        echo "No active ROS 2 nodes found."
    fi
else
    echo "ROS 2 commands not available. Skipping node cleanup."
fi

# Optionally close gnome-terminal tabs
TERMINAL_PIDS=$(pgrep -f "gnome-terminal.*bash -c")
if [ -n "$TERMINAL_PIDS" ]; then
    echo "Closing terminal tabs (PIDs: $TERMINAL_PIDS)..."
    pkill -f "gnome-terminal.*bash -c"
else
    echo "No terminal tabs found to close."
fi

echo "All processes stopped!"
