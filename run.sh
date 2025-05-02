#!/bin/bash

CONFIG_FILE="config.cfg"

# --- First-time setup: Check for config file ---
if [ ! -f "$CONFIG_FILE" ]; then
    echo "Configuration file not found."

    # Prompt user for exe path
    read -p "Enter the path to your Webots executable: " WEBOTS_EXE

    if [ ! -e "$WEBOTS_EXE" ]; then
        echo "Error: The path does not exist. Exiting."
        exit 1
    fi

    # Prompt user for controller path
    read -p "Enter the path to your Webots controller.exe: " WEBOTS_CONTROLLER

    if [ ! -e "$WEBOTS_CONTROLLER" ]; then
        echo "Error: The path does not exist. Exiting."
        exit 1
    fi

    echo "WEBOTS_EXE=\"$WEBOTS_EXE\"" > "$CONFIG_FILE"
    echo "WEBOTS_CONTROLLER=\"$WEBOTS_CONTROLLER\"" >> "$CONFIG_FILE"
    echo "Configuration saved to $CONFIG_FILE."
fi

# Load config file
source "$CONFIG_FILE"

echo ""
read -p "Choose mode: centralised (c) or decentralised (d): " MODE_INPUT

# Normalize input
MODE_INPUT=$(echo "$MODE_INPUT" | tr '[:upper:]' '[:lower:]')
if [ "$MODE_INPUT" = "c" ]; then
    MODE="centralised"
elif [ "$MODE_INPUT" = "d" ]; then
    MODE="decentralised"
else
    echo "Invalid input. Please enter 'c' or 'd'. Exiting."
    exit 1
fi

# --- Robot count selection ---
echo ""
echo "Select number of robots: 2, 3, 5, 7, or 10"
read -p "Enter number of robots: " NUM_ROBOTS

if [[ "$NUM_ROBOTS" != "2" && "$NUM_ROBOTS" != "3" && "$NUM_ROBOTS" != "5" && "$NUM_ROBOTS" != "7" && "$NUM_ROBOTS" != "10" ]]; then
    echo "Invalid number of robots. Exiting."
    exit 1
fi

# Validate number of robots
if ! [[ "$NUM_ROBOTS" =~ ^[0-9]+$ ]] || [ "$NUM_ROBOTS" -lt 1 ]; then
    echo "Error: num_robots must be a positive integer."
    exit 1
fi

# Get the absolute path of the script's directory (repo root)
REPO_DIR="$(cd "$(dirname "$0")" && pwd)"


# Define world and controller paths
if [ "$NUM_ROBOTS" -eq 2 ]; then
    WORLD_PATH="$REPO_DIR/worlds/Experiment1.wbt"
elif [ "$NUM_ROBOTS" -eq 3 ]; then
    WORLD_PATH="$REPO_DIR/worlds/Experiment2.wbt"
elif [ "$NUM_ROBOTS" -eq 5 ]; then
    WORLD_PATH="$REPO_DIR/worlds/Experiment3.wbt"
elif [ "$NUM_ROBOTS" -eq 7 ]; then
    WORLD_PATH="$REPO_DIR/worlds/Experiment4.wbt"
elif [ "$NUM_ROBOTS" -eq 10 ]; then
    WORLD_PATH="$REPO_DIR/worlds/Experiment5.wbt"
else
    echo "Error: Invalid number of robots."
    exit 1
fi

CONTROLLER_PATH="$REPO_DIR/ros2_ws/install/epuck_nav2_pkg/share/epuck_nav2_pkg/scripts/swarm_nav_controller.py"

# Ensure Webots paths exist

if [ ! -f "$WEBOTS_EXE" ]; then
    echo "Error: Webots executable not found at $WEBOTS_EXE."
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
    echo "Error: Controller python script not found at $CONTROLLER_PATH."
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
$TERMINAL --tab --title="Webots" -- bash -c "$WEBOTS_EXE $WORLD_PATH; exec bash" &

# Give Webots a moment to start
sleep 2

# --- Final output and start ---
echo ""
echo "Launching ROS 2 system with..."
echo "Webots executable path: $WEBOTS_EXE"
echo "Webots controller path: $WEBOTS_CONTROLLER"
echo "Mode: $MODE"
echo "Number of Robots: $NUM_ROBOTS"

$TERMINAL --tab --title="ROS 2 System" -- bash -c "source $ROS_SETUP && source $WORKSPACE_SETUP && ros2 launch epuck_nav2_pkg epuck_nav2_swarm_launch.py \
    webots_controller:="$WEBOTS_CONTROLLER" \
    robot_count:="$NUM_ROBOTS"; exec bash"

echo "All components launched! Check the terminal tabs for output."