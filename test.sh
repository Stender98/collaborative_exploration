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

if [ "$MODE" = "centralised" ]; then
    echo "Centralised mode selected."
    LAUNCH="epuck_nav2_swarm_launch.py"
elif [ "$MODE" = "decentralised" ]; then
    echo "Decentralised mode selected."
    LAUNCH="epuck_decentralised_swarm_launch.py"
else
    echo "Error: Invalid mode. Exiting."
    exit 1
fi

echo ""
echo "Select number of robots: 1, 2, 3, 5, 8, 13 or 21"
read -p "Enter number of robots: " NUM_ROBOTS

# Validate number of robots
if [[ "$NUM_ROBOTS" != "1" && "$NUM_ROBOTS" != "2" && "$NUM_ROBOTS" != "3" && "$NUM_ROBOTS" != "5" && "$NUM_ROBOTS" != "8" && "$NUM_ROBOTS" != "13" && "$NUM_ROBOTS" != "21" ]]; then
    echo "Invalid number of robots. Exiting."
    exit 1
fi

if ! [[ "$NUM_ROBOTS" =~ ^[0-9]+$ ]] || [ "$NUM_ROBOTS" -lt 1 ]; then
    echo "Error: num_robots must be a positive integer."
    exit 1
fi

echo ""
read -p "Enter number of runs: " RUN_COUNT

# Validate run_count
if ! [[ "$RUN_COUNT" =~ ^[0-9]+$ ]] || [ "$RUN_COUNT" -lt 1 ]; then
    echo "Error: run_count must be a positive integer."
    exit 1
fi

# Get the absolute path of the script's directory (repo root)
REPO_DIR="$(cd "$(dirname "$0")" && pwd)"

# Define world and controller paths
if [ "$NUM_ROBOTS" -eq 1 ]; then
    WORLD_PATH="$REPO_DIR/worlds/Experiment_1_Robot.wbt"
elif [ "$NUM_ROBOTS" -eq 2 ]; then
    WORLD_PATH="$REPO_DIR/worlds/Experiment_2_Robot.wbt"
elif [ "$NUM_ROBOTS" -eq 3 ]; then
    WORLD_PATH="$REPO_DIR/worlds/Experiment_3_Robot.wbt"
elif [ "$NUM_ROBOTS" -eq 5 ]; then
    WORLD_PATH="$REPO_DIR/worlds/Experiment_5_Robot.wbt"
elif [ "$NUM_ROBOTS" -eq 8 ]; then
    WORLD_PATH="$REPO_DIR/worlds/Experiment_8_Robot.wbt"
elif [ "$NUM_ROBOTS" -eq 13 ]; then
    WORLD_PATH="$REPO_DIR/worlds/Experiment_13_Robot.wbt"
elif [ "$NUM_ROBOTS" -eq 21 ]; then
    WORLD_PATH="$REPO_DIR/worlds/Experiment_21_Robot.wbt"
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

# Path to the logging script
LOGGING_SCRIPT="$REPO_DIR/logging/map_coverage_logger.py"
if [ ! -f "$LOGGING_SCRIPT" ]; then
    echo "Error: Logging script not found at $LOGGING_SCRIPT."
    exit 1
fi

# Terminal command
TERMINAL="gnome-terminal"

# Function to check for coverage completion
check_coverage_completion() {
    local max_attempts=10
    local attempt=1
    local sleep_time=2
    
    source "$ROS_SETUP"
    source "$WORKSPACE_SETUP"
    
    while [ $attempt -le $max_attempts ]; do
        echo "Attempt $attempt/$max_attempts: Checking for coverage completion..."
        COVERAGE_COMPLETE=$(ros2 topic echo --once --timeout 3 /coverage_complete 2>/dev/null | grep "data: true" || true)
        if [ -n "$COVERAGE_COMPLETE" ]; then
            echo "Coverage completion message received!"
            return 0
        fi
        sleep $sleep_time
        ((attempt++))
    done
    return 1
}

# Loop for the specified number of runs
for ((RUN_INDEX=1; RUN_INDEX<=RUN_COUNT; RUN_INDEX++)); do
    echo "Starting run $RUN_INDEX of $RUN_COUNT..."

    # Create log directories for this run
    mkdir -p "$REPO_DIR/logs/$MODE/$NUM_ROBOTS/$RUN_INDEX"

    # Step 1: Launch Webots with the specified world
    echo "Launching Webots with world: $WORLD_PATH..."
    $TERMINAL --tab --title="Webots (Run $RUN_INDEX)" -- bash -c "$WEBOTS_EXE --no-rendering --minimize $WORLD_PATH; exec bash" &
    WEBOTS_PID=$!

    # Give Webots a moment to start
    sleep 2

    echo ""
    echo "Launching ROS 2 system with..."
    echo "Webots executable path: $WEBOTS_EXE"
    echo "Webots controller path: $WEBOTS_CONTROLLER"
    echo "Mode: $MODE"
    echo "Number of Robots: $NUM_ROBOTS"
    echo "Run Index: $RUN_INDEX"

    # Step 2: Launch the ROS 2 system and cpu load monitor
    $TERMINAL --tab --title="CPU Load Monitor (Run $RUN_INDEX)" -- bash -c "python3 $REPO_DIR/logging/cpu_logger.py $MODE_INPUT $NUM_ROBOTS $RUN_INDEX; exec bash" &
    CPU_MONITOR_PID=$!

    $TERMINAL --tab --title="ROS 2 System (Run $RUN_INDEX)" -- bash -c "source $ROS_SETUP && source $WORKSPACE_SETUP && ros2 launch epuck_nav2_pkg $LAUNCH \
        webots_controller:='$WEBOTS_CONTROLLER' \
        robot_count:='$NUM_ROBOTS'; exec bash" &
    ROS2_PID=$!

    # Step 3: Wait for /map and /clock topics to become available
    echo "Waiting for /map and /clock topics to become available..."
    source "$ROS_SETUP"
    source "$WORKSPACE_SETUP"
    TIMEOUT=30
    START_TIME=$(date +%s)
    while true; do
        MAP_TOPIC=$(ros2 topic list | grep -w "/map" || true)
        CLOCK_TOPIC=$(ros2 topic list | grep -w "/clock" || true)
        if [ -n "$MAP_TOPIC" ] && [ -n "$CLOCK_TOPIC" ]; then
            echo "Topics /map and /clock are available."
            break
        fi
        CURRENT_TIME=$(date +%s)
        ELAPSED=$((CURRENT_TIME - START_TIME))
        if [ $ELAPSED -ge $TIMEOUT ]; then
            echo "Error: Timeout waiting for /map and /clock topics."
            kill -9 $WEBOTS_PID $ROS2_PID 2>/dev/null
            exit 1
        fi
        sleep 1
    done

    # Step 4: Launch the logging script
    echo "Launching map coverage logger..."
    $TERMINAL --tab --title="Map Coverage Logger (Run $RUN_INDEX)" -- bash -c "source $ROS_SETUP && source $WORKSPACE_SETUP && python3 $LOGGING_SCRIPT $MODE_INPUT $NUM_ROBOTS $RUN_INDEX; exec bash" &
    LOGGER_PID=$!

    # Give the logger some time to initialize
    sleep 5

    # Step 5: Wait for 95% map coverage
    echo "Waiting for 95% map coverage..."
    source "$ROS_SETUP"
    source "$WORKSPACE_SETUP"
    TIMEOUT=1800  # 30 minutes timeout to prevent infinite loops
    START_TIME=$(date +%s)
    COVERAGE_FOUND=false
    
    while [ "$COVERAGE_FOUND" = false ]; do
        CURRENT_TIME=$(date +%s)
        ELAPSED=$((CURRENT_TIME - START_TIME))
        
        if [ $ELAPSED -ge $TIMEOUT ]; then
            echo "Error: Timeout waiting for 95% map coverage."
            kill -9 $WEBOTS_PID $ROS2_PID $LOGGER_PID $CPU_MONITOR_PID 2>/dev/null
            exit 1
        fi
        
        # Check for coverage completion using the function
        if check_coverage_completion; then
            COVERAGE_FOUND=true
            echo "95% map coverage confirmed."
            break
        fi
        
        # Check if the logger process is still running
        if ! ps -p $LOGGER_PID > /dev/null; then
            echo "Warning: Logger process ended unexpectedly. Checking coverage status..."
            # Try to check coverage status
            if check_coverage_completion; then
                COVERAGE_FOUND=true
                echo "95% map coverage confirmed after logger exit."
                break
            else
                echo "Error: Logger process ended but coverage not reached."
                kill -9 $WEBOTS_PID $ROS2_PID $CPU_MONITOR_PID 2>/dev/null
                exit 1
            fi
        fi
        
        # Wait a bit before checking again
        sleep 5
    done

    # Step 6: Stop cpu monitor, save map and run evaluation scripts
    echo "Stopping CPU load monitor..."
    kill -9 $CPU_MONITOR_PID $LOGGER_PID 2>/dev/null
    # Ensure the CPU monitor is terminated
    pkill -f "cpu_logger.py" 2>/dev/null
    pkill -f "$LOGGING_SCRIPT" 2>/dev/null

    echo "Saving map and running evaluation scripts..."
    ros2 run nav2_map_server map_saver_cli -f "$REPO_DIR/logs/$MODE/$NUM_ROBOTS/$RUN_INDEX/slam_map" 
    sleep 5
    echo "Map saved."
    $TERMINAL --tab --title="Map Evaluation (Run $RUN_INDEX)" -- bash -c "python3 $REPO_DIR/logging/crop.py $MODE_INPUT $NUM_ROBOTS $RUN_INDEX && python3 $REPO_DIR/logging/map_evaluation.py $MODE_INPUT $NUM_ROBOTS $RUN_INDEX && python3 $REPO_DIR/logging/cpu_plot.py $MODE_INPUT $NUM_ROBOTS $RUN_INDEX; exec bash" &
    EVAL_PID=$!

    # Wait for evaluation to finish
    echo "Waiting for evaluation to finish..."
    wait $EVAL_PID
    sleep 2
    # Ensure the evaluation script is terminated
    kill -9 $EVAL_PID 2>/dev/null
    pkill -f "map_evaluation.py" 2>/dev/null
    pkill -f "cpu_plot.py" 2>/dev/null

    # Step 7: Kill all processes
    echo "Terminating processes for run $RUN_INDEX..."
    kill -9 $WEBOTS_PID $ROS2_PID 2>/dev/null
    # Ensure all related processes are terminated
    pkill -f "$WEBOTS_EXE" 2>/dev/null
    pkill -f "ros2 launch" 2>/dev/null
    pkill -f "$LOGGING_SCRIPT" 2>/dev/null

    echo "Run $RUN_INDEX completed."
    sleep 10 # Brief pause before starting next run
done

echo "All $RUN_COUNT runs completed! Check the terminal tabs and log files for output."