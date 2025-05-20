#!/bin/bash

# This script automates the execution of test.sh for all combinations of
# controllers and robot counts with 10 runs each

# Define an array of robot counts
ROBOT_COUNTS=(5 8 13)

# Define controller modes
CONTROLLER_MODES=("d" "c")

# Number of runs for each configuration
NUM_RUNS=3

echo "Starting automated experiment runs"
echo "=================================="
echo "This script will run experiments with:"
echo "- Both controller types (centralised and decentralised)"
echo "- All robot counts: ${ROBOT_COUNTS[@]}"
echo "- ${NUM_RUNS} runs for each configuration"
echo

# Calculate total number of runs
TOTAL_EXPERIMENTS=$((${#CONTROLLER_MODES[@]} * ${#ROBOT_COUNTS[@]}))
TOTAL_RUNS=$((TOTAL_EXPERIMENTS * NUM_RUNS))

echo "Total configurations: $TOTAL_EXPERIMENTS"
echo "Total runs: $TOTAL_RUNS"
echo
echo "Starting in 5 seconds... Press Ctrl+C to cancel"
sleep 5

# Function to run test.sh with specified parameters
run_test() {
    local mode="$1"
    local robots="$2"
    local runs="$3"
    
    echo
    echo "========================================================"
    echo "Running test with mode=$mode, robots=$robots, runs=$runs"
    echo "========================================================"
    
    # Use expect to automate the interactive prompts in test.sh
    expect -c "
        set timeout -1
        spawn ./test.sh
        
        # Handle config file creation if it doesn't exist
        expect {
            \"Configuration file not found.\" {
                expect \"Enter the path to your Webots executable: \"
                send \"/path/to/webots\r\"
                
                expect \"Enter the path to your Webots controller.exe: \"
                send \"/path/to/controller.exe\r\"
                
                expect \"Choose mode: centralised (c) or decentralised (d): \"
            }
            \"Choose mode: centralised (c) or decentralised (d): \" {}
        }
        
        # Enter mode
        send \"$mode\r\"
        
        # Enter number of robots
        expect \"Enter number of robots: \"
        send \"$robots\r\"
        
        # Enter number of runs
        expect \"Enter number of runs: \"
        send \"$runs\r\"
        
        # Wait for completion
        expect \"All $runs runs completed!\"
        
        # Give some time for final operations
        sleep 2
    "
    
    # Allow system to stabilize between configurations
    echo "Completed configuration. Waiting 30 seconds before next configuration..."
    sleep 30
}

# Main execution loop
CONFIG_COUNT=0

for mode in "${CONTROLLER_MODES[@]}"; do
    for robots in "${ROBOT_COUNTS[@]}"; do
        CONFIG_COUNT=$((CONFIG_COUNT + 1))
        
        echo
        echo "========================================================"
        echo "CONFIGURATION $CONFIG_COUNT of $TOTAL_EXPERIMENTS"
        echo "Mode: $([ "$mode" == "c" ] && echo "centralised" || echo "decentralised")"
        echo "Robots: $robots"
        echo "Runs: $NUM_RUNS"
        echo "========================================================"
        
        run_test "$mode" "$robots" "$NUM_RUNS"
    done
done

echo
echo "========================================================"
echo "All experiments completed!"
echo "Total configurations run: $TOTAL_EXPERIMENTS"
echo "Total runs completed: $TOTAL_RUNS"
echo "========================================================"
