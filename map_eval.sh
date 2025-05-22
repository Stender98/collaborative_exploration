#!/bin/bash

# Define parameters
x_values=(1 2 3 5 8 13 21)
y_values=(1 2 3 4 5 6 7 8 9 10)

# Run all combinations
for x in "${x_values[@]}"; do
  for y in "${y_values[@]}"; do
    echo "Running with x=$x, y=$y"
    python logging/map_evaluation.py d $x $y &
  done
done

# Wait for all background processes to finish
wait
echo "All processes completed!"