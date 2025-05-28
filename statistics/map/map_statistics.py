import os
import pandas as pd
import numpy as np
from pathlib import Path

# Define swarm sizes and number of runs
swarm_sizes = [1, 2, 3, 5, 8, 13, 21]
num_runs = 10
approach = "centralised"

# Define the output CSV file name
output_file = approach + "_descriptive_statistics_map_coverage.csv"

# Initialize a list to store results
results = []

# Function to read and process a single coverage.csv file
def process_coverage_file(file_path, swarm_size, run_id):
    try:
        df = pd.read_csv(file_path)
        # Ensure columns are named correctly
        df.columns = df.columns.str.strip()
        # Extract the last row's coverage
        final_coverage = df['Coverage(%)'].iloc[-1]
        return pd.DataFrame({
            'SwarmSize': [swarm_size],
            'RunID': [run_id],
            'FinalCoverage': [final_coverage]
        })
    except (pd.errors.EmptyDataError, KeyError, IndexError) as e:
        print(f"Error processing {file_path}: {e}")
        return None

# Loop through each swarm size
for swarm_size in swarm_sizes:
    # Initialize a list to store final coverage values for this swarm size
    swarm_data = []
    
    # Loop through each run
    for run_id in range(1, num_runs + 1):
        # Construct the file path (e.g., 1/1/coverage.csv)
        file_path = os.path.join("..", "..", "logs", approach, str(swarm_size), str(run_id), "coverage.csv")
        
        # Check if file exists
        if os.path.exists(file_path):
            df = process_coverage_file(file_path, swarm_size, run_id)
            if df is not None:
                swarm_data.append(df)
        else:
            print(f"Warning: File {file_path} not found.")
    
    if not swarm_data:
        print(f"No data found for swarm size {swarm_size}.")
        continue
    
    # Concatenate all runs for this swarm size
    combined_df = pd.concat(swarm_data, ignore_index=True)
    
    # Compute descriptive statistics for final coverage across all runs
    stats = {
        'SwarmSize': swarm_size,
        'Mean': np.mean(combined_df['FinalCoverage']),
        'Median': np.median(combined_df['FinalCoverage']),
        'StdDev': np.std(combined_df['FinalCoverage']),
        'Min': np.min(combined_df['FinalCoverage']),
        'Max': np.max(combined_df['FinalCoverage']),
        'Count': len(combined_df['FinalCoverage'])
    }
    
    results.append(pd.DataFrame([stats]))

# Combine all results into a single dataframe
if results:
    final_df = pd.concat(results, ignore_index=True)
    
    # Sort by SwarmSize for clarity
    final_df = final_df.sort_values('SwarmSize')
    
    # Save to CSV
    final_df.to_csv(output_file, index=False)
    print(f"Descriptive statistics saved to {output_file}")
else:
    print("No data processed. Check folder structure and file paths.")

# Display the output for verification
if os.path.exists(output_file):
    output_df = pd.read_csv(output_file)
    print("\nOutput statistics:")
    print(output_df)