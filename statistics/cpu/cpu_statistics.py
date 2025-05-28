import os
import pandas as pd
import numpy as np
from pathlib import Path

# Define swarm sizes and number of runs
swarm_sizes = [1, 2, 3, 5, 8, 13, 21]
num_runs = 10
approach = "decentralised"

# Define the output CSV file name
output_file = approach + "_descriptive_statistics_cpu_memory.csv"

# Initialize a list to store results
results = []

# Function to read and process a single cpu_log.csv file
def process_cpu_log_file(file_path, swarm_size, run_id):
    try:
        df = pd.read_csv(file_path)
        
        # Ensure columns are named correctly (strip whitespace)
        df.columns = df.columns.str.strip()
        
        # Extract the relevant CPU and Memory columns from the new format
        # Use Target_CPU(%) and Target_Memory(MB) as these appear to be the robot metrics
        df = df[['Target_CPU(%)', 'Target_Memory(MB)']].copy()
        
        # Rename columns to match expected output format
        df.rename(columns={
            'Target_CPU(%)': 'CPU(%)', 
            'Target_Memory(MB)': 'Memory(MB)'
        }, inplace=True)
        
        df['SwarmSize'] = swarm_size
        df['RunID'] = run_id
        return df
    except (pd.errors.EmptyDataError, KeyError, IndexError) as e:
        print(f"Error processing {file_path}: {e}")
        return None

# Loop through each swarm size
for swarm_size in swarm_sizes:
    # Initialize a list to store data for this swarm size
    swarm_data = []
    
    # Loop through each run
    for run_id in range(1, num_runs + 1):
        # Construct the file path (e.g., decentralised/1/1/cpu_log.csv)
        file_path = os.path.join("..", "..", "logs", approach, str(swarm_size), str(run_id), "cpu_log.csv")
        
        # Check if file exists
        if os.path.exists(file_path):
            df = process_cpu_log_file(file_path, swarm_size, run_id)
            if df is not None:
                swarm_data.append(df)
        else:
            print(f"Warning: File {file_path} not found.")
    
    if not swarm_data:
        print(f"No data found for swarm size {swarm_size}.")
        continue
    
    # Concatenate all runs for this swarm size
    combined_df = pd.concat(swarm_data, ignore_index=True)
    
    # Compute descriptive statistics for CPU and Memory across all rows and runs
    stats = {
        'SwarmSize': swarm_size,
        'MeanCPU': np.mean(combined_df['CPU(%)']),
        'MedianCPU': np.median(combined_df['CPU(%)']),
        'StdDevCPU': np.std(combined_df['CPU(%)']),
        'MinCPU': np.min(combined_df['CPU(%)']),
        'MaxCPU': np.max(combined_df['CPU(%)']),
        'CountCPU': len(combined_df['CPU(%)']),
        'MeanMemory': np.mean(combined_df['Memory(MB)']),
        'MedianMemory': np.median(combined_df['Memory(MB)']),
        'StdDevMemory': np.std(combined_df['Memory(MB)']),
        'MinMemory': np.min(combined_df['Memory(MB)']),
        'MaxMemory': np.max(combined_df['Memory(MB)']),
        'CountMemory': len(combined_df['Memory(MB)'])
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