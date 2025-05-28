import os
import pandas as pd
import numpy as np
from pathlib import Path

# Define swarm sizes and number of runs
swarm_sizes = [5, 8, 13]
num_runs = 3
approach = "decentralised"

# Function to read and process a single coverage.csv file
def read_coverage_file(file_path):
    try:
        df = pd.read_csv(file_path)
        # Ensure columns are named correctly
        df.columns = df.columns.str.strip()
        return df
    except (pd.errors.EmptyDataError, KeyError, IndexError) as e:
        print(f"Error reading {file_path}: {e}")
        return None

# Process each swarm size
for swarm_size in swarm_sizes:
    print(f"Processing swarm size: {swarm_size}")
    
    # List to store dataframes from all runs
    all_runs_data = []
    
    # Collect data from all runs
    for run_id in range(1, num_runs + 1):
        file_path = os.path.join("../../..", "logs", "full_map", approach, str(swarm_size), str(run_id), "coverage.csv")
        
        if os.path.exists(file_path):
            df = read_coverage_file(file_path)
            if df is not None:
                # Add run identifier for tracking
                df['RunID'] = run_id
                all_runs_data.append(df)
        else:
            print(f"Warning: File {file_path} not found.")
    
    if not all_runs_data:
        print(f"No data found for swarm size {swarm_size}. Skipping.")
        continue
    
    # Combine all runs data
    combined_data = pd.concat(all_runs_data, ignore_index=True)
    
    # Handle time normalization
    # Option 1: Use the original timestamps (may not align well across runs)
    # Option 2: Normalize to relative time (starting from 0 for each run)
    # Option 3: Use sequence number (1, 2, 3, ...) instead of time
    
    # Let's go with Option 2 - normalize to relative time
    # First, find the minimum time for each run and subtract it
    min_times = combined_data.groupby('RunID')['Time(s)'].transform('min')
    combined_data['RelativeTime'] = combined_data['Time(s)'] - min_times
    
    # Round to nearest second for binning
    combined_data['TimeRounded'] = combined_data['RelativeTime'].round(0)
    
    # Group by the rounded time and calculate averages
    avg_data = combined_data.groupby('TimeRounded').agg({
        'Coverage(%)': 'mean',
        'Number of running robots': 'mean'
    }).reset_index()
    
    # Rename columns for clarity
    avg_data.columns = ['Time(s)', 'Coverage(%)', 'Number of running robots']
    
    # Sort by time
    avg_data = avg_data.sort_values('Time(s)')
    
    # Create output directory if it doesn't exist
    output_dir = os.path.join(approach)
    os.makedirs(output_dir, exist_ok=True)
    
    # Save to CSV
    output_file = os.path.join(output_dir, f"{approach}_swarm{swarm_size}_avg_coverage.csv")
    avg_data.to_csv(output_file, index=False)
    
    print(f"Average coverage data for swarm size {swarm_size} saved to {output_file}")
    print(f"Data shape: {avg_data.shape}")
    print(f"Time range: {avg_data['Time(s)'].min()} to {avg_data['Time(s)'].max()}")
    print(f"Final average coverage: {avg_data['Coverage(%)'].iloc[-1]:.2f}%")
    print("-" * 50)

print("Processing complete.")