import os
import pandas as pd
import numpy as np
from pathlib import Path

# Define swarm sizes and number of runs
swarm_sizes = [1, 2, 3, 5, 8, 13, 21]
num_runs = 10
approach = "centralised"

# Define the output CSV file name
output_file = approach + "_descriptive_statistics_metrics.csv"

# Initialize a list to store results
results = []

# Function to read and process a single metrics.csv file
def process_metrics_file(file_path, swarm_size, run_id):
    try:
        df = pd.read_csv(file_path)
        # Ensure columns are named correctly
        df.columns = df.columns.str.strip()
        
        # Pivot the Metric and Value columns into a single row
        df_pivot = df.pivot(columns='Metric', values='Value').reset_index(drop=True)
        df_pivot['SwarmSize'] = swarm_size
        df_pivot['RunID'] = run_id
        return df_pivot
    except (pd.errors.EmptyDataError, KeyError, IndexError) as e:
        print(f"Error processing {file_path}: {e}")
        return None

# Loop through each swarm size
for swarm_size in swarm_sizes:
    # Initialize a list to store data for this swarm size
    swarm_data = []
    
    # Loop through each run
    for run_id in range(1, num_runs + 1):
        # Construct the file path
        file_path = os.path.join("..", "..", "logs", approach, str(swarm_size), str(run_id), "metrics.csv")
        
        # Check if file exists
        if os.path.exists(file_path):
            df = process_metrics_file(file_path, swarm_size, run_id)
            if df is not None:
                swarm_data.append(df)
        else:
            print(f"Warning: File {file_path} not found.")
    
    if not swarm_data:
        print(f"No data found for swarm size {swarm_size}.")
        continue
    
    # Concatenate all runs for this swarm size
    combined_df = pd.concat(swarm_data, ignore_index=True)
    
    # Compute descriptive statistics for each metric
    metrics = ['mse', 'ssim', 'accuracy', 'precision', 'recall', 'f1_score', 'iou']
    stats = {'SwarmSize': swarm_size}
    
    for metric in metrics:
        if metric in combined_df.columns:
            metric_values = combined_df[metric].dropna().values
            if len(metric_values) > 0:
                stats.update({
                    f'Mean{metric.capitalize()}': np.mean(metric_values),
                    f'Median{metric.capitalize()}': np.median(metric_values),
                    f'StdDev{metric.capitalize()}': np.std(metric_values),
                    f'Min{metric.capitalize()}': np.min(metric_values),
                    f'Max{metric.capitalize()}': np.max(metric_values),
                    f'Count{metric.capitalize()}': len(metric_values)
                })
            else:
                print(f"Warning: No valid data for metric {metric} for swarm size {swarm_size}.")
                stats.update({
                    f'Mean{metric.capitalize()}': np.nan,
                    f'Median{metric.capitalize()}': np.nan,
                    f'StdDev{metric.capitalize()}': np.nan,
                    f'Min{metric.capitalize()}': np.nan,
                    f'Max{metric.capitalize()}': np.nan,
                    f'Count{metric.capitalize()}': 0
                })
        else:
            print(f"Warning: Metric {metric} not found in data for swarm size {swarm_size}.")
    
    results.append(pd.DataFrame([stats]))

# Combine all results into a single dataframe
if results:
    final_df = pd.concat(results, ignore_index=True)
    # Sort by SwarmSize for clarity
    final_df = final_df.sort_values('SwarmSize')
    
    # Try to save to CSV with error handling
    try:
        output_dir = os.path.dirname(output_file) or '.'
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir)
        final_df.to_csv(output_file, index=False)
        print(f"Successfully wrote statistics to {output_file}")
    except Exception as e:
        print(f"Error writing CSV file {output_file}: {e}")
else:
    print("No data processed. Check folder structure and file paths.")

# Display the output for verification
if os.path.exists(output_file):
    output_df = pd.read_csv(output_file)
    print("\nOutput CSV contents:")
    print(output_df)
else:
    print(f"\nOutput CSV file {output_file} not found.")

if results:
    print("\nFinal dataframe:")
    print(final_df)