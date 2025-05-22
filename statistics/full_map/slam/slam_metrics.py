import os
import pandas as pd
import numpy as np
from pathlib import Path

# Define swarm sizes and number of runs
swarm_sizes = [5, 8, 13]
num_runs = 3
approach = "centralised"

# Define the output CSV file name
output_file = approach + "_combined_descriptive_statistics_metrics.csv"

# Initialize a list to store all data across all swarm sizes and runs
all_data = []

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

# Loop through each swarm size and run to collect all data
for swarm_size in swarm_sizes:
    for run_id in range(1, num_runs + 1):
        # Construct the file path
        file_path = os.path.join("..", "..", "..", "logs", "full_map", approach, str(swarm_size), str(run_id), "metrics.csv")
        
        # Check if file exists
        if os.path.exists(file_path):
            df = process_metrics_file(file_path, swarm_size, run_id)
            if df is not None:
                all_data.append(df)
        else:
            print(f"Warning: File {file_path} not found.")

if not all_data:
    print("No data found across any swarm sizes.")
    exit()

# Combine all data into a single dataframe
combined_df = pd.concat(all_data, ignore_index=True)

# Compute overall descriptive statistics for each metric
metrics = ['mse', 'ssim', 'accuracy', 'precision', 'recall', 'f1_score', 'iou']
overall_stats = {}

for metric in metrics:
    if metric in combined_df.columns:
        metric_values = combined_df[metric].dropna().values
        if len(metric_values) > 0:
            overall_stats.update({
                f'Mean{metric.capitalize()}': np.mean(metric_values),
                f'Median{metric.capitalize()}': np.median(metric_values),
                f'StdDev{metric.capitalize()}': np.std(metric_values),
                f'Min{metric.capitalize()}': np.min(metric_values),
                f'Max{metric.capitalize()}': np.max(metric_values),
                f'Count{metric.capitalize()}': len(metric_values)
            })
        else:
            print(f"Warning: No valid data for metric {metric}.")
            overall_stats.update({
                f'Mean{metric.capitalize()}': np.nan,
                f'Median{metric.capitalize()}': np.nan,
                f'StdDev{metric.capitalize()}': np.nan,
                f'Min{metric.capitalize()}': np.nan,
                f'Max{metric.capitalize()}': np.nan,
                f'Count{metric.capitalize()}': 0
            })
    else:
        print(f"Warning: Metric {metric} not found in combined data.")

# Add total sample information
overall_stats['TotalSamples'] = len(combined_df)
overall_stats['SwarmSizes'] = str(swarm_sizes)
overall_stats['NumRuns'] = num_runs

# Create final dataframe with a single row of overall statistics
final_df = pd.DataFrame([overall_stats])

# Try to save to CSV with error handling
try:
    output_dir = os.path.dirname(output_file) or '.'
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)
    final_df.to_csv(output_file, index=False)
    print(f"Successfully wrote combined statistics to {output_file}")
except Exception as e:
    print(f"Error writing CSV file {output_file}: {e}")

# Display the output for verification
if os.path.exists(output_file):
    output_df = pd.read_csv(output_file)
    print("\nOutput CSV contents:")
    print(output_df)
else:
    print(f"\nOutput CSV file {output_file} not found.")

print("\nFinal combined dataframe:")
print(final_df)

# Optional: Display summary of data collection
print(f"\nData collection summary:")
print(f"Total samples collected: {len(combined_df)}")
print(f"Swarm sizes processed: {swarm_sizes}")
print(f"Runs per swarm size: {num_runs}")
print(f"Expected total samples: {len(swarm_sizes) * num_runs}")
if len(combined_df) < len(swarm_sizes) * num_runs:
    print(f"Note: Some files were missing or could not be processed.")