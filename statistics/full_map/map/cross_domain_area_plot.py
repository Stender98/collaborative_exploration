import pandas as pd
import matplotlib.pyplot as plt
import os
import numpy as np
from matplotlib.gridspec import GridSpec

# List of CSV files to process
# Define swarm sizes and number of runs
swarm_sizes = [5, 8, 13, 5, 8, 13]
approaches = ["centralised", "centralised", "centralised", "decentralised", "decentralised", "decentralised"]

# Function to read and process a single CSV file
def read_csv_file(file_path):
    try:
        df = pd.read_csv(file_path)
        
        # Strip whitespace from column names
        df.columns = df.columns.str.strip()
        
        # Validate required columns
        required_columns = ['Time(s)', 'Coverage(%)', 'Number of running robots']
        if not all(col in df.columns for col in required_columns):
            print(f"Error: File {file_path} is missing required columns: {required_columns}")
            print(f"Available columns: {df.columns.tolist()}")
            return None
            
        # Normalize time to start from 0
        df['Time(s)'] = df['Time(s)'] - df['Time(s)'].min()
        return df
    except Exception as e:
        print(f"Error reading {file_path}: {str(e)}")
        return None

# Create a single figure with two subplots using GridSpec
fig = plt.figure(figsize=(14, 10))
gs = GridSpec(2, 1, height_ratios=[2, 1], hspace=0.5)

# Top subplot for coverage percentage
ax_coverage = fig.add_subplot(gs[0])

# Bottom subplot for robot count
ax_robots = fig.add_subplot(gs[1], sharex=ax_coverage)

# Set up colors and markers
colors = plt.cm.tab10(np.linspace(0, 1, len(swarm_sizes)))
markers = ['o', 's', 'D']

# Track all dataframes for statistics
all_dfs = []
legend_entries = []

# Process each CSV file
for i, swarm_size in enumerate(swarm_sizes):
    file_path = os.path.join(approaches[i], f'{approaches[i]}_swarm{swarm_size}_avg_coverage.csv')
    if not os.path.exists(file_path):
        print(f"File {file_path} not found. Skipping.")
        continue
    
    # Extract trial number for labeling
    trial_name = f"{approaches[i].capitalize()} Swarm size {swarm_size}"
    
    df = read_csv_file(file_path)
    if df is None or df.empty:
        print(f"No valid data in {file_path}. Skipping.")
        continue
    
    all_dfs.append(df)

    colorIdx = 2
    if i > 2:
        colorIdx = 4
    
    # Plot coverage percentage on top subplot
    line_coverage = ax_coverage.plot(
        df['Time(s)'],
        df['Coverage(%)'],
        color=colors[colorIdx],
        marker=markers[i % len(markers)],
        markevery=max(1, len(df)//20),
        linewidth=2,
        label=trial_name
    )
    
    # Plot robot count on bottom subplot with the same color for easy correlation
    line_robots = ax_robots.plot(
        df['Time(s)'],
        df['Number of running robots'],
        color=colors[colorIdx],
        marker=markers[i % len(markers)],
        markevery=max(1, len(df)//20),
        linewidth=2,
        label=trial_name
    )
    
    # Store legend entries
    legend_entries.append((line_coverage[0], f"{trial_name}"))

# Set labels and titles
ax_coverage.set_xlabel('Coverage (%)')
ax_coverage.set_ylabel('Time (s)', x=0.05)
ax_coverage.set_title(f'Compared Map Coverage Over Time For Both Approaches')
ax_coverage.grid(True)

ax_robots.set_xlabel('Time (s)', x=0.05)
ax_robots.set_ylabel('Number of Running Robots')
ax_robots.set_title('Active Robot Count Over Time')
ax_robots.grid(True)

# Add a single legend for both plots in the top subplot
if legend_entries:
    ax_coverage.legend([entry[0] for entry in legend_entries], 
                       [entry[1] for entry in legend_entries],
                       loc='lower right')

# Calculate and show statistics
if all_dfs:
    # Find min, max, average coverage at the end
    final_coverages = [df['Coverage(%)'].iloc[-1] for df in all_dfs]
    avg_final = sum(final_coverages) / len(final_coverages)
    min_final = min(final_coverages)
    max_final = max(final_coverages)
    
    # Add text annotation with statistics
    stat_text = f"Final Coverage: Avg={avg_final:.2f}%, Min={min_final:.2f}%, Max={max_final:.2f}%"
    ax_coverage.annotate(stat_text, xy=(0.5, -0.15), xycoords='axes fraction', 
                ha='center', fontsize=10, bbox=dict(boxstyle="round,pad=0.5", fc="white", alpha=0.8))

plt.tight_layout()
plt.savefig('./combined_robots_coverage_plot.png')
print(f"Combined plot saved as 'coverage_and_robots_plot.png'")