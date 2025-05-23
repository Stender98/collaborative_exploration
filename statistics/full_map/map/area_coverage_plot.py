import pandas as pd
import matplotlib.pyplot as plt
import os
import numpy as np
from matplotlib.gridspec import GridSpec

# Set global font sizes for matplotlib
plt.rcParams.update({
    'font.size': 20,          # Base font size
    'axes.titlesize': 24,     # Title font size
    'axes.labelsize': 16,     # Axis label font size
    'xtick.labelsize': 20,    # X-axis tick label size
    'ytick.labelsize': 20,    # Y-axis tick label size
    'legend.fontsize': 20,    # Legend font size
    'figure.titlesize': 26    # Figure title font size
})

# List of CSV files to process
# Define swarm sizes and number of runs
swarm_sizes = [5, 8, 13]
num_runs = 3
approach = "decentralised"

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
fig = plt.figure(figsize=(16, 12))  # Increased figure size to accommodate larger fonts
gs = GridSpec(2, 1, height_ratios=[2, 1], hspace=0.6)  # Increased spacing

# Top subplot for coverage percentage
ax_coverage = fig.add_subplot(gs[0])

# Bottom subplot for robot count
ax_robots = fig.add_subplot(gs[1], sharex=ax_coverage)

# Set up colors and markers
colors = plt.cm.tab10(np.linspace(0, 1, len(swarm_sizes)))
markers = ['o', 's', 'D', '^', 'v', '<', '>', 'p', '*', 'h']

# Track all dataframes for statistics
all_dfs = []
legend_entries = []

# Process each CSV file
for i, swarm_size in enumerate(swarm_sizes):
    file_path = os.path.join(approach, f'{approach}_swarm{swarm_size}_avg_coverage.csv')
    if not os.path.exists(file_path):
        print(f"File {file_path} not found. Skipping.")
        continue
    
    # Extract trial number for labeling
    trial_name = f"Swarm size {swarm_size}"
    
    df = read_csv_file(file_path)
    if df is None or df.empty:
        print(f"No valid data in {file_path}. Skipping.")
        continue
    
    all_dfs.append(df)
    
    # Plot coverage percentage on top subplot
    line_coverage = ax_coverage.plot(
        df['Time(s)'],
        df['Coverage(%)'],
        color=colors[i],
        marker=markers[i % len(markers)],
        markevery=max(1, len(df)//20),
        linewidth=3,  # Increased line width for better visibility
        markersize=8,  # Increased marker size
        label=trial_name
    )
    
    # Plot robot count on bottom subplot with the same color for easy correlation
    line_robots = ax_robots.plot(
        df['Time(s)'],
        df['Number of running robots'],
        color=colors[i],
        marker=markers[i % len(markers)],
        markevery=max(1, len(df)//20),
        linewidth=3,  # Increased line width for better visibility
        markersize=8,  # Increased marker size
        label=trial_name
    )
    
    # Store legend entries
    legend_entries.append((line_coverage[0], f"{trial_name}"))

# Set labels and titles - removed explicit fontsize to use rcParams
ax_coverage.set_ylabel('Coverage (%)')
ax_coverage.set_xlabel('Time (s)')
ax_coverage.set_title(f'Map Coverage Over Time For {approach.capitalize()} Approach', pad=20)
ax_coverage.grid(True, alpha=0.7)

ax_robots.set_xlabel('Time (s)')
ax_robots.set_ylabel('Number of Running Robots')
ax_robots.set_title('Active Robot Count Over Time', pad=20)
ax_robots.grid(True, alpha=0.7)

ax_coverage.xaxis.set_label_coords(0.05, -0.1)
ax_robots.xaxis.set_label_coords(0.05, -0.2)

# Add a single legend for both plots in the top subplot - removed explicit fontsize
if legend_entries:
    legend = ax_coverage.legend([entry[0] for entry in legend_entries], 
                               [entry[1] for entry in legend_entries],
                               loc='lower right', framealpha=0.9)
    legend.get_frame().set_linewidth(1.5)

# Calculate and show statistics
if all_dfs:
    # Find min, max, average coverage at the end
    final_coverages = [df['Coverage(%)'].iloc[-1] for df in all_dfs]
    avg_final = sum(final_coverages) / len(final_coverages)
    min_final = min(final_coverages)
    max_final = max(final_coverages)
    
    # Add text annotation with statistics - removed explicit fontsize to use base font
    stat_text = f"Final Coverage: Avg={avg_final:.2f}%, Min={min_final:.2f}%, Max={max_final:.2f}%"
    ax_coverage.annotate(stat_text, xy=(0.5, -0.25), xycoords='axes fraction', 
                ha='center', 
                bbox=dict(boxstyle="round,pad=0.8", fc="white", alpha=0.9, edgecolor='black'))

# Adjust tick parameters - removed explicit labelsize to use rcParams
ax_coverage.tick_params(axis='both', which='major', width=1.5, length=6)
ax_robots.tick_params(axis='both', which='major', width=1.5, length=6)

plt.tight_layout()
plt.savefig(f'{approach}/robots_coverage_plot.png', dpi=300, bbox_inches='tight')  # Higher DPI for better quality
print(f"Combined plot saved as 'robots_coverage_plot.png'")