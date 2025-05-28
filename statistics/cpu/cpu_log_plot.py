import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import glob
from matplotlib.gridspec import GridSpec

# Set global font sizes for matplotlib
plt.rcParams.update({
    'font.size': 20,          # Base font size
    'axes.titlesize': 24,     # Title font size
    'axes.labelsize': 12,     # Axis label font size
    'xtick.labelsize': 20,    # X-axis tick label size
    'ytick.labelsize': 20,    # Y-axis tick label size
    'legend.fontsize': 20,    # Legend font size
    'figure.titlesize': 26    # Figure title font size
})

# Base directory containing all log files
base_dir = "/home/markus/MasterThesis/logs"

# Function to extract data from CSV files
def process_csv_files():
    results = {
        'centralised': {},
        'decentralised': {}
    }
    
    # Find all CSV files in the directory structure
    pattern = os.path.join(base_dir, '*', '*', '*', 'cpu_log.csv')
    csv_files = glob.glob(pattern)
    
    for file_path in csv_files:
        # Extract controller type and swarm size from path
        parts = file_path.split(os.sep)
        controller_type = parts[5].lower()  # 'centralised' or 'decentralised'
        # Extract swarm size and trial run from folder names
        try:
            swarm_size = int(parts[6])  # The swarm size folder (1, 2, 3, etc.)
        except ValueError:
            print(f"Could not determine swarm size from path: {file_path}")
            continue  # Skip if we can't determine swarm size
        
        try:
            # Read CSV file
            df = pd.read_csv(file_path, skipinitialspace=True)
            
            # Calculate median CPU usage for this file
            median_cpu = df['System_CPU(%)'].median()
            
            # Add to results
            if swarm_size not in results[controller_type]:
                results[controller_type][swarm_size] = []
            
            results[controller_type][swarm_size].append(median_cpu)
            
        except Exception as e:
            print(f"Error processing {file_path}: {e}")
    
    return results

# Function to calculate the median of medians for each swarm size
def calculate_final_medians(results):
    final_results = {
        'centralised': {},
        'decentralised': {}
    }
    
    for controller_type in results:
        for swarm_size, values in results[controller_type].items():
            final_results[controller_type][swarm_size] = np.median(values)
    
    return final_results

# Function to create plots with improved styling
def create_plots(final_results):
    # Sort swarm sizes for consistent x-axis
    swarm_sizes_centralised = sorted(final_results['centralised'].keys())
    swarm_sizes_decentralised = sorted(final_results['decentralised'].keys())
    all_swarm_sizes = sorted(set(swarm_sizes_centralised + swarm_sizes_decentralised))
    
    # Create color palette similar to the second script
    colors = plt.cm.tab10(np.linspace(0, 1, 6))
    markers = ['o', 's', 'D']  # Circle, square, diamond
    
    # Create a figure with GridSpec for better layout control
    fig = plt.figure(figsize=(16, 12))  # Increased figure size to accommodate larger fonts
    gs = GridSpec(2, 1, height_ratios=[3, 1], hspace=0.6)  # Increased spacing
    
    # Main CPU usage plot
    ax_cpu = fig.add_subplot(gs[0])
    
    # Create combined data for comparison chart
    centralised_data = {size: final_results['centralised'].get(size, None) for size in all_swarm_sizes}
    decentralised_data = {size: final_results['decentralised'].get(size, None) for size in all_swarm_sizes}
    
    # Filter out None values for plotting
    cent_x = [size for size in all_swarm_sizes if centralised_data[size] is not None]
    cent_y = [centralised_data[size] for size in cent_x]
    
    decent_x = [size for size in all_swarm_sizes if decentralised_data[size] is not None]
    decent_y = [decentralised_data[size] for size in decent_x]
    
    # Plot with better styling - removed explicit fontsize to use rcParams
    ax_cpu.plot(
        cent_x, cent_y, 
        color=colors[2], 
        marker=markers[0], 
        linewidth=3,  # Increased line width for better visibility
        markersize=10,  # Increased marker size
        label='Centralised'
    )
    
    ax_cpu.plot(
        decent_x, decent_y, 
        color=colors[4], 
        marker=markers[1], 
        linewidth=3,  # Increased line width for better visibility
        markersize=10,  # Increased marker size
        label='Decentralised'
    )
    
    # Add styling elements - removed explicit fontsize to use rcParams
    ax_cpu.set_title('Comparison of Median System CPU Usage by Swarm Size', pad=20)
    ax_cpu.set_xlabel('Swarm Size')
    ax_cpu.set_ylabel('Median System CPU Usage (%)')
    ax_cpu.grid(True, alpha=0.7)
    
    # Keep legend in top left corner as requested - removed explicit fontsize
    legend = ax_cpu.legend(loc='upper left', framealpha=0.9)
    legend.get_frame().set_linewidth(1.5)
    
    ax_cpu.set_xticks(all_swarm_sizes)
    
    # Adjust tick parameters - removed explicit labelsize to use rcParams
    ax_cpu.tick_params(axis='both', which='major', width=1.5, length=6)

    ax_cpu.xaxis.set_label_coords(0.05, -0.1)

    plt.tight_layout()
    plt.savefig('cpu_usage_comparison.png', dpi=300, bbox_inches='tight')  # Higher DPI for better quality
    
    print("Plot saved as 'cpu_usage_comparison.png'")

def main():
    print("Collecting data from CSV files...")
    results = process_csv_files()
    
    print("Calculating final medians...")
    final_results = calculate_final_medians(results)
    
    print("Creating plots with improved styling...")
    create_plots(final_results)
    
    print("Results summary:")
    for controller_type, sizes in final_results.items():
        print(f"\n{controller_type.capitalize()} controller:")
        for size, median in sorted(sizes.items()):
            print(f"  Swarm size {size}: {median:.2f}% median CPU usage")

if __name__ == "__main__":
    main()