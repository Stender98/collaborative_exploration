import pandas as pd
import matplotlib.pyplot as plt
import argparse
import os

parser = argparse.ArgumentParser(description='CPU and Memory Usage Plotter')
parser.add_argument('mode', choices=['c', 'd'], help='Mode: c for centralized, d for decentralized')
parser.add_argument('num_robots', type=int, help='Number of robots')
parser.add_argument('run_count', type=int, help='Current run count')
args = parser.parse_args()

current_directory = os.path.dirname(os.path.abspath(__file__))
current_directory += "/../"  # Adjust path to the parent directory

if args.mode == 'c':
    mode = 'centralised'
elif args.mode == 'd':
    mode = 'decentralised'

LOG_FILE = os.path.join(current_directory, "logs", mode, str(args.num_robots), str(args.run_count), "cpu_log.csv")
OUTPUT_FILE = os.path.join(current_directory, "logs", mode, str(args.num_robots), str(args.run_count), "cpu_log_graph.png")

# Read the CSV file with the new format
df = pd.read_csv(LOG_FILE)
df.columns = df.columns.str.strip()

# Create a figure with 2 rows and 2 columns for the plots
plt.figure(figsize=(15, 10))

# Plot 1: CPU Usage (System and Target)
ax1 = plt.subplot(2, 2, 1)
ax1.plot(df['Time(s)'], df['System_CPU(%)'], color='tab:red', label='System CPU')
ax1.plot(df['Time(s)'], df['Target_CPU(%)'], color='tab:orange', label='Target Process CPU')
ax1.set_title('CPU Usage Over Time')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('CPU (%)')
ax1.legend()
ax1.grid(True, alpha=0.3)

# Plot 2: Target Process Memory Usage
ax2 = plt.subplot(2, 2, 2)
ax2.plot(df['Time(s)'], df['Target_Memory(MB)'], color='tab:blue')
ax2.set_title('Target Process Memory Usage')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Memory (MB)')
ax2.grid(True, alpha=0.3)

# Plot 3: System Memory Usage
ax3 = plt.subplot(2, 2, 3)
ax3.plot(df['Time(s)'], df['System_Memory_Used(MB)'], color='tab:green', label='Used')
ax3.plot(df['Time(s)'], df['System_Memory_Total(MB)'], color='tab:purple', label='Total', linestyle='--')
ax3.set_title('System Memory Usage')
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Memory (MB)')
ax3.legend()
ax3.grid(True, alpha=0.3)

# Plot 4: Memory Usage as percentage of total
ax4 = plt.subplot(2, 2, 4)
memory_percentage = (df['System_Memory_Used(MB)'] / df['System_Memory_Total(MB)']) * 100
target_memory_percentage = (df['Target_Memory(MB)'] / df['System_Memory_Total(MB)']) * 100
ax4.plot(df['Time(s)'], memory_percentage, color='tab:cyan', label='System Memory %')
ax4.plot(df['Time(s)'], target_memory_percentage, color='tab:pink', label='Target Memory %')
ax4.set_title('Memory Usage (% of Total)')
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('Memory (%)')
ax4.legend()
ax4.grid(True, alpha=0.3)

plt.tight_layout()

# Save to PNG
plt.savefig(OUTPUT_FILE, dpi=300)
print(f"Plot saved to {OUTPUT_FILE}")

# Uncomment to show the plot
# plt.show()