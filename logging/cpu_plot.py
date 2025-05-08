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

LOG_FILE = os.path.join(current_directory, "logs", mode, args.num_robots, args.run_count, "cpu_log.csv")
OUTPUT_FILE = os.path.join(current_directory, "logs", mode, args.num_robots, args.run_count, "cpu_log_graph.png")

df = pd.read_csv(LOG_FILE)
df.columns = df.columns.str.strip()

plt.figure(figsize=(12, 5))

plt.subplot(1, 2, 1)
plt.plot(df['Time(s)'], df['CPU(%)'], color='tab:red')
plt.title('CPU Usage Over Time')
plt.xlabel('Time (s)')
plt.ylabel('CPU (%)')

plt.subplot(1, 2, 2)
plt.plot(df['Time(s)'], df['Memory(MB)'], color='tab:blue')
plt.title('Memory Usage Over Time')
plt.xlabel('Time (s)')
plt.ylabel('Memory (MB)')

plt.tight_layout()

# Save to PNG
plt.savefig(OUTPUT_FILE, dpi=300)
# Or show the plot
# plt.show()
