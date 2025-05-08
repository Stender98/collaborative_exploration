import pandas as pd
import matplotlib.pyplot as plt

# Load log file with arg
df = pd.read_csv('cpu_logger_output.csv')
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
plt.savefig("cpu_memory_usage.png", dpi=300)
# Or show the plot
# plt.show()
