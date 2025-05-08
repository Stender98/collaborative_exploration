import psutil
import time
import argparse
import os

parser = argparse.ArgumentParser(description='CPU and Memory Usage Logger')
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

TARGET_NAME = "swarm" # Target all processes with "webots" in their command line
LOG_FILE = os.path.join(current_directory, "logs", mode, str(args.num_robots), str(args.run_count), "cpu_log.csv")

# Ensure the directory exists
os.makedirs(os.path.dirname(LOG_FILE), exist_ok=True)

print(f"Monitoring processes matching: {TARGET_NAME}")
print(f"Logging to: {LOG_FILE}")

try:
    with open(LOG_FILE, "w") as f:
        f.write("Time(s), CPU(%), Memory(MB)\n")
        f.flush()  # Flush the header immediately
        start = time.time()

        while True:
            total_cpu = 0.0
            total_mem = 0.0
            for proc in psutil.process_iter(['name', 'cmdline', 'cpu_percent', 'memory_info']):
                try:
                    cmdline = proc.info.get('cmdline')
                    if cmdline and isinstance(cmdline, list) and TARGET_NAME in ' '.join(cmdline):
                        total_cpu += proc.cpu_percent(0.1)
                        total_mem += proc.memory_info().rss / 1024 / 1024  # MB
                except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                    continue
            
            elapsed = time.time() - start
            line = f"{elapsed:.2f}, {total_cpu:.2f}, {total_mem:.2f}\n"
            f.write(line)
            f.flush()  # Force writing to disk after each measurement
            time.sleep(1)
            
except KeyboardInterrupt:
    print("CPU logging stopped by user.")
except Exception as e:
    print(f"Error occurred: {e}")
finally:
    print("Logging complete. Data saved to:", LOG_FILE)