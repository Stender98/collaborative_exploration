import psutil
import time

TARGET_NAME = "swarm_decentralised"  # Change with arg
LOG_FILE = "cpu_logger_output.csv"

print(f"Monitoring processes matching: {TARGET_NAME}")
print(f"Logging to: {LOG_FILE}")

with open(LOG_FILE, "w") as f:
    f.write("Time(s), CPU(%), Memory(MB)\n")
    start = time.time()

    try:
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
            f.write(f"{elapsed:.2f}, {total_cpu:.2f}, {total_mem:.2f}\n")
            time.sleep(1)
    except KeyboardInterrupt:
        print("CPU logging stopped.")
