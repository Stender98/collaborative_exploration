import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rosgraph_msgs.msg import Clock
import argparse
import os
import time
from std_msgs.msg import String 

class MapCoverageLogger(Node):
    def __init__(self, mode, num_robots, run_count):
        super().__init__('map_coverage_logger')
        self.webots_map_width = 200
        self.webots_map_height = 200
        self.slam_map = None
        self.sim_time = 0.0
        self.last_log_time = 0.0
        self.mode = mode  # 'centralised' or 'decentralised'
        self.num_robots = num_robots
        self.run_count = run_count
        self.terminated_count = 0
        self.start_real_time = time.time()
        self.max_duration_seconds = 120  # Hard limit of 120 seconds of real time

        # Subscriber to the map topic
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Subscriber to the clock topic
        self.clock_subscription = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            10
        )

        # Subscriber to the terminated topic
        self.terminated_subscription = self.create_subscription(
            String,
            '/terminated',
            self.terminated_callback,
            10
        )

        # Create log directory and file
        self.log_dir = f"logs/{self.mode}/{self.num_robots}/{self.run_count}"
        os.makedirs(self.log_dir, exist_ok=True)
        self.log_file = os.path.join(self.log_dir, "coverage.csv")
        
        # Write header to log file
        with open(self.log_file, 'w') as f:
            f.write("Time(s),Coverage(%),Number of running robots\n")

        # Timer to log data every second (regardless of sim time)
        self.log_timer = self.create_timer(1.0, self.timed_log_coverage)
        
        # Timer to check if we've exceeded our max duration
        self.duration_check_timer = self.create_timer(1.0, self.check_duration)

        # Flag to ensure we always have exactly 120 entries
        self.entries_count = 0
        self.get_logger().info(f'Map Coverage Logger started. Mode: {mode}, Robots: {num_robots}, Run: {run_count}')

    def map_callback(self, msg):
        self.slam_map = msg

    def terminated_callback(self, msg):
        """Increment counter when a termination message is received."""
        self.terminated_count += 1
        self.get_logger().info(f"Received termination message: {msg.data}. Total terminated robots: {self.terminated_count}")

    def clock_callback(self, msg):
        self.sim_time = msg.clock.sec + msg.clock.nanosec / 1e9

    def calculate_coverage(self):
        if self.slam_map is None:
            return 0.0
        # Count explored cells (values 0 for free, 100 for occupied, -1 for unknown)
        explored_cells = sum(1 for cell in self.slam_map.data if cell != -1)
        total_cells = self.webots_map_width * self.webots_map_height
        coverage_percent = (explored_cells / total_cells) * 100 if total_cells > 0 else 0.0
        return coverage_percent

    def timed_log_coverage(self):
        """Log coverage on a regular real-time interval"""
        coverage = self.calculate_coverage()
        elapsed_time = self.entries_count + 1  # Ensure time is sequential
        
        with open(self.log_file, 'a') as f:
            f.write(f"{elapsed_time:.1f},{coverage:.2f},{self.num_robots - self.terminated_count:.0f}\n")
        
        self.entries_count += 1
        self.get_logger().debug(f"Logged entry {self.entries_count}: time={elapsed_time:.1f}, coverage={coverage:.2f}%")
        
        # If we've logged exactly 120 entries, shut down
        if self.entries_count >= 120:
            self.get_logger().info('Completed 120 logging entries. Shutting down...')
            self.destroy_node()
            rclpy.shutdown()

    def check_duration(self):
        """Check if we've exceeded the maximum run time"""
        elapsed = time.time() - self.start_real_time
        if elapsed > self.max_duration_seconds:
            # Ensure we have exactly 120 entries before shutting down
            entries_needed = 120 - self.entries_count
            if entries_needed > 0:
                self.get_logger().info(f'Time limit reached with {self.entries_count} entries. Adding {entries_needed} more entries...')
                coverage = self.calculate_coverage()
                with open(self.log_file, 'a') as f:
                    for i in range(entries_needed):
                        elapsed_time = self.entries_count + 1
                        f.write(f"{elapsed_time:.1f},{coverage:.2f},{self.num_robots - self.terminated_count:.0f}\n")
                        self.entries_count += 1
            
            self.get_logger().info(f'Maximum duration ({self.max_duration_seconds}s) reached. Shutting down with {self.entries_count} entries.')
            self.destroy_node()
            rclpy.shutdown()

def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Map Coverage Logger')
    parser.add_argument('mode', choices=['c', 'd'], help='Mode: c for centralised, d for decentralised')
    parser.add_argument('num_robots', type=int, help='Number of robots')
    parser.add_argument('run_count', type=int, help='Current run count')
    args = parser.parse_args()

    # Convert mode to full name
    mode = 'centralised' if args.mode == 'c' else 'decentralised'

    # Initialize ROS 2
    rclpy.init()

    # Create node
    map_logger = MapCoverageLogger(mode, args.num_robots, args.run_count)

    try:
        rclpy.spin(map_logger)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        map_logger.get_logger().error(f'Error occurred: {str(e)}')
    finally:
        # Ensure we have exactly 120 entries before shutting down
        if hasattr(map_logger, 'entries_count') and not map_logger.destroyed:
            entries_needed = 120 - map_logger.entries_count
            if entries_needed > 0:
                map_logger.get_logger().info(f'Shutting down with {map_logger.entries_count} entries. Adding {entries_needed} more entries...')
                coverage = map_logger.calculate_coverage() if hasattr(map_logger, 'calculate_coverage') else 0.0
                with open(map_logger.log_file, 'a') as f:
                    for i in range(entries_needed):
                        elapsed_time = map_logger.entries_count + 1
                        f.write(f"{elapsed_time:.1f},{coverage:.2f},{map_logger.num_robots - map_logger.terminated_count:.0f}\n")
                        map_logger.entries_count += 1
                
        if not map_logger.destroyed:
            map_logger.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()