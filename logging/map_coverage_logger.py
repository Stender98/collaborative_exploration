import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rosgraph_msgs.msg import Clock
import argparse
import os
import time
from datetime import datetime

class MapCoverageLogger(Node):
    def __init__(self, mode, num_robots, run_count):
        super().__init__('map_coverage_logger')
        self.webots_map_width = 200
        self.webots_map_height = 200
        self.slam_map = None
        self.sim_time = 0.0
        self.last_log_time = 0.0
        self.mode = mode  # 'centralized' or 'decentralized'
        self.num_robots = num_robots
        self.run_count = run_count

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

        # Create log directory and file
        self.log_dir = f"log/{self.mode}/{self.num_robots}/{self.run_count}"
        os.makedirs(self.log_dir, exist_ok=True)
        self.log_file = os.path.join(self.log_dir, "coverage.log")
        
        # Write header to log file
        with open(self.log_file, 'w') as f:
            f.write("Time(s),Coverage(%)\n")

        # Timer to check topic availability every 5 seconds
        self.topic_check_timer = self.create_timer(5.0, self.check_topics)

    def map_callback(self, msg):
        self.slam_map = msg

    def clock_callback(self, msg):
        self.sim_time = msg.clock.sec + msg.clock.nanosec / 1e9
        # Log coverage every second
        if self.sim_time >= self.last_log_time + 1.0:
            self.log_coverage()
            self.last_log_time = self.sim_time

    def calculate_coverage(self):
        if self.slam_map is None:
            return 0.0
        # Count explored cells (values 0 for free, 100 for occupied, -1 for unknown)
        explored_cells = sum(1 for cell in self.slam_map.data if cell != -1)
        total_cells = self.webots_map_width * self.webots_map_height
        coverage_percent = (explored_cells / total_cells) * 100 if total_cells > 0 else 0.0
        return coverage_percent

    def log_coverage(self):
        coverage = self.calculate_coverage()
        with open(self.log_file, 'a') as f:
            f.write(f"{self.sim_time:.1f},{coverage:.2f}\n")

    def check_topics(self):
        # Check if /map and /clock topics have publishers
        map_publishers = self.get_publishers_info_by_topic('/map')
        clock_publishers = self.get_publishers_info_by_topic('/clock')
        
        if not map_publishers or not clock_publishers:
            self.get_logger().info('One or more topics (/map or /clock) are no longer available. Shutting down...')
            self.destroy_node()
            rclpy.shutdown()

def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Map Coverage Logger')
    parser.add_argument('mode', choices=['c', 'd'], help='Mode: c for centralized, d for decentralized')
    parser.add_argument('num_robots', type=int, help='Number of robots')
    parser.add_argument('run_count', type=int, help='Current run count')
    args = parser.parse_args()

    # Convert mode to full name
    mode = 'centralized' if args.mode == 'c' else 'decentralized'

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
        if not map_logger.destroyed:
            map_logger.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()