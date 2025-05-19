import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool  # For publishing completion
import argparse
import os
from std_msgs.msg import String 

class MapCoverageLogger(Node):
    def __init__(self, mode, num_robots, run_count):
        super().__init__('map_coverage_logger')
        self.webots_map_width = 200
        self.webots_map_height = 200
        self.slam_map = None
        self.sim_time = 0.0
        self.last_log_time = 0.0
        self.mode = mode
        self.num_robots = num_robots
        self.run_count = run_count
        self.terminated_count = 0
        self.coverage_threshold = 95.0  # 95% coverage threshold
        self.coverage_reached = False   # Flag to track completion

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
            self.terminated_calback,
            10
        )

        # Publisher for coverage completion
        self.coverage_publisher = self.create_publisher(
            Bool,
            '/coverage_complete',
            10
        )

        # Create log directory and file
        self.log_dir = f"logs/{self.mode}/{self.num_robots}/{self.run_count}"
        os.makedirs(self.log_dir, exist_ok=True)
        self.log_file = os.path.join(self.log_dir, "coverage.csv")
        
        # Write header to log file
        with open(self.log_file, 'w') as f:
            f.write("Time(s),Coverage(%),Number of running robots\n")

        # Timer to check topic availability every 5 seconds
        self.topic_check_timer = self.create_timer(5.0, self.check_topics)
        
        # Timer to continuously publish coverage complete status (if reached)
        self.coverage_timer = self.create_timer(1.0, self.publish_coverage_status)

    def map_callback(self, msg):
        self.slam_map = msg

    def terminated_calback(self, msg):
        """Increment counter when a termination message is received."""
        self.terminated_count += 1
        self.get_logger().info(f"Received termination message: {msg.data}. Total terminated robots: {self.terminated_count}")

    def clock_callback(self, msg):
        self.sim_time = msg.clock.sec + msg.clock.nanosec / 1e9
        # Log coverage every second
        if self.sim_time >= self.last_log_time + 1.0:
            self.log_coverage()
            self.last_log_time = self.sim_time

    def calculate_coverage(self):
        if self.slam_map is None:
            return 0.0
        explored_cells = sum(1 for cell in self.slam_map.data if cell != -1)
        total_cells = self.webots_map_width * self.webots_map_height
        coverage_percent = (explored_cells / total_cells) * 100 if total_cells > 0 else 0.0
        return coverage_percent

    def log_coverage(self):
        coverage = self.calculate_coverage()
        with open(self.log_file, 'a') as f:
            f.write(f"{self.sim_time:.1f},{coverage:.2f},{self.num_robots - self.terminated_count:.0f}\n")
        
        # Check if coverage threshold is reached
        if coverage >= self.coverage_threshold and not self.coverage_reached:
            self.get_logger().info(f"Reached {self.coverage_threshold}% map coverage at time {self.sim_time:.1f}s")
            self.coverage_reached = True
            
            # Publish completion message immediately
            self.publish_coverage_message()

    def publish_coverage_message(self):
        msg = Bool()
        msg.data = True
        self.coverage_publisher.publish(msg)
        self.get_logger().info("Published coverage complete message")

    def publish_coverage_status(self):
        # This will continuously publish the coverage status once reached
        if self.coverage_reached:
            self.publish_coverage_message()

    def check_topics(self):
        map_publishers = self.get_publishers_info_by_topic('/map')
        clock_publishers = self.get_publishers_info_by_topic('/clock')
        
        if not map_publishers or not clock_publishers:
            self.get_logger().info('One or more topics (/map or /clock) are no longer available. Shutting down...')
            self.destroy_node()
            rclpy.shutdown()

def main():
    parser = argparse.ArgumentParser(description='Map Coverage Logger')
    parser.add_argument('mode', choices=['c', 'd'], help='Mode: c for centralised, d for decentralised')
    parser.add_argument('num_robots', type=int, help='Number of robots')
    parser.add_argument('run_count', type=int, help='Current run count')
    args = parser.parse_args()

    mode = 'centralised' if args.mode == 'c' else 'decentralised'
    rclpy.init()
    map_logger = MapCoverageLogger(mode, args.num_robots, args.run_count)

    try:
        rclpy.spin(map_logger)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        map_logger.get_logger().error(f'Error occurred: {str(e)}')
    finally:
        map_logger.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()