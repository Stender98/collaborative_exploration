import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import cv2
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.time import Time

class FrontierExploration(Node):
    def __init__(self):
        super().__init__('frontier_exploration')
        self.nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.sending_goal = False
        self.last_goal_coords = None  # Track last sent goal to avoid repetition
        self.last_goal_time = None  # Track when the last goal was completed/aborted
        self.cooldown_duration = 2.0  # Seconds to wait before selecting new frontier
        self.min_map_coverage = 0.1  # Minimum fraction of known map cells (0 to 1)

        # Subscriber to the map topic
        self.subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10
        )

        # Publisher to the goal topic
        self.goal_publisher = self.create_publisher(
            PoseStamped,
            'goal',
            10
        )

        self.map_data = None

    def map_callback(self, msg):
        self.get_logger().info('Received map update')
        self.map_data = msg
        if not self.sending_goal and self.is_map_ready():
            frontier = self.find_frontier()
            if frontier is not None:
                self.send_goal(frontier)

    def is_map_ready(self):
        """Check if enough map data is available and cooldown period has passed."""
        if self.map_data is None:
            return False

        # Check map coverage
        map_array = np.array(self.map_data.data, dtype=np.int8)
        known_cells = np.sum((map_array == 0) | (map_array == 100))
        total_cells = map_array.size
        coverage = known_cells / total_cells if total_cells > 0 else 0.0
        if coverage < self.min_map_coverage:
            self.get_logger().info(f'Map coverage too low: {coverage:.2%} < {self.min_map_coverage:.2%}')
            return False

        # Check cooldown period
        if self.last_goal_time is not None:
            elapsed = (self.get_clock().now() - self.last_goal_time).nanoseconds / 1e9
            if elapsed < self.cooldown_duration:
                self.get_logger().info(f'Waiting for cooldown: {elapsed:.2f}/{self.cooldown_duration}s')
                return False

        return True

    def find_frontier(self):
        width = self.map_data.info.width
        height = self.map_data.info.height
        resolution = self.map_data.info.resolution
        origin = self.map_data.info.origin

        map_array = np.array(self.map_data.data, dtype=np.int8).reshape((height, width))

        # Frontier detection: unknown (-1) adjacent to known free space (0)
        kernel = np.array([[1,1,1],[1,0,1],[1,1,1]], dtype=np.uint8)
        free = (map_array == 0).astype(np.uint8)  # Only consider free space
        unknown = (map_array == -1).astype(np.uint8)
        border = cv2.dilate(free, kernel, iterations=1)
        frontier = cv2.bitwise_and(border, unknown)

        frontier_coords = np.column_stack(np.where(frontier > 0))
        if frontier_coords.size == 0:
            self.get_logger().info('No frontiers found')
            return None

        # Filter frontiers
        valid_coords = []
        min_distance = 0.1  # Minimum distance from current position (meters)
        center_x = width // 2
        center_y = height // 2
        for y, x in frontier_coords:
            # Skip frontiers near obstacles
            region = map_array[max(0, y-1):min(height, y+2), max(0, x-1):min(width, x+2)]
            if np.any(region == 100):
                continue

            # Convert to world coordinates
            world_x = origin.position.x + x * resolution
            world_y = origin.position.y + y * resolution

            # Skip if too close to current position
            dist = np.hypot(world_x - (origin.position.x + center_x * resolution),
                          world_y - (origin.position.y + center_y * resolution))
            if dist < min_distance:
                continue

            # Skip if same as last goal
            if self.last_goal_coords and np.hypot(world_x - self.last_goal_coords[0],
                                                world_y - self.last_goal_coords[1]) < 0.05:
                continue

            valid_coords.append((y, x, world_x, world_y))

        if not valid_coords:
            self.get_logger().info('No valid frontiers found')
            return None

        # Sort by distance to center (Euclidean distance)
        valid_coords.sort(key=lambda c: (c[2] - (origin.position.x + center_x * resolution))**2 +
                                       (c[3] - (origin.position.y + center_y * resolution))**2)

        # Select closest valid frontier
        _, _, world_x, world_y = valid_coords[0]
        return (world_x, world_y)

    def send_goal(self, goal_coords):
        if not self.nav2_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Nav2 server not available')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_coords[0]
        goal_msg.pose.pose.position.y = goal_coords[1]
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Sending goal to ({goal_coords[0]:.2f}, {goal_coords[1]:.2f})')
        self.sending_goal = True
        self.last_goal_coords = goal_coords

        send_goal_future = self.nav2_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2')
            self.sending_goal = False
            self.last_goal_time = self.get_clock().now()
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        status = future.result().status
        self.sending_goal = False
        self.last_goal_time = self.get_clock().now()

        if status == 4:  # Aborted
            self.get_logger().warn('Goal aborted. Trying next frontier')
        else:
            self.get_logger().info('Goal completed')

        # Try next frontier if map data is available and ready
        if self.map_data is not None and self.is_map_ready():
            frontier = self.find_frontier()
            if frontier is not None:
                self.send_goal(frontier)