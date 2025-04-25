import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
import cv2
from std_msgs.msg import String
import json
from tf2_msgs.msg import TFMessage
import tf_transformations

class FrontierExploration(Node):
    def __init__(self):
        super().__init__('frontier_exploration')
        # Declare and get robot_id parameter for namespacing
        self.declare_parameter('robot_id', 'robot_0')
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        self.get_logger().info(f'Initializing frontier exploration for {self.robot_id}')

        # Action client for navigation (namespaced)
        self.nav2_client = ActionClient(self, NavigateToPose, f'/{self.robot_id}/navigate_to_pose')
        self.sending_goal = False
        self.last_goal_coords = None  # Track last sent goal to avoid repetition
        self.last_goal_time = None  # Track when the last goal was completed/aborted
        self.current_goal_handle = None  # Track current goal handle for cancellation
        self.cooldown_duration = 2.0  # Seconds to wait before selecting new frontier
        self.min_map_coverage = 0.1  # Minimum fraction of known map cells (0 to 1)
        self.frontier_check_interval = 1.0  # Seconds between frontier checks
        self.min_free_neighbors = 3  # Minimum number of free space neighbors for a frontier
        self.min_frontier_distance = 0.5  # Minimum distance between frontiers (meters)
        self.direction_weight = 0.5  # Weight for directional diversity in frontier scoring

        # Subscriber to the shared map topic
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Publisher for the goal topic (namespaced)
        self.goal_publisher = self.create_publisher(
            PoseStamped,
            f'/{self.robot_id}/goal',
            10
        )

        # Publisher for assigned frontiers (shared topic)
        self.frontier_publisher = self.create_publisher(
            Point,
            '/assigned_frontiers',
            10
        )

        # Subscriber for assigned frontiers from other robots
        self.frontier_subscription = self.create_subscription(
            Point,
            '/assigned_frontiers',
            self.frontier_callback,
            10
        )

        # Subscriber for robot's pose (using TF for simplicity)
        self.tf_subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10
        )

        self.map_data = None
        self.other_frontiers = {}  # Dictionary of {robot_id: (x, y, timestamp)}
        self.current_pose = None  # Current robot pose (x, y, yaw)

        # Timer to periodically check if current goal is still a frontier
        self.check_frontier_timer = self.create_timer(
            self.frontier_check_interval,
            self.check_current_goal
        )

    def tf_callback(self, msg):
        """Update current robot pose from TF messages."""
        for transform in msg.transforms:
            if transform.child_frame_id == f'{self.robot_id}/base_link' and transform.header.frame_id == 'map':
                t = transform.transform.translation
                q = transform.transform.rotation
                self.current_pose = (
                    t.x,
                    t.y,
                    tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]  # Yaw
                )
                break

    def frontier_callback(self, msg):
        """Store frontiers assigned by other robots."""
        # Point message uses x for frontier x, y for frontier y, z for timestamp
        # We use the header.frame_id to store the robot_id
        robot_id = msg._frame_id if msg._frame_id else 'unknown'
        if robot_id != self.robot_id:
            self.other_frontiers[robot_id] = (msg.x, msg.y, msg.z)
            # Clean up old frontiers (older than 30 seconds)
            current_time = self.get_clock().now().nanoseconds / 1e9
            self.other_frontiers = {
                rid: coords
                for rid, coords in self.other_frontiers.items()
                if current_time - coords[2] < 30.0
            }

    def publish_assigned_frontier(self, goal_coords):
        """Publish the assigned frontier to the shared topic."""
        frontier_msg = Point()
        frontier_msg.x = goal_coords[0]
        frontier_msg.y = goal_coords[1]
        frontier_msg.z = self.get_clock().now().nanoseconds / 1e9  # Timestamp
        frontier_msg._frame_id = self.robot_id  # Use frame_id to store robot_id
        self.frontier_publisher.publish(frontier_msg)

    def map_callback(self, msg):
        self.get_logger().info('Received map update')
        self.map_data = msg
        if not self.sending_goal and self.is_map_ready():
            frontier = self.find_frontier()
            if frontier is not None:
                self.send_goal(frontier)

    def is_map_ready(self):
        """Check if enough map data is available, cooldown period has passed, and pose is known."""
        if self.map_data is None or self.current_pose is None:
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
        kernel = np.array([[1, 1, 1], [1, 0, 1], [1, 1, 1]], dtype=np.uint8)
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
        for y, x in frontier_coords:
            # Skip frontiers near obstacles
            region = map_array[max(0, y-1):min(height, y+2), max(0, x-1):min(width, x+2)]
            if np.any(region == 100):
                continue

            # Check for free space neighbors
            larger_region = map_array[max(0, y-2):min(height, y+3), max(0, x-2):min(width, x+3)]
            free_count = np.sum(larger_region == 0)
            if free_count < self.min_free_neighbors:
                continue  # Skip if not enough free space neighbors

            # Convert to world coordinates
            world_x = origin.position.x + x * resolution
            world_y = origin.position.y + y * resolution

            # Skip if too close to current position
            dist = np.hypot(world_x - self.current_pose[0], world_y - self.current_pose[1])
            if dist < min_distance:
                continue

            # Skip if same as last goal
            if self.last_goal_coords and np.hypot(world_x - self.last_goal_coords[0],
                                                 world_y - self.last_goal_coords[1]) < 0.05:
                continue

            # Skip if too close to other robots' frontiers
            too_close = False
            for _, (fx, fy, _) in self.other_frontiers.items():
                if np.hypot(world_x - fx, world_y - fy) < self.min_frontier_distance:
                    too_close = True
                    break
            if too_close:
                continue

            # Calculate direction score (promote angular diversity)
            direction_score = 0.0
            if self.other_frontiers:
                frontier_vec = np.array([world_x - self.current_pose[0], world_y - self.current_pose[1]])
                frontier_vec /= np.linalg.norm(frontier_vec) if np.linalg.norm(frontier_vec) > 0 else np.array([1, 0])
                angles = []
                for _, (fx, fy, _) in self.other_frontiers.items():
                    other_vec = np.array([fx - self.current_pose[0], fy - self.current_pose[1]])
                    other_vec /= np.linalg.norm(other_vec) if np.linalg.norm(other_vec) > 0 else np.array([1, 0])
                    cos_angle = np.dot(frontier_vec, other_vec)
                    angles.append(np.arccos(np.clip(cos_angle, -1.0, 1.0)))
                direction_score = np.max(angles) if angles else np.pi  # Maximize angle difference

            valid_coords.append((y, x, world_x, world_y, free_count, direction_score))

        if not valid_coords:
            self.get_logger().info('No valid frontiers with sufficient free space found')
            return None

        # Sort by a combination of free space neighbors and direction score
        valid_coords.sort(key=lambda c: (
            -c[4],  # Prioritize free space neighbors
            -c[5] * self.direction_weight,  # Then directional diversity
            (c[2] - self.current_pose[0])**2 + (c[3] - self.current_pose[1])**2  # Then distance
        ))

        # Select the top frontier
        _, _, world_x, world_y, _, _ = valid_coords[0]
        return (world_x, world_y)

    def is_goal_still_frontier(self, goal_coords):
        """Check if the goal coordinates are still a frontier."""
        if self.map_data is None or goal_coords is None:
            return False

        width = self.map_data.info.width
        height = self.map_data.info.height
        resolution = self.map_data.info.resolution
        origin = self.map_data.info.origin

        # Convert world coordinates to map indices
        x_idx = int((goal_coords[0] - origin.position.x) / resolution)
        y_idx = int((goal_coords[1] - origin.position.y) / resolution)

        # Check if indices are within map bounds
        if not (0 <= x_idx < width and 0 <= y_idx < height):
            self.get_logger().info('Goal coordinates out of map bounds')
            return False

        map_array = np.array(self.map_data.data, dtype=np.int8).reshape((height, width))

        # Check a small region around the goal
        region = map_array[max(0, y_idx-1):min(height, y_idx+2), max(0, x_idx-1):min(width, x_idx+2)]
        is_unknown = np.any(region == -1)  # Check for unknown cells
        is_free_nearby = np.any(map_array[max(0, y_idx-2):min(height, y_idx+3),
                                         max(0, x_idx-2):min(width, x_idx+3)] == 0)  # Check for free space nearby

        return is_unknown and is_free_nearby

    def check_current_goal(self):
        """Periodically check if the current goal is still a frontier."""
        if not self.sending_goal or self.current_goal_handle is None or self.last_goal_coords is None:
            return

        if not self.is_goal_still_frontier(self.last_goal_coords):
            self.get_logger().info('Current goal is no longer a frontier. Canceling goal.')
            self.current_goal_handle.cancel_goal_async()
            self.sending_goal = False
            self.last_goal_time = self.get_clock().now()
            self.current_goal_handle = None
            # Try to find a new frontier immediately
            if self.map_data is not None and self.is_map_ready():
                frontier = self.find_frontier()
                if frontier is not None:
                    self.send_goal(frontier)

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
        self.publish_assigned_frontier(goal_coords)  # Publish the assigned frontier

        send_goal_future = self.nav2_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2')
            self.sending_goal = False
            self.last_goal_time = self.get_clock().now()
            self.current_goal_handle = None
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        status = future.result().status
        self.sending_goal = False
        self.last_goal_time = self.get_clock().now()
        self.current_goal_handle = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal reached successfully')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Goal canceled')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn('Goal aborted')
        else:
            self.get_logger().warn(f'Goal failed with status {status}')

        # Try next frontier if map data is available and ready
        if self.map_data is not None and self.is_map_ready():
            frontier = self.find_frontier()
            if frontier is not None:
                self.send_goal(frontier)