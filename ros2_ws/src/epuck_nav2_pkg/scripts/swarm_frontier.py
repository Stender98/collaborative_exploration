import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PointStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
import cv2
import tf2_ros
from tf2_ros import TransformException
import math

class FrontierExploration(Node):
    def __init__(self, robot_name: str):
        super().__init__(f'{robot_name}_frontier_exploration')  # Namespace node with robot_name
        self.robot_name = robot_name
        self.get_logger().info(f'Initializing frontier exploration for {self.robot_name}')

        # Action client for navigation (namespaced)
        self.nav2_client = ActionClient(self, NavigateToPose, f'/{self.robot_name}/navigate_to_pose')
        self.sending_goal = False
        self.last_goal_coords = None  # Track last sent goal to avoid repetition
        self.last_goal_time = None  # Track when the last goal was completed/aborted
        self.current_goal_handle = None  # Track current goal handle for cancellation
        self.cooldown_duration = 2.0  # Seconds to wait before selecting new frontier
        self.min_map_coverage = 0.01  # Reduced to 1% to avoid coverage issues
        self.frontier_check_interval = 1.0  # Seconds between frontier checks
        self.min_free_neighbors = 1  # Minimum free cells needed for valid frontier
        self.min_frontier_distance = 0.5  # Minimum distance between frontiers (meters)
        self.frontier_reserve_timeout = 10.0  # How long to consider a frontier reserved

        # TF Buffer and Listener for robot pose lookups
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriber to the map topic (shared)
        self.subscription = self.create_subscription(
            OccupancyGrid,
            f'/{self.robot_name}/global_costmap/costmap',
            self.map_callback,
            10
        )

        # Publisher for the goal topic (namespaced)
        self.goal_publisher = self.create_publisher(
            PoseStamped,
            f'/{self.robot_name}/goal',
            10
        )

        # Publisher for assigned frontiers (shared topic, using PointStamped)
        self.frontier_publisher = self.create_publisher(
            PointStamped,
            '/assigned_frontiers',
            10
        )

        # Subscriber for assigned frontiers from other robots
        self.frontier_subscription = self.create_subscription(
            PointStamped,
            '/assigned_frontiers',
            self.frontier_callback,
            10
        )

        self.map_data = None
        self.other_frontiers = {}  # Dictionary of {robot_id: (x, y, timestamp)}

        # Timer to periodically check if current goal is still a frontier
        self.check_frontier_timer = self.create_timer(
            self.frontier_check_interval,
            self.check_current_goal
        )

    def get_robot_position(self):
        """Get the robot's current position in map coordinates."""
        # Using TF to find pose.
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                f'{self.robot_name}/base_footprint',
                rclpy.time.Time()
            )
            return (transform.transform.translation.x, transform.transform.translation.y)
        except TransformException as ex:
            self.get_logger().warning(f'Could not get robot position: {ex}')
            # If we can't get the position, return the map origin as a fallback
            if self.map_data is not None:
                origin = self.map_data.info.origin
                width = self.map_data.info.width
                height = self.map_data.info.height
                resolution = self.map_data.info.resolution
                # Return the center of the map
                return (
                    origin.position.x + width * resolution / 2,
                    origin.position.y + height * resolution / 2
                )
            return (0.0, 0.0)

    def frontier_callback(self, msg):
        """Store frontiers assigned by other robots."""
        robot_id = msg.header.frame_id if msg.header.frame_id else 'unknown'
        if robot_id != self.robot_name:
            current_time = self.get_clock().now().nanoseconds / 1e9
            # Use msg.point for coordinates
            self.other_frontiers[robot_id] = (msg.point.x, msg.point.y, current_time)
            self.get_logger().debug(f'Received frontier from {robot_id} at ({msg.point.x:.2f}, {msg.point.y:.2f})')
            
            # Clean up old frontiers (older than frontier_reserve_timeout seconds)
            self.other_frontiers = {
                rid: coords
                for rid, coords in self.other_frontiers.items()
                if current_time - coords[2] < self.frontier_reserve_timeout
            }

    def publish_assigned_frontier(self, goal_coords):
        """Publish the assigned frontier to the shared topic."""
        frontier_msg = PointStamped()
        frontier_msg.header.stamp = self.get_clock().now().to_msg()
        frontier_msg.header.frame_id = self.robot_name
        frontier_msg.point.x = goal_coords[0]
        frontier_msg.point.y = goal_coords[1]
        frontier_msg.point.z = 0.0
        self.frontier_publisher.publish(frontier_msg)

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
            self.get_logger().info('Map data is None')
            return False

        # Check map coverage
        map_array = np.array(self.map_data.data, dtype=np.int8)
        known_cells = np.sum((map_array == 0) | (map_array == 100))
        total_cells = map_array.size
        coverage = known_cells / total_cells if total_cells > 0 else 0.0
        self.get_logger().debug(f'Map coverage: {coverage:.2%}')
        if coverage < self.min_map_coverage:
            self.get_logger().debug(f'Map coverage too low: {coverage:.2%} < {self.min_map_coverage:.2%}')
            return False

        # Check cooldown period
        if self.last_goal_time is not None:
            elapsed = (self.get_clock().now() - self.last_goal_time).nanoseconds / 1e9
            self.get_logger().debug(f'Cooldown elapsed: {elapsed:.2f}s')
            if elapsed < self.cooldown_duration:
                return False

        self.get_logger().info('Map is ready')
        return True

    def find_frontier(self):
        width = self.map_data.info.width
        height = self.map_data.info.height
        resolution = self.map_data.info.resolution
        origin = self.map_data.info.origin

        map_array = np.array(self.map_data.data, dtype=np.int16).reshape((height, width))
        self.get_logger().debug(f'Map stats: {np.sum(map_array == -1)} unknown, {np.sum(map_array == 0)} free, {np.sum(map_array > 0)} occupied')

        # Frontier detection: unknown (-1) adjacent to known free space (0)
        kernel = np.array([[1, 1, 1], [1, 0, 1], [1, 1, 1]], dtype=np.uint8)
        free = (map_array == 0).astype(np.uint8)  # Only consider free space
        unknown = (map_array == -1).astype(np.uint8)
        border = cv2.dilate(free, kernel, iterations=1)
        frontier = cv2.bitwise_and(border, unknown)

        frontier_coords = np.column_stack(np.where(frontier > 0))
        self.get_logger().info(f'Found {len(frontier_coords)} potential frontiers')
        if frontier_coords.size == 0:
            self.get_logger().info('No frontiers found')
            return None

        # Get current robot position
        robot_pos = np.array(self.get_robot_position())
        self.get_logger().info(f'Robot position: ({robot_pos[0]:.2f}, {robot_pos[1]:.2f})')

        # Filter frontiers
        valid_coords = []
        
        for y, x in frontier_coords:
            # Skip frontiers near obstacles (any value > 0)
            region = map_array[max(0, y-1):min(height, y+2), max(0, x-1):min(width, x+2)]
            if np.any(region > 0):
                continue

            # Check for free space neighbors
            larger_region = map_array[max(0, y-2):min(height, y+3), max(0, x-2):min(width, x+3)]
            free_count = np.sum(larger_region == 0)
            if free_count < self.min_free_neighbors:
                continue

            # Convert to world coordinates
            world_x = origin.position.x + x * resolution
            world_y = origin.position.y + y * resolution
            world_pos = np.array([world_x, world_y])

            # Skip if same as last goal
            if self.last_goal_coords and np.linalg.norm(world_pos - np.array(self.last_goal_coords)) < 0.05:
                continue

            # Skip if too close to other robots' frontiers
            too_close = False
            for other_robot, (fx, fy, _) in self.other_frontiers.items():
                if np.linalg.norm(world_pos - np.array([fx, fy])) < self.min_frontier_distance:
                    too_close = True
                    self.get_logger().debug(f'Skip frontier at ({world_x:.2f}, {world_y:.2f}): too close to {other_robot}')
                    break
            if too_close:
                continue

            # Calculate distance from robot position
            dist_to_robot = np.linalg.norm(world_pos - robot_pos)

            # Calculate angular score to favor different directions
            angular_score = 0
            if self.other_frontiers:
                frontier_vec = world_pos - robot_pos
                frontier_angle = np.arctan2(frontier_vec[1], frontier_vec[0])
                for _, (fx, fy, _) in self.other_frontiers.items():
                    other_vec = np.array([fx, fy]) - robot_pos
                    other_angle = np.arctan2(other_vec[1], other_vec[0])
                    angle_diff = np.abs((frontier_angle - other_angle + np.pi) % (2 * np.pi) - np.pi)
                    angular_score += angle_diff  # Higher score for larger angular difference
            
            valid_coords.append((y, x, world_x, world_y, free_count, dist_to_robot, angular_score))

        self.get_logger().info(f'Found {len(valid_coords)} valid frontiers after filtering')
        if not valid_coords:
            self.get_logger().info('No valid frontiers found')
            return None

        # Sort by distance to robot (ascending) as primary criterion
        # Use angular score (descending) as secondary criterion to favor different directions
        valid_coords.sort(key=lambda c: (c[5], -c[6]))
        
        # Select the closest frontier in a distinct direction
        _, _, world_x, world_y, _, dist, _ = valid_coords[0]
        self.get_logger().info(f'Selected frontier at ({world_x:.2f}, {world_y:.2f}), distance: {dist:.2f}m')
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