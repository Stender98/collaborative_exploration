#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String 
from geometry_msgs.msg import Quaternion, TransformStamped, Twist, PointStamped
from rosgraph_msgs.msg import Clock
import tf_transformations
import tf2_ros
from nav2_msgs.action import NavigateToPose
from controller import Robot
import numpy as np
from swarm_frontier import FrontierExploration

# Constants
MAX_SPEED = 6  # E-puck max velocity is 6.28 rad/s
WHEEL_RADIUS = 0.0205
WHEEL_DISTANCE = 0.052
CMD_VEL_TIMEOUT = 0.5  # Seconds to consider Nav2 inactive if no /cmd_vel

# System status
DEBUG = False
ENABLE_LIDAR = True
ENABLE_CAMERA = False
ENABLE_DIST = False
STUCK_TIMEOUT = 60.0  # 1 minute timeout for no progress
MOVEMENT_THRESHOLD = 0.05  # Minimum distance (meters) to consider as progress

class EPuckController(Node):
    def __init__(self):
        self.robot = Robot()
        super().__init__('epuck_controller_' + self.robot.getName())
        self.get_logger().info("Using simulation time")

        # Webots setup
        global TIME_STEP
        TIME_STEP = int(self.robot.getBasicTimeStep())
        print("Time step is: ", TIME_STEP)

        # Initialize GPS
        self.gps = self.robot.getGPS('gps')
        if self.gps is None:
            print("Error: GPS device not found!")
        else:
            self.gps.enable(TIME_STEP)

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.robot.getTime()
        self.first_step = True

        # For tracking movement progress
        self.last_moved_time = self.robot.getTime()
        self.last_x = self.x
        self.last_y = self.y
        self.terminated = False

        # Motors
        self.left_motor = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        # Position sensors
        self.left_sensor = self.robot.getDevice('left wheel sensor')
        self.right_sensor = self.robot.getDevice('right wheel sensor')
        self.left_sensor.enable(TIME_STEP)
        self.right_sensor.enable(TIME_STEP)
        self.prev_left_pos = 0.0
        self.prev_right_pos = 0.0

        # Lidar setup
        if ENABLE_LIDAR:
            self.lidar = self.robot.getDevice("LDS-01")
            self.lidar.enable(TIME_STEP)
            self.num_rays = self.lidar.getHorizontalResolution()
            self.fov = self.lidar.getFov()
            self.angle_step = self.fov / self.num_rays
            self.get_logger().info(
                f"Lidar initialized: {self.num_rays} rays, FOV: {self.fov:.2f} rad, Angle step: {self.angle_step:.4f} rad"
            )

        # ROS 2 publishers and broadcasters
        self.odom_publisher = self.create_publisher(Odometry, self.robot.getName() + '/odom', 10)
        self.scan_publisher = self.create_publisher(LaserScan, self.robot.getName() + '/scan', 10)
        self.clock_publisher = self.create_publisher(Clock, '/clock', 10)
        self.terminated_publisher = self.create_publisher(String, '/terminated', 10) 
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscriber for Nav2 velocity commands
        self.latest_cmd_vel = Twist()
        self.last_cmd_vel_time = self.get_clock().now()
        self.create_subscription(Twist, f'/{self.robot.getName()}/cmd_vel', self.cmd_vel_callback, 10)

        # Subscriber for namespaced /goal topic
        self.create_subscription(PointStamped, f'/{self.robot.getName()}/goal', self.goal_callback, 10)

        # Action client for Nav2 (namespaced)
        self.nav2_client = ActionClient(self, NavigateToPose, f'/{self.robot.getName()}/navigate_to_pose') 

    def cmd_vel_callback(self, msg):
        """Store the latest Nav2 velocity command and update timestamp."""
        self.latest_cmd_vel = msg
        self.last_cmd_vel_time = self.get_clock().now()

    def goal_callback(self, msg):
        """Convert PointStamped to NavigateToPose goal and send to Nav2."""
        if not self.nav2_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 action server not available")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header = msg.header
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()  # Update timestamp
        goal_msg.pose.pose.position.x = msg.point.x
        goal_msg.pose.pose.position.y = msg.point.y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0  # Default orientation (yaw = 0)

        self.get_logger().info(f"Sending Nav2 goal: x={msg.point.x}, y={msg.point.y}")
        self.nav2_client.send_goal_async(goal_msg)

    def set_speed(self, left, right):
        """Set motor velocities."""
        self.left_motor.setVelocity(left)
        self.right_motor.setVelocity(right)

    def publish_clock(self, clock):
        """Publish simulation time to /clock."""
        if(self.robot.getName() == 'robot0'):
            clock_msg = Clock()
            clock_msg.clock = clock
            self.clock_publisher.publish(clock_msg)
        else:
            pass

    def set_speed(self, left, right):
        """Set motor velocities."""
        self.left_motor.setVelocity(left)
        self.right_motor.setVelocity(right)


    def publish_scan(self, clock):
        """Publish LiDAR scan data."""
        if not ENABLE_LIDAR or not hasattr(self, 'lidar'):
            return
        ranges = list(reversed(self.lidar.getRangeImage()))
        if not ranges:
            return

        msg = LaserScan()
        msg.header.stamp = clock
        msg.header.frame_id = self.robot.getName() + "/laser"
        msg.angle_min = -np.pi
        msg.angle_max = np.pi
        msg.angle_increment = np.pi * 2 / len(ranges)
        msg.range_min = 0.1
        msg.range_max = 3.5
        msg.ranges = ranges
        self.scan_publisher.publish(msg)

    def publish_odom_and_tf(self, clock):
        """Publish odometry and TF based on wheel encoder data."""
        current_time = self.robot.getTime()
        dt = current_time - self.last_time
        self.last_time = current_time

        current_left_pos = self.left_sensor.getValue()
        current_right_pos = self.right_sensor.getValue()

        # Read GPS coordinates
        gps_values = self.gps.getValues()
        if gps_values and not any(np.isnan(gps_values)):
            gps_x, gps_y = gps_values[0], gps_values[1]  # Global X, Y coordinates
        else:
            gps_x, gps_y = self.last_x, self.last_y  # Fallback to last position
            self.get_logger().warn("Invalid GPS data, using last known position.")

        if self.first_step:
            self.prev_left_pos = current_left_pos
            self.prev_right_pos = current_right_pos
            self.first_step = False
            self.last_x = gps_x
            self.last_y = gps_y
            return

        delta_left = current_left_pos - self.prev_left_pos
        delta_right = current_right_pos - self.prev_right_pos
        self.prev_left_pos = current_left_pos
        self.prev_right_pos = current_right_pos

        delta_s_left = delta_left * WHEEL_RADIUS
        delta_s_right = delta_right * WHEEL_RADIUS

        delta_s = (delta_s_right + delta_s_left) / 2.0
        delta_theta = (delta_s_right - delta_s_left) / WHEEL_DISTANCE

        if abs(delta_theta) < 1e-6:
            dx = delta_s * np.cos(self.theta)
            dy = delta_s * np.sin(self.theta)
        else:
            radius = delta_s / delta_theta
            dx = radius * (np.sin(self.theta + delta_theta) - np.sin(self.theta))
            dy = radius * (np.cos(self.theta) - np.cos(self.theta + delta_theta))

        self.x += dx if not np.isnan(dx) else 0.0
        self.y += dy if not np.isnan(dy) else 0.0
        self.theta += delta_theta if not np.isnan(delta_theta) else 0.0

        # Check movement using GPS
        distance_moved = np.sqrt((gps_x - self.last_x) ** 2 + (gps_y - self.last_y) ** 2)
        if distance_moved > MOVEMENT_THRESHOLD:
            self.last_moved_time = current_time
            self.last_x = gps_x
            self.last_y = gps_y

        # Check if stuck
        if current_time - self.last_moved_time > STUCK_TIMEOUT:
            self.get_logger().warn(f"Robot {self.robot.getName()} stuck for over {STUCK_TIMEOUT} seconds, terminating.")
            term_msg = String()
            term_msg.data = f"Robot {self.robot.getName()} terminated due to no progress."
            self.terminated_publisher.publish(term_msg)
            self.terminated = True
            return

        # Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = clock
        odom_msg.header.frame_id = self.robot.getName() + "/odom"
        odom_msg.child_frame_id = self.robot.getName() + "/base_footprint"
        odom_msg.pose.pose.position.x = self.x if not np.isnan(self.x) else 0.0
        odom_msg.pose.pose.position.y = self.y if not np.isnan(self.y) else 0.0
        odom_msg.pose.pose.position.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, self.theta if not np.isnan(self.theta) else 0.0)
        odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        odom_msg.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01
        ]
        self.odom_publisher.publish(odom_msg)

        # TF: odom -> base_footprint
        transform = TransformStamped()
        transform.header.stamp = clock
        transform.header.frame_id = self.robot.getName() + "/odom"
        transform.child_frame_id = self.robot.getName() + "/base_footprint"
        transform.transform.translation.x = self.x if not np.isnan(self.x) else 0.0
        transform.transform.translation.y = self.y if not np.isnan(self.y) else 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation = odom_msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

        # TF: base_footprint -> base_link
        base_link_tf = TransformStamped()
        base_link_tf.header.stamp = clock
        base_link_tf.header.frame_id = self.robot.getName() + "/base_footprint"
        base_link_tf.child_frame_id = self.robot.getName() + "/base_link"
        base_link_tf.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(base_link_tf)

        # TF: base_link -> laser
        laser_tf = TransformStamped()
        laser_tf.header.stamp = clock
        laser_tf.header.frame_id = self.robot.getName() + "/base_link"
        laser_tf.child_frame_id = self.robot.getName() + "/laser"
        laser_tf.transform.translation.z = 0.05
        laser_tf.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(laser_tf)

    def run(self):
        """Main control loop with automatic mode switching."""
        executor = MultiThreadedExecutor()
        executor.add_node(self)
        frontier_node = FrontierExploration(self.robot.getName())
        executor.add_node(frontier_node)

        # Initial TF and clock publish
        sim_time = self.robot.getTime()
        self.get_clock().set_ros_time_override(rclpy.time.Time(seconds=sim_time))
        clock_now = self.get_clock().now().to_msg()
        for _ in range(50):  # Publish early to ensure Nav2 sees TF and clock
            self.publish_odom_and_tf(clock_now)
            self.publish_clock(clock_now)
            executor.spin_once(timeout_sec=0.01)

        try:
            while self.robot.step(TIME_STEP) != -1:
                # Sync ROS clock with Webots sim time and get current time
                sim_time = self.robot.getTime()
                self.get_clock().set_ros_time_override(rclpy.time.Time(seconds=sim_time))
                clock_now = self.get_clock().now().to_msg()  # Use current ROS time after override

                # Publish clock, odom, and scan with consistent time
                self.publish_clock(clock_now)
                if ENABLE_LIDAR:
                    self.publish_scan(clock_now)
                self.publish_odom_and_tf(clock_now)

                left_speed = 0.0
                right_speed = 0.0

                # Check if Nav2 is active (recent /cmd_vel message)
                time_since_cmd_vel = (self.get_clock().now() - self.last_cmd_vel_time).nanoseconds / 1e9
                nav2_active = time_since_cmd_vel < CMD_VEL_TIMEOUT and \
                              (abs(self.latest_cmd_vel.linear.x) > 0.01 or abs(self.latest_cmd_vel.angular.z) > 0.01)

                if nav2_active:
                    linear = self.latest_cmd_vel.linear.x
                    angular = self.latest_cmd_vel.angular.z
                    left_speed = (linear - angular * WHEEL_DISTANCE / 2.0) / WHEEL_RADIUS
                    right_speed = (linear + angular * WHEEL_DISTANCE / 2.0) / WHEEL_RADIUS
                    left_speed = max(min(left_speed, MAX_SPEED), -MAX_SPEED)
                    right_speed = max(min(right_speed, MAX_SPEED), -MAX_SPEED)

                self.set_speed(left_speed, right_speed)
                executor.spin_once(timeout_sec=TIME_STEP / 1000.0)

                if self.terminated:
                    while True:
                        clock_now = self.get_clock().now().to_msg()
                        self.publish_clock(clock_now)
                        self.set_speed(0, 0)
                        executor.spin_once(timeout_sec=TIME_STEP / 1000.0)
                        if self.robot.step(TIME_STEP) == -1:
                            break

        except KeyboardInterrupt:
            self.get_logger().info("Controller interrupted, stopping the robot.")
        finally:
            self.cleanup()

    def cleanup(self):
        """Shutdown and cleanup."""
        self.get_logger().info("Cleaning up...")
        self.set_speed(0, 0)
        if ENABLE_LIDAR and hasattr(self, 'lidar'):
            self.lidar.disable()
        rclpy.shutdown()
        self.get_logger().info("Shutdown complete.")

def main():
    rclpy.init()
    controller = EPuckController()
    controller.run()

if __name__ == '__main__':
    main()