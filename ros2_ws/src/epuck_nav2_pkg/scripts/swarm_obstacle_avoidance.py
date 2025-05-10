from controller import Robot

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String 
from geometry_msgs.msg import Quaternion, TransformStamped
from rosgraph_msgs.msg import Clock
import tf_transformations
import tf2_ros

import numpy as np
import random

# Constants
MAX_SPEED = 6.28  # E-puck max velocity is 6.28 rad/s
WHEEL_RADIUS = 0.0205
WHEEL_DISTANCE = 0.052

# System status
ENABLE_LIDAR = True
STUCK_TIMEOUT = 30.0  # 1 minute timeout for no progress
MOVEMENT_THRESHOLD = 0.05  # Minimum distance (meters) to consider as progress

class EPuckController(Node):
    def __init__(self):
        super().__init__('epuck_controller')
        self.get_logger().info("Using simulation time")
        
        # Webots setup
        self.robot = Robot()
        global TIME_STEP
        TIME_STEP = int(self.robot.getBasicTimeStep())
        print("Time step is: ", TIME_STEP)

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

        if self.first_step:
            self.prev_left_pos = current_left_pos
            self.prev_right_pos = current_right_pos
            self.first_step = False
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

        # Check for movement progress
        distance_moved = np.sqrt((self.x - self.last_x) ** 2 + (self.y - self.last_y) ** 2)
        if distance_moved > MOVEMENT_THRESHOLD:
            self.last_moved_time = current_time
            self.last_x = self.x
            self.last_y = self.y

        # Check if stuck for too long
        if current_time - self.last_moved_time > STUCK_TIMEOUT:
            self.get_logger().warn(f"Robot {self.robot.getName()} stuck for over {STUCK_TIMEOUT} seconds, terminating.")
            # Publish termination message
            term_msg = String()
            term_msg.data = f"Robot {self.robot.getName()} terminated due to no progress."
            self.terminated_publisher.publish(term_msg)
            self.cleanup()
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
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.2
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

    def detect_obstacle(self):
        """Obstacle detection using LiDAR"""
        ranges = self.lidar.getRangeImage()
        front = min(ranges[len(ranges) // 3 : 2 * len(ranges) // 3])
        sides = min(ranges[:len(ranges) // 3] + ranges[2 * len(ranges) // 3:])
        return front < 0.3 or sides < 0.3

    def avoid_obstacle(self):
        """Obstacle avoidance that steers away from obstacles"""
        ranges = self.lidar.getRangeImage()
        
        # Check front-left and front-right sectors
        front_left = min(ranges[len(ranges)//4:len(ranges)//2])
        front_right = min(ranges[len(ranges)//2:3*len(ranges)//4])
        
        # Check left and right sectors
        left = min(ranges[:len(ranges)//4])
        right = min(ranges[3*len(ranges)//4:])

        # Determine which way to turn
        if front_left < 0.3 or front_right < 0.3:
            if front_left > front_right:
                return -MAX_SPEED * 0.2, MAX_SPEED * 0.5
            else:
                return MAX_SPEED * 0.5, -MAX_SPEED * 0.2
        elif left < 0.3 or right < 0.3:
            if left > right:
                return MAX_SPEED * 0.5, -MAX_SPEED * 0.2
            else:
                return -MAX_SPEED * 0.2, MAX_SPEED * 0.5
        else:
            # No obstacles detected, move forward
            return MAX_SPEED * 0.5, MAX_SPEED * 0.5
    
    def random_walk(self):
        """Random walk with better exploration"""
        # Get some sensor input to inform the random walk
        ranges = self.lidar.getRangeImage()
        front_dist = min(ranges[len(ranges)//3:2*len(ranges)//3])
        
        # Base speed
        base_speed = MAX_SPEED * 0.75
        
        # Randomness factor
        random_factor = random.uniform(-0.2, 0.2)
        
        # Bias toward open space
        if front_dist > 1.0:
            # Go faster in open areas
            left = base_speed * (1.0 + random_factor)
            right = base_speed * (1.0 - random_factor)
        else:
            # Slow down in tighter areas
            slow_factor = max(0.3, front_dist / 1.0)
            left = base_speed * slow_factor * (1.0 + random_factor)
            right = base_speed * slow_factor * (1.0 - random_factor)
        
        return left, right
    
    def run(self):
        self.get_logger().info("Controller started.")
        executor = MultiThreadedExecutor()
        executor.add_node(self)

        # Initial TF and clock publish
        sim_time = self.robot.getTime()
        self.get_clock().set_ros_time_override(rclpy.time.Time(seconds=sim_time))
        clock_now = self.get_clock().now().to_msg()
        for _ in range(20): 
            self.publish_odom_and_tf(clock_now)
            self.publish_clock(clock_now)
            executor.spin_once(timeout_sec=0.01)
        
        #timer for 5 minutes before we leave while loop
        self.get_logger().info("Starting main loop.")
        self.get_logger().info("Press Ctrl+C to stop the robot.")

        try:
            while self.robot.step(TIME_STEP) != -1:
                # Sync ROS clock with Webots sim time and get current time
                sim_time = self.robot.getTime()
                self.get_clock().set_ros_time_override(rclpy.time.Time(seconds=sim_time))
                clock_now = self.get_clock().now().to_msg()

                # Publish clock, odom, and scan with consistent time
                self.publish_clock(clock_now)
                if ENABLE_LIDAR:
                    self.publish_scan(clock_now)
                self.publish_odom_and_tf(clock_now)

                
                if self.detect_obstacle():
                    left_speed, right_speed = self.avoid_obstacle()
                else:
                    left_speed, right_speed = self.random_walk()
                self.set_speed(left_speed, right_speed)
                executor.spin_once(timeout_sec=0.01)
        
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