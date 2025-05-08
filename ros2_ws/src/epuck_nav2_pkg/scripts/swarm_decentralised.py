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

# FSM states
RANDOM_WALK = 0
AVOID_OBSTACLE = 1
FOLLOW_WALL = 2
AVOID_ROBOT = 3

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

        # Emitter and receiver for robot detection
        self.emitter = self.robot.getDevice("emitter")
        self.receiver = self.robot.getDevice("receiver")
        self.receiver.enable(TIME_STEP)

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
        return front < 0.3

    def avoid_obstacle(self):
        """Obstacle avoidance that steers away from obstacles"""
        ranges = self.lidar.getRangeImage()
        
        # Check front-left and front-right sectors
        front_left = min(ranges[len(ranges)//4:len(ranges)//2])
        front_right = min(ranges[len(ranges)//2:3*len(ranges)//4])
        
        # Determine which way to turn
        if front_left > front_right:
            return -MAX_SPEED * 0.2, MAX_SPEED * 0.5
        else:
            return MAX_SPEED * 0.5, -MAX_SPEED * 0.2

    def detect_wall_side(self):
        """Wall detection that returns side and distance"""
        ranges = self.lidar.getRangeImage()

        left_front = min(ranges[len(ranges)//6:len(ranges)//3])
        left_side = min(ranges[:len(ranges)//6])
        right_front = min(ranges[2*len(ranges)//3:5*len(ranges)//6])
        right_side = min(ranges[5*len(ranges)//6:])
        
        if left_side < 0.4:
            return 'left', left_side, left_front
        elif right_side < 0.4:
            return 'right', right_side, right_front
        return None, 0, 0

    def follow_wall(self, side, side_dist, front_dist):
        """Wall following with PID-like control"""
        target_dist = 0.25  # Desired distance from wall
        
        left_speed = 0
        right_speed = 0

        if side == 'left':
            # Error is positive when too far from wall
            error = target_dist - side_dist
            # Also check the angle to the wall using front and side
            angle_error = front_dist - side_dist
            
            # PID-like control
            p_gain = 2.0
            d_gain = 1.0
            
            # Adjust speeds based on distance and angle to wall
            base_speed = MAX_SPEED * 0.4
            speed_diff = (error * p_gain) + (angle_error * d_gain)
            
            left_speed = base_speed - speed_diff
            right_speed = base_speed + speed_diff
        
        elif side == 'right':
            # Similar logic for right wall following
            error = target_dist - side_dist
            angle_error = front_dist - side_dist
            
            p_gain = 2.0
            d_gain = 1.0
            
            base_speed = MAX_SPEED * 0.4
            speed_diff = (error * p_gain) + (angle_error * d_gain)
            
            left_speed = base_speed + speed_diff
            right_speed = base_speed - speed_diff
        
        # Ensure speeds are within limits
        left_speed = max(0, min(MAX_SPEED, left_speed))
        right_speed = max(0, min(MAX_SPEED, right_speed))
        
        return left_speed, right_speed

    def random_walk(self):
        """Random walk with better exploration"""
        # Get some sensor input to inform the random walk
        ranges = self.lidar.getRangeImage()
        front_dist = min(ranges[len(ranges)//3:2*len(ranges)//3])
        
        # Base speed
        base_speed = MAX_SPEED * 0.5
        
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

    def detect_robot(self):
        """
        Robot detection with decisive directional information
        """
        robot_detected = False
        max_signal = 0
        
        # Process all received messages
        while self.receiver.getQueueLength() > 0:
            signal_strength = self.receiver.getSignalStrength()
            
            if signal_strength > max_signal:
                max_signal = signal_strength
            
            if signal_strength > 25:  # Detection threshold
                robot_detected = True
                
            self.receiver.nextPacket()
        
        # If no robot detected, return early
        if not robot_detected:
            return False, 0, None
        
        # Use LiDAR to find the most likely direction of the robot
        # We'll look for the nearest object that could be the robot
        ranges = self.lidar.getRangeImage()
        
        # Find clusters of close readings that might be a robot
        potential_robots = []
        in_cluster = False
        cluster_start = 0
        cluster_min_dist = float('inf')
        
        for i, distance in enumerate(ranges):
            if distance < 0.5:  # Potential robot detection range
                if not in_cluster:
                    in_cluster = True
                    cluster_start = i
                    cluster_min_dist = distance
                elif distance < cluster_min_dist:
                    cluster_min_dist = distance
            elif in_cluster:
                # End of cluster
                cluster_end = i - 1
                # Only consider clusters of reasonable size
                if 3 <= (cluster_end - cluster_start) <= 20:
                    cluster_center = (cluster_start + cluster_end) // 2
                    potential_robots.append((cluster_min_dist, cluster_center))
                in_cluster = False
        
        # If we were still in a cluster when we finished the loop
        if in_cluster and 3 <= (len(ranges) - 1 - cluster_start) <= 20:
            cluster_center = (cluster_start + len(ranges) - 1) // 2
            potential_robots.append((cluster_min_dist, cluster_center))
        
        # Sort by distance and take the closest potential robot
        if potential_robots:
            potential_robots.sort()
            _, closest_idx = potential_robots[0]
            
            # Convert index to angle (-π to π)
            angle = -np.pi + closest_idx * self.angle_step
            return True, max_signal, angle
        
        # Fallback - just take the minimum distance point
        shortest_range = min(ranges)
        shortest_idx = ranges.index(shortest_range)
        angle = -np.pi + shortest_idx * self.angle_step
        
        return True, max_signal, angle

    def avoid_robot(self, signal_strength, angle):
        """
        Robot avoidance that creates significant separation
        """
        # Normalize signal strength but with higher sensitivity
        normalized_signal = min(1.0, signal_strength / 50.0)  # More sensitive to distance
        
        # Base speed components
        base_move = MAX_SPEED * (0.5 + 0.5 * normalized_signal)  # Higher speed when closer
        turn_factor = MAX_SPEED * normalized_signal  # More aggressive turns when closer
        
        # Decisive movements based on relative position
        if abs(angle) < np.pi/3:  # Robot in front - back up fast and turn
            if angle < 0:  # Front-left - back up right
                left_speed = -base_move
                right_speed = -base_move * 0.3
            else:  # Front-right - back up left
                left_speed = -base_move * 0.3
                right_speed = -base_move
        elif abs(angle) > 2*np.pi/3:  # Robot behind - move forward fast
            if angle < 0:  # Back-left - forward right
                left_speed = base_move
                right_speed = base_move * 0.3
            else:  # Back-right - forward left
                left_speed = base_move * 0.3
                right_speed = base_move
        elif angle < 0:  # Robot on left - move right
            left_speed = base_move
            right_speed = -turn_factor
        else:  # Robot on right - move left
            left_speed = -turn_factor
            right_speed = base_move
        
        # Add random component to break symmetry
        left_speed += random.uniform(-0.5, 0.5)
        right_speed += random.uniform(-0.5, 0.5)
        
        # Ensure speeds are within bounds but maintain directionality
        left_speed = max(-MAX_SPEED, min(MAX_SPEED, left_speed))
        right_speed = max(-MAX_SPEED, min(MAX_SPEED, right_speed))
        
        return left_speed, right_speed
    
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
        
        state = RANDOM_WALK
        state_timer = 0 
        MIN_STATE_TIME = {
            RANDOM_WALK: 10,
            AVOID_OBSTACLE: 15,
            FOLLOW_WALL: 10,
            AVOID_ROBOT: 30  # Longer timer to ensure full separation
        }

        # Robot avoidance cooldown 
        robot_cooldown = 0
        ROBOT_COOLDOWN_TIME = 50  # Time steps before allowing another robot detection
        last_robot_angle = None
        
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

                # Broadcast presence locally
                self.emitter.send(self.robot.getName().encode('utf-8'))

                wall_side, side_dist, front_dist = wall_info = self.detect_wall_side()

                # FSM transitions with hysteresis
                if state_timer >=  MIN_STATE_TIME.get(state, 15):
                    new_state = state  # Default to keeping current state
                    
                    if robot_cooldown == 0:
                        robot_detected, signal_strength, angle = self.detect_robot()
                        if robot_detected and signal_strength > 30:
                            new_state = AVOID_ROBOT
                            last_robot_angle = angle
                            #print(f"State: AVOID_ROBOT (Signal: {signal_strength:.1f}, Angle: {angle:.2f})")
                    
                    # Other state checks, only if not avoiding robot
                    if new_state != AVOID_ROBOT:
                        if self.detect_obstacle():
                            new_state = AVOID_OBSTACLE  
                            #print("State: AVOID_OBSTACLE")
                        elif wall_side is not None and side_dist < 0.4:
                            # Wall detected, switch to wall following
                            new_state = FOLLOW_WALL
                            #print(f"State: FOLLOW_WALL ({wall_side} wall, dist: {side_dist:.2f})")
                        else:
                            new_state = RANDOM_WALK
                            #print("State: RANDOM_WALK")
                    
                    if new_state != state:
                        state = new_state
                        state_timer = 0
                        
                        # Set cooldown when exiting AVOID_ROBOT state
                        if state != AVOID_ROBOT and new_state != AVOID_ROBOT:
                            robot_cooldown = ROBOT_COOLDOWN_TIME
                else:
                    state_timer += 1
                
                # FSM behavior
                if state == AVOID_ROBOT:
                    # Check if robot is still detected
                    robot_still_detected, signal_strength, angle = self.detect_robot()
                    
                    if robot_still_detected:
                        # Update last known angle if still detected
                        last_robot_angle = angle
                    
                    # Use last known angle even if not currently detected
                    left_speed, right_speed = self.avoid_robot(40, last_robot_angle)  # Use fixed signal strength for consistency
                    self.set_speed(left_speed, right_speed)
                    
                elif state == AVOID_OBSTACLE:
                    left_speed, right_speed = self.avoid_obstacle()
                    self.set_speed(left_speed, right_speed)
                elif state == FOLLOW_WALL:
                    left_speed, right_speed = self.follow_wall(wall_side, side_dist, front_dist)
                    self.set_speed(left_speed, right_speed)
                elif state == RANDOM_WALK:
                    left_speed, right_speed = self.random_walk()
                    self.set_speed(left_speed, right_speed)
                    
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