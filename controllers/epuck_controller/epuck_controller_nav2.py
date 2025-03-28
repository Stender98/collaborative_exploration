"""epuck_controller."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
import tf_transformations
import tf2_ros
from controller import Robot, Keyboard
import numpy as np

TIME_STEP = 128  # Webots simulation time step
MAX_SPEED = 6  # E-puck max velocity is 6.28 rad/s

# System status
DEBUG = False
ENABLE_LIDAR = True
ENABLE_CAMERA = False
ENABLE_DIST = False

print("Controller initiated, starting the robot.")
rclpy.init()

# Robot and IO devices
robot = Robot()
keyboard = Keyboard()
keyboard.enable(TIME_STEP)

# Robot state
x = 0.0
y = 0.0
theta = 0.0
last_time = robot.getTime()
first_step = True  # Flag for initial step

# Motors
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Position sensors
left_sensor = robot.getDevice('left wheel sensor')
right_sensor = robot.getDevice('right wheel sensor')
left_sensor.enable(TIME_STEP)
right_sensor.enable(TIME_STEP)
prev_left_pos = 0.0
prev_right_pos = 0.0

# Control mode flag (True = keyboard, False = Nav2)
use_keyboard = True

def set_speed(left, right):
    left_motor.setVelocity(left)
    right_motor.setVelocity(right)

# ROS2 node
publicist = rclpy.create_node('epuck_controller')
odom_publisher = publicist.create_publisher(Odometry, '/odom', 10)
lidar_publisher = publicist.create_publisher(LaserScan, '/scan', 10)
tf_broadcaster = tf2_ros.TransformBroadcaster(publicist)

# Subscriber for Nav2 velocity commands
cmd_vel_subscriber = publicist.create_subscription(
    Twist, '/cmd_vel', lambda msg: cmd_vel_callback(msg), 10
)

# Store latest Nav2 command
latest_cmd_vel = Twist()

def cmd_vel_callback(msg):
    global latest_cmd_vel
    latest_cmd_vel = msg

# Lidar setup
if ENABLE_LIDAR:
    lidar = robot.getDevice("LDS-01")
    lidar.enable(TIME_STEP)
    num_rays = lidar.getHorizontalResolution()
    fov = lidar.getFov()
    angle_step = fov / num_rays
    print(f"Lidar initialized: {num_rays} rays, FOV: {fov:.2f} rad, Angle step: {angle_step:.4f} rad")

def publish_scan(clock):
    ranges = list(reversed(lidar.getRangeImage()))
    if ranges is None:
        return
    
    msg = LaserScan()
    msg.header.stamp = clock
    msg.header.frame_id = "laser"
    msg.angle_min = -np.pi
    msg.angle_max = np.pi
    msg.angle_increment = np.pi * 2 / len(ranges)
    msg.range_min = 0.1
    msg.range_max = 3.5
    msg.ranges = ranges
    lidar_publisher.publish(msg)

def publish_odom(clock):
    global last_time, x, y, theta, prev_left_pos, prev_right_pos, first_step
    current_time = robot.getTime()
    dt = current_time - last_time
    last_time = current_time

    current_left_pos = left_sensor.getValue()
    current_right_pos = right_sensor.getValue()

    if first_step:
        prev_left_pos = current_left_pos
        prev_right_pos = current_right_pos
        first_step = False
        return

    delta_left = current_left_pos - prev_left_pos
    delta_right = current_right_pos - prev_right_pos
    prev_left_pos = current_left_pos
    prev_right_pos = current_right_pos

    wheel_radius = 0.0205
    wheel_distance = 0.052

    delta_s_left = delta_left * wheel_radius
    delta_s_right = delta_right * wheel_radius

    delta_s = (delta_s_right + delta_s_left) / 2.0
    delta_theta = (delta_s_right - delta_s_left) / wheel_distance

    if abs(delta_theta) < 1e-6:
        dx = delta_s * np.cos(theta)
        dy = delta_s * np.sin(theta)
    else:
        radius = delta_s / delta_theta
        dx = radius * (np.sin(theta + delta_theta) - np.sin(theta))
        dy = radius * (np.cos(theta) - np.cos(theta + delta_theta))

    x += dx if not np.isnan(dx) else 0.0
    y += dy if not np.isnan(dy) else 0.0
    theta += delta_theta if not np.isnan(delta_theta) else 0.0

    odom_msg = Odometry()
    odom_msg.header.stamp = clock
    odom_msg.header.frame_id = "odom"
    odom_msg.child_frame_id = "base_footprint"
    
    odom_msg.pose.pose.position.x = x if not np.isnan(x) else 0.0
    odom_msg.pose.pose.position.y = y if not np.isnan(y) else 0.0
    odom_msg.pose.pose.position.z = 0.0
    
    q = tf_transformations.quaternion_from_euler(0, 0, theta if not np.isnan(theta) else 0.0)
    odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    odom_msg.pose.covariance = [
        0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.01
    ]

    odom_publisher.publish(odom_msg)

    transform = TransformStamped()
    transform.header.stamp = odom_msg.header.stamp
    transform.header.frame_id = "odom"
    transform.child_frame_id = "base_footprint"
    transform.transform.translation.x = x if not np.isnan(x) else 0.0
    transform.transform.translation.y = y if not np.isnan(y) else 0.0
    transform.transform.translation.z = 0.0
    transform.transform.rotation = odom_msg.pose.pose.orientation
    tf_broadcaster.sendTransform(transform)

    base_link_transform = TransformStamped()
    base_link_transform.header.stamp = odom_msg.header.stamp
    base_link_transform.header.frame_id = "base_footprint"
    base_link_transform.child_frame_id = "base_link"
    base_link_transform.transform.translation.x = 0.0
    base_link_transform.transform.translation.y = 0.0
    base_link_transform.transform.translation.z = 0.0
    base_link_transform.transform.rotation.w = 1.0
    tf_broadcaster.sendTransform(base_link_transform)

    laser_transform = TransformStamped()
    laser_transform.header.stamp = odom_msg.header.stamp
    laser_transform.header.frame_id = "base_link"
    laser_transform.child_frame_id = "laser"
    laser_transform.transform.translation.x = 0.0
    laser_transform.transform.translation.y = 0.0
    laser_transform.transform.translation.z = 0.05
    laser_transform.transform.rotation.w = 1.0
    tf_broadcaster.sendTransform(laser_transform)

def run_publicist(clock):
    publish_scan(clock)
    publish_odom(clock)

# Main loop
print("Controller started. Use arrow keys for manual control, press 'K' to toggle keyboard/Nav2 mode.")
try:
    while robot.step(TIME_STEP) != -1:
        key = keyboard.getKey()
        left_speed = 0.0
        right_speed = 0.0

        # Toggle control mode with 'K'
        if key == ord('K'):
            use_keyboard = not use_keyboard
            print(f"Switched to {'keyboard' if use_keyboard else 'Nav2'} control mode.")

        if use_keyboard:
            # Keyboard control
            if key == Keyboard.UP:
                left_speed = MAX_SPEED
                right_speed = MAX_SPEED
            elif key == Keyboard.DOWN:
                left_speed = -MAX_SPEED
                right_speed = -MAX_SPEED
            elif key == Keyboard.LEFT:
                left_speed = -MAX_SPEED * 0.3
                right_speed = MAX_SPEED * 0.3
            elif key == Keyboard.RIGHT:
                left_speed = MAX_SPEED * 0.3
                right_speed = -MAX_SPEED * 0.3
        else:
            # Nav2 control
            linear = latest_cmd_vel.linear.x
            angular = latest_cmd_vel.angular.z
            wheel_radius = 0.0205
            wheel_distance = 0.052

            # Convert to wheel velocities
            left_speed = (linear - angular * wheel_distance / 2.0) / wheel_radius
            right_speed = (linear + angular * wheel_distance / 2.0) / wheel_radius

            # Clamp speeds to MAX_SPEED
            left_speed = max(min(left_speed, MAX_SPEED), -MAX_SPEED)
            right_speed = max(min(right_speed, MAX_SPEED), -MAX_SPEED)

        set_speed(left_speed, right_speed)
        
        if ENABLE_LIDAR:
            clock_now = publicist.get_clock().now().to_msg()
            run_publicist(clock_now)

except KeyboardInterrupt:
    print("Controller interrupted, stopping the robot.")

finally:
    print("Cleaning up...")
    set_speed(0, 0)
    if ENABLE_LIDAR:
        lidar.disable()
    rclpy.shutdown()
    print("Shutdown complete.")