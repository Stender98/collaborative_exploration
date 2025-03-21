"""epuck_controller."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
import tf_transformations
import tf2_ros
from controller import Robot, Lidar
import numpy as np

# Constants
TIME_STEP = 128  # Webots simulation time step
MAX_SPEED = 3.0  # Reduced max speed for smoother movement
MAX_ANGULAR_SPEED = 1.0  # Maximum angular speed (in radians per second)
SAFE_DISTANCE_THRESHOLD = 80  # Threshold for obstacle detection (proximity sensor value)
WHEEL_RADIUS = 0.0205  # Wheel radius (in meters)
AXLE_LENGTH = 0.053  # Distance between the centers of the wheels (in meters)

# System status
DEBUG = False
ENABLE_LIDAR = True
ENABLE_CAMERA = False
ENABLE_DIST = True  # Enable distance sensors for obstacle avoidance

print("Controller initiated, starting the robot.")
rclpy.init()

# Robot and IO devices
robot = Robot()

# Robot state
x = 0.0
y = 0.0
theta = 0.0
last_time = robot.getTime()

# Motors
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))  
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Encoders
left_encoder = robot.getDevice("left wheel sensor")
right_encoder = robot.getDevice("right wheel sensor")
left_encoder.enable(TIME_STEP)
right_encoder.enable(TIME_STEP)

def set_speed(left, right):
    left_motor.setVelocity(left)
    right_motor.setVelocity(right)

# ROS2 publisher node
publicist = rclpy.create_node('epuck_controller')
odom_publisher = publicist.create_publisher(Odometry, '/odom', 10)
lidar_publisher = publicist.create_publisher(LaserScan, '/scan', 10)
tf_broadcaster = tf2_ros.TransformBroadcaster(publicist)

# Distance sensors
if ENABLE_DIST:
    ps = []
    psNames = [
        'ps0', 'ps1', 'ps2', 'ps3',
        'ps4', 'ps5', 'ps6', 'ps7'
    ]
    for i in range(8):
        ps.append(robot.getDevice(psNames[i]))
        ps[i].enable(TIME_STEP)

# LiDAR
if ENABLE_LIDAR:
    lidar = robot.getDevice("LDS-01")
    lidar.enable(TIME_STEP)

    # Get LiDAR properties
    num_rays = lidar.getHorizontalResolution()  # Total number of points per scan
    fov = lidar.getFov()  # Field of view (should be ~360°)
    angle_step = fov / num_rays  # Angle per step
    print(f"Lidar initialized: {num_rays} rays, FOV: {fov:.2f} rad, Angle step: {angle_step:.4f} rad")

def publish_scan(clock):
    ranges = list(reversed(lidar.getRangeImage()))
    if ranges is None:
        return
    
    # Filter out invalid measurements
    filtered_ranges = []
    for r in ranges:
        if r > lidar.getMinRange() and r < lidar.getMaxRange():
            filtered_ranges.append(r)
        else:
            filtered_ranges.append(float('inf'))  # Mark invalid measurements as infinity

    msg = LaserScan()
    msg.header.stamp = clock
    msg.header.frame_id = "laser"

    msg.angle_min = -np.pi  # Assume full 360° LiDAR
    msg.angle_max = np.pi
    msg.angle_increment = np.pi * 2 / len(filtered_ranges)
    msg.range_min = lidar.getMinRange()
    msg.range_max = lidar.getMaxRange()
    msg.ranges = filtered_ranges

    lidar_publisher.publish(msg)

def publish_odom(clock):
    global last_time, x, y, theta  # Update global variables
    current_time = robot.getTime()
    dt = current_time - last_time
    last_time = current_time

    # Get encoder values (in radians)
    left_encoder_value = left_encoder.getValue()
    right_encoder_value = right_encoder.getValue()

    # Calculate linear and angular displacement
    delta_left = left_encoder_value * WHEEL_RADIUS  # Distance traveled by the left wheel
    delta_right = right_encoder_value * WHEEL_RADIUS  # Distance traveled by the right wheel
    delta_distance = (delta_left + delta_right) / 2.0  # Average distance traveled
    delta_theta = (delta_right - delta_left) / AXLE_LENGTH  # Change in orientation

    # Limit angular speed
    if abs(delta_theta) > MAX_ANGULAR_SPEED * dt:
        delta_theta = np.sign(delta_theta) * MAX_ANGULAR_SPEED * dt

    # Update robot position and orientation
    x += delta_distance * np.cos(theta) * dt
    y += delta_distance * np.sin(theta) * dt
    theta += delta_theta * dt

    # Create odometry message
    odom_msg = Odometry()
    odom_msg.header.stamp = clock
    odom_msg.header.frame_id = "odom"
    odom_msg.child_frame_id = "base_footprint"

    odom_msg.pose.pose.position.x = x
    odom_msg.pose.pose.position.y = y
    odom_msg.pose.pose.position.z = 0.0 

    q = tf_transformations.quaternion_from_euler(0, 0, theta)
    odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    odom_publisher.publish(odom_msg)

    # Broadcast TF transform
    transform = TransformStamped()
    transform.header.stamp = odom_msg.header.stamp
    transform.header.frame_id = "odom"
    transform.child_frame_id = "base_footprint"
    transform.transform.translation.x = x
    transform.transform.translation.y = y
    transform.transform.translation.z = 0.0
    transform.transform.rotation = odom_msg.pose.pose.orientation

    # Assuming laser is at (0.1, 0.0, 0.0) relative to base_footprint
    laser_transform = TransformStamped()
    laser_transform.header.stamp = odom_msg.header.stamp
    laser_transform.header.frame_id = "base_footprint"
    laser_transform.child_frame_id = "laser"
    laser_transform.transform.translation.x = 0.1
    laser_transform.transform.translation.y = 0.0
    laser_transform.transform.translation.z = 0.0
    laser_transform.transform.rotation.w = 1.0  # No rotation
    tf_broadcaster.sendTransform(laser_transform)

    tf_broadcaster.sendTransform(transform)

def run_publicist(clock):
    publish_scan(clock)
    publish_odom(clock)

# Main loop
print("Controller started, starting smooth obstacle avoidance.")
try:
    while robot.step(TIME_STEP) != -1:
        # Read distance sensor outputs
        if ENABLE_DIST:
            psValues = [ps[i].getValue() for i in range(8)]
            
            # Check if any sensor detects an obstacle closer than SAFE_DISTANCE_THRESHOLD
            if any(value > SAFE_DISTANCE_THRESHOLD for value in psValues):
                # Steer away from the obstacle
                left_obstacle = (psValues[5] + psValues[6] + psValues[7]) / 3.0
                right_obstacle = (psValues[0] + psValues[1] + psValues[2]) / 3.0
                steering_correction = (right_obstacle - left_obstacle) * 0.1  # Scaling factor
                left_speed = MAX_SPEED - steering_correction
                right_speed = MAX_SPEED + steering_correction
                set_speed(left_speed, right_speed)
            else:
                # No obstacle detected, move forward
                set_speed(MAX_SPEED, MAX_SPEED)
        
        # Publish ROS messages
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
    if ENABLE_CAMERA:
        camera.disable()
    if ENABLE_DIST:
        for i in range(8):
            ps[i].disable()
    rclpy.shutdown()
    print("Shutdown complete.")