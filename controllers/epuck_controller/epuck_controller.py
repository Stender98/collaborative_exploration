"""epuck_controller."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
import tf_transformations
import tf2_ros
from controller import Robot, Keyboard, Lidar
import numpy as np


TIME_STEP = 128  #Webots simulation time step
MAX_SPEED = 6.28  #E-puck max speed

#system status
DEBUG = False
ENABLE_LIDAR = True
ENABLE_CAMERA = False
ENABLE_DIST = False

print("Controller initiated, starting the robot.")
rclpy.init()

#robot and IO devices
robot = Robot()
keyboard = Keyboard()
keyboard.enable(TIME_STEP)

#robot state
x = 0.0
y = 0.0
theta = 0.0
last_time = robot.getTime()

#motors
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))  
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

def set_speed(left, right):
    left_motor.setVelocity(left)
    right_motor.setVelocity(right)

#ros2 publisher node
publicist = rclpy.create_node('epuck_controller')
odom_publisher = publicist.create_publisher(Odometry, '/odom', 10)
lidar_publisher = publicist.create_publisher(LaserScan, '/scan', 10)
tf_broadcaster = tf2_ros.TransformBroadcaster(publicist)

#distance sensor
if ENABLE_DIST:
    ps = []
    psNames = [
        'ps0', 'ps1', 'ps2', 'ps3',
        'ps4', 'ps5', 'ps6', 'ps7'
    ]
    for i in range(8):
        ps.append(robot.getDevice(psNames[i]))
        ps[i].enable(TIME_STEP)

#camera
if ENABLE_CAMERA:
    camera = robot.getDevice("camera")
    camera.enable(TIME_STEP)

#lidar
if ENABLE_LIDAR:
    lidar = robot.getDevice("LDS-01")
    lidar.enable(TIME_STEP)

    #get LiDAR properties
    num_rays = lidar.getHorizontalResolution()  #total number of points per scan
    fov = lidar.getFov()  #field of view (should be ~360°)
    angle_step = fov / num_rays  #angle per step
    print(f"Lidar initialized: {num_rays} rays, FOV: {fov:.2f} rad, Angle step: {angle_step:.4f} rad")

def publish_scan(clock):
    ranges = list(reversed(lidar.getRangeImage()))
    if ranges is None:
        return
    
    msg = LaserScan()
    msg.header.stamp = clock
    msg.header.frame_id = "laser"

    msg.angle_min = -np.pi #assume full 360° lidar
    msg.angle_max = np.pi
    msg.angle_increment = np.pi * 2 / len(ranges)
    msg.range_min = 0.1  #adjust for your LiDAR specs
    msg.range_max = 3.5
    msg.ranges = ranges


    lidar_publisher.publish(msg)

def publish_odom(clock):
    global last_time, x, y, theta #update global variables
    current_time = robot.getTime()
    dt = current_time - last_time
    last_time = current_time

    left_speed = left_motor.getVelocity()
    right_speed = right_motor.getVelocity()

    wheel_radius = 0.041  #adjust for your robot
    wheel_distance = 0.053  #distance between wheels

    v_left = left_speed * wheel_radius
    v_right = right_speed * wheel_radius

    v = (v_right + v_left) / 2.0
    omega = (v_right - v_left) / wheel_distance

    x += v * dt * np.cos(theta)
    y += v * dt * np.sin(theta)
    theta += omega * dt

    #create odometry message
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

    #broadcast TF transform
    transform = TransformStamped()
    transform.header.stamp = odom_msg.header.stamp
    transform.header.frame_id = "odom"
    transform.child_frame_id = "base_footprint"
    transform.transform.translation.x = x
    transform.transform.translation.y = y
    transform.transform.translation.z = 0.0
    transform.transform.rotation = odom_msg.pose.pose.orientation

    #assuming laser is at (0.1, 0.0, 0.0) relative to base_footprint
    laser_transform = TransformStamped()
    laser_transform.header.stamp = odom_msg.header.stamp
    laser_transform.header.frame_id = "base_footprint"
    laser_transform.child_frame_id = "laser"
    laser_transform.transform.translation.x = 0.1
    laser_transform.transform.translation.y = 0.0
    laser_transform.transform.translation.z = 0.0
    laser_transform.transform.rotation.w = 1.0  #no rotation
    tf_broadcaster.sendTransform(laser_transform)


    tf_broadcaster.sendTransform(transform)

def run_publicist(clock):
    publish_scan(clock)
    publish_odom(clock)

#main loop
print("Controller started, control the robot with arrow keys.")
try:
    while robot.step(TIME_STEP) != -1:
        #read distance sensors outputs
        if ENABLE_DIST:
            psValues = []
            for i in range(8):
                psValues.append(ps[i].getValue())
            
        key = keyboard.getKey()
        
        #default stop
        left_speed = 0
        right_speed = 0

        #key control logic
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

        set_speed(left_speed, right_speed)
        
        #publish ROS messages
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
