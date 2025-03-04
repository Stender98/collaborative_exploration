"""epuck_controller."""
from controller import Robot, Keyboard
import socket
import struct

TIME_STEP = 64  #Webots simulation time step
MAX_SPEED = 6.28  #E-puck max speed

#system status
DEBUG = False
ENABLE_LIDAR = True
ENABLE_CAMERA = True
ENABLE_DIST = False

# UDP settings
UDP_IP = "localhost"  # Change this to your actual Docker container IP
UDP_PORT = 5005

print("Controller initiated, starting the robot.")

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

#robot and IO devices
robot = Robot()
keyboard = Keyboard()
keyboard.enable(TIME_STEP)

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

        #key control
        if key == Keyboard.UP:
            left_speed = MAX_SPEED
            right_speed = MAX_SPEED
        elif key == Keyboard.DOWN:
            left_speed = -MAX_SPEED
            right_speed = -MAX_SPEED
        elif key == Keyboard.LEFT:
            left_speed = -MAX_SPEED / 2
            right_speed = MAX_SPEED / 2
        elif key == Keyboard.RIGHT:
            left_speed = MAX_SPEED / 2
            right_speed = -MAX_SPEED / 2

        set_speed(left_speed, right_speed)

        if ENABLE_LIDAR:
            lidar_data = lidar.getRangeImage()
            
            #pack data into binary format
            data = struct.pack(f'{len(lidar_data)}f', *lidar_data)

            #send over UDP to Docker container running ROS2 and SLAM
            sock.sendto(data, (UDP_IP, UDP_PORT))

            if DEBUG:
                forward_dist = lidar_data[num_rays // 2]  # Front (~180°)
                left_dist = lidar_data[num_rays // 4]  # Left (~90°)
                right_dist = lidar_data[3 * num_rays // 4]  # Right (~270°)
                print(f"Lidar: Forward={forward_dist:.2f}, Left={left_dist:.2f}, Right={right_dist:.2f}")

except KeyboardInterrupt:
    print("Controller interrupted, stopping the robot.")

finally:
    print("Cleaning up...")
    set_speed(0, 0)  #stop motors
    if ENABLE_LIDAR:
        lidar.disable()
    if ENABLE_CAMERA:
        camera.disable()
    if ENABLE_DIST:
        for i in range(8):
            ps[i].disable()
    print("Shutdown complete.")
