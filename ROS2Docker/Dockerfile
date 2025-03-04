FROM ros:jazzy-ros-core-noble

# Install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# Bootstrap rosdep
RUN rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO

# Setup colcon mixin and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# Install ROS2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-base=0.11.0-1* \
    ros-jazzy-slam-toolbox \
    ros-jazzy-rviz2 \
    ros-jazzy-rviz-default-plugins \
    && rm -rf /var/lib/apt/lists/*

# Add setup.bash to bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Create the UDP to ROS2 Python script
RUN echo 'import socket\n\
import struct\n\
import rclpy\n\
from rclpy.node import Node\n\
from sensor_msgs.msg import LaserScan\n\
import math\n\
\n\
UDP_IP = "0.0.0.0"  # Listen on all interfaces\n\
UDP_PORT = 5005\n\
\n\
class LiDARReceiver(Node):\n\
    def __init__(self):\n\
        super().__init__("lidar_receiver")\n\
        self.publisher_ = self.create_publisher(LaserScan, "/scan", 10)\n\
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)\n\
        self.sock.bind((UDP_IP, UDP_PORT))\n\
        self.timer = self.create_timer(0.1, self.receive_data)\n\
\n\
    def receive_data(self):\n\
        data, _ = self.sock.recvfrom(4096)  # Adjust buffer size if needed\n\
        ranges = struct.unpack(f"{len(data)//4}f", data)\n\
\n\
        # Create LaserScan message\n\
        scan_msg = LaserScan()\n\
        scan_msg.header.stamp = self.get_clock().now().to_msg()\n\
        scan_msg.header.frame_id = "lidar_frame"\n\
\n\
        scan_msg.angle_min = 0.0\n\
        scan_msg.angle_max = 2 * math.pi\n\
        scan_msg.angle_increment = 2 * math.pi / len(ranges)\n\
        scan_msg.range_min = 0.05\n\
        scan_msg.range_max = 3.5\n\
\n\
        scan_msg.ranges = list(ranges)\n\
\n\
        self.publisher_.publish(scan_msg)\n\
        self.get_logger().info("Received LiDAR data")\n\
\n\
def main(args=None):\n\
    rclpy.init(args=args)\n\
    node = LiDARReceiver()\n\
    rclpy.spin(node)\n\
    rclpy.shutdown()\n\
\n\
if __name__ == "__main__":\n\
    main()' > /root/udp_to_ros2.py
