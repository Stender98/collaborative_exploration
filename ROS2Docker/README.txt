#NEED THIS FOR GUI TO SHOW IN WINDOWS
downloaded VcXsrv for Windows used in run script

#BUILD IMAGE FROM DOCKERFILE
docker build -t ros2-slam-container .

#START CONTAINER
.\run_ros2_docker.bat

#FOR OPENING MULTIPLE TERMINALS
docker exec -it <container_id> bash

#RUNNING SLAM
#1 RUNNING SLAM TOOLBOX (Might use something else later if it does not work)
ros2 launch slam_toolbox online_async_launch.py

#2 RUN SCRIPT THAT DOCKERFILE CREATED TO LISTEN ON UDP
python3 root/udp_to_ros2.py

#3 VERIFY DOCKER CONTAINER IS GETTING LIDAR DATA OVER UDP
ros2 topic echo /scan

#RUNNING RViz2
rviz2

#RTAB-Map dependencies
sudo apt update && sudo apt install -y \
  ros-jazzy-cv-bridge \
  ros-jazzy-grid-map-core \
  ros-jazzy-gtsam \
  ros-jazzy-libg2o \
  ros-jazzy-libpointmatcher \
  ros-jazzy-qt-gui-cpp

sudo apt-get install -y \
  ros-jazzy-rtabmap \
  ros-jazzy-rtabmap-viz \
  ros-jazzy-rtabmap-slam 

ros2 run rtabmap_viz rtabmap_viz _frame_id:=base_link