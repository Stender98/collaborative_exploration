#VERIFY TOPICS IS PUBLISHED TO
ros2 topic echo /scan
ros2 topic echo /odom
ros2 topic echo /map

#HOW TO RUN ON LINUX
#terminal1
webots

#terminal2
/snap/webots/27/usr/share/webots/webots-controller ~/MasterThesis/controllers/epuck_controller/epuck_controller.py

Markus:
/usr/local/webots/webots-controller ~/Desktop/Master/MasterThesis/controllers/epuck_controller/epuck_controller.py

#terminal3
ros2 launch slam_toolbox online_async_launch.py

#terminal4
ros2 run rviz2 rviz2



