from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    repo_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../../../'))
    controller_path = os.path.join(repo_dir, 'ros2_ws/src/epuck_nav2_pkg/scripts/epuck_controller_nav2.py')
    slam_params = os.path.join(get_package_share_directory('epuck_nav2_pkg'), 'config', 'slam_toolbox_params.yaml')
    
    # Default Webots controller path
    webots_controller = '/usr/local/webots/webots-controller' if os.getenv('USER') == 'markus' else '/snap/webots/27/usr/share/webots/webots-controller'
    
    # Get the path to nav2_params.yaml within the package
    config_path = os.path.join(get_package_share_directory('epuck_nav2_pkg'), 'config', 'nav2_params.yaml')

    # Nav2 launch file
    nav2_launch_file = os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')

    return LaunchDescription([
        # Your e-puck controller with Webots controller runner
        ExecuteProcess(
            cmd=[webots_controller, controller_path],
            output='screen',
            name='epuck_controller'
        ),
        # SLAM Toolbox (assuming async mode)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params]
        ),
        # Nav2 bringup using IncludeLaunchDescription
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            'slam': 'False',
            'params_file': config_path,
            'map': '',
            'use_sim_time': 'True',
            'autostart': 'True',  # Ensure lifecycle nodes start automatically
            '--ros-args': '--log-level DEBUG'
        }.items()
)
    ])