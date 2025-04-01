from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Define paths
    repo_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../epuck_nav2_pkg'))
    controller_path = os.path.join(repo_dir, 'scripts/epuck_controller_nav2.py')
    slam_params = os.path.join(get_package_share_directory('epuck_nav2_pkg'), 'config', 'slam_toolbox_params.yaml')
    config_path = os.path.join(get_package_share_directory('epuck_nav2_pkg'), 'config', 'nav2_params.yaml')

    # Webots controller path
    webots_controller = '/usr/local/webots/webots-controller' if os.getenv('USER') == 'markus' else '/snap/webots/27/usr/share/webots/webots-controller'

    # Nav2 launch file
    nav2_launch_file = os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')

    return LaunchDescription([
        # e-puck controller with Webots
        ExecuteProcess(
            cmd=[webots_controller, controller_path],
            output='screen',
            name='epuck_controller'
        ),
        # Nav2 bringup with SLAM Toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments={
                'slam': 'True',              # Enable SLAM mode
                'params_file': config_path,  # Nav2 params
                'slam_params_file': slam_params,  # SLAM Toolbox params
                'map': '',                   # No static map
                'use_sim_time': 'True',      # Use simulation time
                'autostart': 'True',         # Auto-start lifecycle nodes
            }.items()
        )
    ])