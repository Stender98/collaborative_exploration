from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Define paths
    repo_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../epuck_nav2_pkg'))
    controller_path = os.path.join(repo_dir, 'scripts/epuck_controller_nav2.py')
    config_path = os.path.join(get_package_share_directory('epuck_nav2_pkg'), 'config', 'nav2_params.yaml')

    # RViz configuration file path (optional, adjust as needed)
    rviz_config_path = os.path.join(get_package_share_directory('epuck_nav2_pkg'), 'config', 'nav2_rviz_config.rviz')

    # Webots controller path
    webots_controller = '/usr/local/webots/webots-controller' if os.getenv('USER') == 'markus' else '/snap/webots/27/usr/share/webots/webots-controller'

    # Nav2 launch file
    nav2_launch_file = os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')

    # Define the controller launch action
    controller_launch = ExecuteProcess(
        cmd=[webots_controller, controller_path],
        output='screen',
        name='epuck_controller'
    )

    # Define the Nav2 bringup action
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            'slam': 'True',              # Enable SLAM mode
            'params_file': config_path,  # Nav2 params
            'map': '',                   # No static map
            'use_sim_time': 'True',      # Use simulation time
            'autostart': 'True',         # Auto-start lifecycle nodes
        }.items()
    )

    # Define the RViz launch action
    rviz_launch = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config_path],  # Use custom config file
        # cmd=['rviz2'],  # Uncomment to launch RViz without a config file
        output='screen',
        name='rviz2'
    )

    return LaunchDescription([
        # e-puck controller with Webots (starts immediately)
        controller_launch,
        # Nav2 bringup with a 2-second delay
        TimerAction(
            period=5.0,  # Delay in seconds
            actions=[nav2_bringup]
        ),
        TimerAction(
            period=6.0,  # Delay in seconds
            actions=[rviz_launch]
        )
    ])