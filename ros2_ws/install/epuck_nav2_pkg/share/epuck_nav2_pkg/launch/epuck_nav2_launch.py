from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Define paths
    repo_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../epuck_nav2_pkg'))
    controller_path = os.path.join(repo_dir, 'scripts/swarm_nav_controller.py')
    config_dir = os.path.join(get_package_share_directory('epuck_nav2_pkg'), 'config')
    rviz_config_path = os.path.join(config_dir, 'nav2_rviz_config.rviz')

    # Webots controller path
    webots_controller = '/usr/local/webots/webots-controller' if os.getenv('USER') == 'markus' else '/snap/webots/27/usr/share/webots/webots-controller'

    # Nav2 launch file
    nav2_launch_file = os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')


    num_robots = DeclareLaunchArgument(
        'num_robots',
        default_value='2',
        description='Number of robots to launch'
    )

    # Initialize LaunchDescription
    ld = LaunchDescription([num_robots])

    # Generate launch actions for each robot
    for i in range(2):
        robotid = f'robot{i}'
        namespace = f'/{robotid}'
        config_path = os.path.join(config_dir, f'nav2_params_{robotid}.yaml')

        # Webots controller (runs EPuckController with FrontierExploration)
        controller_launch = ExecuteProcess(
            cmd=[webots_controller, f'--robot-name={robotid}', controller_path],
            output='screen',
            name=f'epuck_controller_{robotid}',
            shell=True
        )

        # Nav2 bringup (no SLAM, using external merged map)
        nav2_bringup = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments={
                'namespace': namespace,
                'slam': 'False',
                'params_file': config_path,
                'map': '',
                'use_sim_time': 'True',
                'autostart': 'True',
            }.items()
        )

        # Add actions with staggered delays
        ld.add_action(controller_launch)
        ld.add_action(TimerAction(
            period=5.0 + i * 1.0,  # Stagger Nav2 bringup
            actions=[nav2_bringup]
        ))

    # Single RViz instance to visualize all robots
    rviz_launch = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config_path],
        output='screen',
        name='rviz2'
    )

    ld.add_action(TimerAction(
        period=8.0,  # Start RViz after all robots
        actions=[rviz_launch]
    ))

    return ld