from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
    DeclareLaunchArgument,
    LogInfo
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction
from nav2_common.launch import ReplaceString

def generate_launch_description():
    # Print for debugging
    print("Generating launch description...")

    # Define paths
    repo_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../epuck_nav2_pkg'))
    controller_path = os.path.join(repo_dir, 'scripts/swarm_nav_controller.py')
    config_dir = os.path.join(get_package_share_directory('epuck_nav2_pkg'), 'config')
    rviz_config_path = os.path.join(config_dir, 'nav2_rviz_config.rviz')
    config_path = os.path.join(config_dir, f'nav2_params_swarm.yaml')

    # Webots controller path
    webots_controller = '/usr/local/webots/webots-controller' if os.getenv('USER') == 'markus' else '/snap/webots/27/usr/share/webots/webots-controller'

    # Nav2 launch file path
    nav2_launch_file = os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_my_launch.py')

    # Assert paths to help with debugging
    assert os.path.exists(controller_path), f"Missing controller script: {controller_path}"
    assert os.path.exists(config_dir), f"Missing config directory: {config_dir}"
    #assert os.path.exists(rviz_config_path), f"Missing RViz config: {rviz_config_path}"
    assert os.path.exists(nav2_launch_file), f"Missing Nav2 launch file: {nav2_launch_file}"

    # Declare launch arguments
    num_robots_arg = DeclareLaunchArgument(
        'num_robots',
        default_value='2',
        description='Number of robots to launch'
    )

    ld = LaunchDescription()
    ld.add_action(num_robots_arg)

    # Set number of robots statically for now
    num_robots = 2

    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    slam_toolbox_bringup = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'slam_launch.py')
                ),
            )
    
    ld.add_action(TimerAction(
        period=5.0 + num_robots,
        actions=[
            LogInfo(msg="Launching SLAM Toolbox..."),
            slam_toolbox_bringup
        ]
    ))

    # Launch loop
    for i in range(num_robots):
        namespace= f'robot{i}'
       
        # Dynamically namespace the parameter file
        namespaced_params = ReplaceString(
            source_file=config_path,
            replacements={'/namespace': f'/{namespace}',
                          '/framenamespace': f'{namespace}',
                          }  
        )
        

        # EPuck Webots controller
        controller_launch = ExecuteProcess(
            cmd=[webots_controller, f'--robot-name={namespace}', controller_path],
            output='screen',
            name=f'epuck_controller_{namespace}',
            shell=True
        )

        # Nav2 bringup
        nav2_bringup = GroupAction([
            PushRosNamespace(namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch_file),
                launch_arguments={
                    'namespace': namespace,
                    'slam': 'True',
                    'params_file': namespaced_params,
                    'use_sim_time': 'True',
                    'autostart': 'True',
                }.items()
            )
]       )

        # Add controller
        ld.add_action(LogInfo(msg=f"Launching EPuck controller for {namespace}"))
        ld.add_action(controller_launch)

        # Staggered Nav2 startup
        ld.add_action(TimerAction(
            period=10.0 + i * 1.0,
            actions=[
                LogInfo(msg=f"Launching Nav2 for {namespace} with config: {config_path}"),
                nav2_bringup
            ]
        ))
    ld.add_action(LogInfo(msg=f"Attempting to include Nav2 launch file: {nav2_launch_file}"))

    # RViz launch
    rviz_launch = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config_path],
        output='screen',
        name='rviz2'
    )

    ld.add_action(TimerAction(
        period=8.0 + num_robots,
        actions=[
            LogInfo(msg="Launching RViz2..."),
            rviz_launch
        ]
    ))

    return ld
