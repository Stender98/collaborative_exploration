from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import GroupAction, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import yaml
import tempfile

def generate_launch_description():
    # Print for debugging
    print("Generating launch description...")

    # Define paths
    config_dir = os.path.join(get_package_share_directory('epuck_nav2_pkg'), 'config')
    rviz_config_path = os.path.join(config_dir, 'nav2_rviz_config.rviz')

    webots_controller_arg = DeclareLaunchArgument(
        'webots_controller',
        default_value='',
        description='Path to the Webots controller executable'
    )

    robot_count_arg = DeclareLaunchArgument(
        'robot_count',
        default_value='2',
        description='Number of robots to launch'
    )

    ld = LaunchDescription()

    ld.add_action(webots_controller_arg)
    ld.add_action(robot_count_arg)

    spawn_robots = OpaqueFunction(function=launch_robots)
    
    # Load and modify Slam Toolbox parameters
    start_slam_toolbox = OpaqueFunction(function=launch_slam_toolbox)

    ld.add_action(start_slam_toolbox)
    ld.add_action(TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg="Launching robot controllers..."),
            spawn_robots
        ]
    ))

    # RViz launch
    rviz_launch = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config_path],
        output='screen',
        name='rviz2'
    )

    #ld.add_action(rviz_launch)

    return ld

def launch_robots(context, *args, **kwargs):
    repo_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../epuck_nav2_pkg'))
    controller_path = os.path.join(repo_dir, 'scripts/swarm_obstacle_avoidance.py')
    assert os.path.exists(controller_path), f"Missing controller script: {controller_path}"
    
    # Get robot count and webots controller path from LaunchConfigurations
    num_robots = int(LaunchConfiguration('robot_count').perform(context))
    webots_controller = LaunchConfiguration('webots_controller').perform(context)

    # Generate namespaces
    namespaces = [f'robot{i}' for i in range(num_robots)]

    actions = []

    # Launch each robot controller
    for namespace in namespaces:
        # Create a launch action for each controller
        controller_launch = ExecuteProcess(
            cmd=[webots_controller, f'--robot-name={namespace}', controller_path],
            output='screen',
            name=f'epuck_controller_{namespace}',
            shell=True
        )

        # Add log info and controller launch action
        actions.append(LogInfo(msg=f"Launching EPuck controller for {namespace}"))
        actions.append(controller_launch)

    return actions

def launch_slam_toolbox(context, *args, **kwargs):
    # Get the SLAM Toolbox config path and namespaces
    config_dir = os.path.join(get_package_share_directory('epuck_nav2_pkg'), 'config')
    slam_toolbox_config_path = os.path.join(config_dir, 'slam_toolbox_params.yaml')
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    num_robots = int(LaunchConfiguration('robot_count').perform(context))
    
    namespaces = [f'robot{i}' for i in range(num_robots)]
    
    with open(slam_toolbox_config_path, 'r') as file:
        slam_toolbox_config = yaml.safe_load(file)

    odom_frames = [f'{ns}/odom' for ns in namespaces]
    base_frames = [f'{ns}/base_footprint' for ns in namespaces]
    scan_topics = [f'{ns}/scan' for ns in namespaces]

    # Update the configuration
    slam_toolbox_config['slam_toolbox']['ros__parameters'].update({
        'odom_frames': odom_frames,
        'base_frames': base_frames,
        'scan_topics': scan_topics
    })

    # Write the modified config to a temporary file
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as temp_file:
        yaml.dump(slam_toolbox_config, temp_file)
        temp_slam_toolbox_config_path = temp_file.name

    slam_toolbox_bringup = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'slam_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'True',
                    'autostart': 'True',
                    'params_file': temp_slam_toolbox_config_path
                }.items(),
            )
    
    return [slam_toolbox_bringup]