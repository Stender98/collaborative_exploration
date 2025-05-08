# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushROSNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml
from launch.actions import OpaqueFunction

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    log_level = LaunchConfiguration('log_level')
    robot_count = LaunchConfiguration('robot_count')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    remappings = [('/tf', '/tf'), ('/tf_static', '/tf_static')]

    # Environment variable for logging
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )
    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='True',
        description='Whether to use composed bringup'
    )
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )
    declare_robot_count_cmd = DeclareLaunchArgument(
        'robot_count',
        default_value='2',
        description='Number of robots to launch'
    )

    # Use OpaqueFunction to handle dynamic robot count
    def launch_robots(context, *args, **kwargs):
        num_robots = int(LaunchConfiguration('robot_count').perform(context))
        nav_instances_cmds = []

        for i in range(num_robots):
            namespace = f'robot{i}'

            temp_paramsfile = ReplaceString(
                source_file=params_file,
                replacements={'<robot_namespace>': ('/', namespace), '<robot_frame_namespace>': ('', namespace)},
            )

            configured_params = ParameterFile(
                RewrittenYaml(
                    source_file=temp_paramsfile,
                    root_key=namespace,
                    param_rewrites={},
                    convert_types=True,
                ),
                allow_substs=True,
            )

            # Specify the actions for each robot
            group = GroupAction(
                [
                    PushROSNamespace(namespace),
                    Node(
                        condition=IfCondition(use_composition),
                        name='nav2_container',
                        package='rclcpp_components',
                        executable='component_container_isolated',
                        parameters=[configured_params, {'autostart': autostart}],
                        arguments=['--ros-args', '--log-level', log_level],
                        remappings=remappings,
                        output='screen',
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(launch_dir, 'swarm_navigation_launch.py')
                        ),
                        launch_arguments={
                            'namespace': namespace,
                            'use_sim_time': use_sim_time,
                            'autostart': autostart,
                            'params_file': temp_paramsfile,
                            'use_composition': use_composition,
                            'container_name': 'nav2_container',
                        }.items(),
                    ),
                ]
            )

            nav_instances_cmds.append(group)

        return nav_instances_cmds

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_robot_count_cmd)

    # Add the OpaqueFunction to generate robot instances
    ld.add_action(OpaqueFunction(function=launch_robots))

    return ld