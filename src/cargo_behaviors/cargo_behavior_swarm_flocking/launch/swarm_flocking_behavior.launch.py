"""Launch file for the swarm behavior."""

import os

from ament_index_python.packages import get_package_share_directory
from cargo_core.declare_launch_arguments_from_config_file import DeclareLaunchArgumentsFromConfigFile
from cargo_core.launch_configuration_from_config_file import LaunchConfigurationFromConfigFile
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Entrypoint."""
    # Get default configuration file
    package_folder = get_package_share_directory('cargo_behaviors_swarm_flocking')
    behavior_config_file = os.path.join(package_folder,
                                        'config/config_default.yaml')
    print(behavior_config_file)
    return LaunchDescription([
        DeclareLaunchArgument('log_level',
                              description='Logging level',
                              default_value='info'),
        DeclareLaunchArgument('use_sim_time',
                              description='Use simulation clock if true',
                              default_value='false'),

        DeclareLaunchArgumentsFromConfigFile(
            name='behavior_config_file', source_file=behavior_config_file,
            description='Path to behavior configuration file'),
        Node(
            package='cargo_behaviors_swarm_flocking',
            executable='swarm_flocking_behavior_node',
            namespace='Swarm',
            name='SwarmFlockingBehavior',
            output='screen',
            arguments=['--ros-args', '--log-level',
                       LaunchConfiguration('log_level')],
            emulate_tty=True,
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                },
                LaunchConfigurationFromConfigFile(
                    'behavior_config_file',
                    default_file=behavior_config_file)
            ]
        )])

