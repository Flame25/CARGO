"""Launch file for the takeoff behavior."""
import os

from ament_index_python.packages import get_package_share_directory
from cargo_core.declare_launch_arguments_from_config_file import DeclareLaunchArgumentsFromConfigFile
from cargo_core.launch_configuration_from_config_file import LaunchConfigurationFromConfigFile
from cargo_core.launch_plugin_utils import get_available_plugins
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

BEHAVIOR_NAME = 'takeoff'


def generate_launch_description() -> LaunchDescription:
    """Entrypoint."""
    # Get default configuration file
    package_folder = get_package_share_directory('cargo_behavior_motion')
    behavior_config_file = os.path.join(package_folder,
                                        BEHAVIOR_NAME +
                                        '_behavior/config/config_default.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('log_level',
                              description='Logging level',
                              default_value='info'),
        DeclareLaunchArgument('use_sim_time',
                              description='Use simulation clock if true',
                              default_value='false'),
        DeclareLaunchArgument('namespace',
                              description='Drone namespace',
                              default_value=EnvironmentVariable(
                                  'AEROSTACK2_SIMULATION_DRONE_ID')),
        DeclareLaunchArgument('plugin_name',
                              description='Plugin name',
                              choices=get_available_plugins(
                                  'cargo_behavior_motion', BEHAVIOR_NAME)),
        DeclareLaunchArgumentsFromConfigFile(
            name='behavior_config_file', source_file=behavior_config_file,
            description='Path to behavior configuration file'),
        Node(
            package='cargo_behavior_motion',
            executable=BEHAVIOR_NAME + '_behavior_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            arguments=['--ros-args', '--log-level',
                       LaunchConfiguration('log_level')],
            emulate_tty=True,
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'plugin_name': LaunchConfiguration('plugin_name'),
                },
                LaunchConfigurationFromConfigFile(
                    'behavior_config_file',
                    default_file=behavior_config_file)
            ]
        )])
