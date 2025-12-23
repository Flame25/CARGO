"""Launch file for the motion controller node with differential_flatness plugin."""

import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

package_folder = get_package_share_directory('cargo_motion_controller')
sys.path.append(package_folder + '/launch')


def generate_launch_description() -> LaunchDescription:
    from controller_launch import get_launch_description_from_plugin
    return LaunchDescription(get_launch_description_from_plugin('pid_speed_controller'))
