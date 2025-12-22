"""Launch file for the state estimator node with ground_truth_odometry_fuse plugin."""

__authors__ = 'Pedro Arias Pérez'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

package_folder = get_package_share_directory('cargo_state_estimator')
sys.path.append(package_folder + '/launch')


def generate_launch_description() -> LaunchDescription:
    from state_estimator_launch import get_launch_description_from_plugin
    return LaunchDescription(get_launch_description_from_plugin('ground_truth_odometry_fuse'))
