"""Launch the real-robot eye-to-hand calibration node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("piper_calibration"),
        "config",
        "real_eye_to_hand_params.yaml",
    )
    return LaunchDescription([
        Node(
            package="piper_calibration",
            executable="calibration_node",
            name="piper_calibration",
            output="screen",
            parameters=[params_file],
        )
    ])
