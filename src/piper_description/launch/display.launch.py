import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory("piper_description")

    urdf_path = os.path.join(pkg_dir, "urdf", "piper.urdf")
    rviz_path = os.path.join(pkg_dir, "rviz", "piper.rviz")

    with open(urdf_path, "r") as f:
        robot_desc = f.read()

    return LaunchDescription([
        # 机器人状态发布器（从 URDF 发布 TF）
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_desc}],
        ),

        # 关节状态发布器 GUI（各关节滑块）
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
        ),

        # RViz2
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_path],
        ),
    ])
