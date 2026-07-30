from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("piper_description")
    robot_desc = Command([
        "xacro ",
        PathJoinSubstitution([
            package_share, "urdf", "piper.urdf.xacro",
        ]),
        " mock_hardware:=true",
    ])
    rviz_path = PathJoinSubstitution([
        package_share, "rviz", "piper.rviz",
    ])

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
