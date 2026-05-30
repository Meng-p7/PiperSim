import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    piper_desc_dir = get_package_share_directory("piper_description")

    # 使用 Gazebo 专用 URDF（碰撞用简单几何体，避免插件崩溃）
    urdf_path = os.path.join(piper_desc_dir, "urdf", "piper_gazebo.urdf")

    with open(urdf_path, "r") as f:
        robot_desc = f.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            "world", default_value="", description="Gazebo world file"),

        # Robot state publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_desc, "use_sim_time": True}],
            output="screen",
        ),

        # Gazebo server + client
        ExecuteProcess(
            cmd=[
                "gazebo", "--verbose", "-s", "libgazebo_ros_init.so",
                "-s", "libgazebo_ros_factory.so",
            ],
            output="screen",
        ),

        # Spawn robot into Gazebo
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            name="spawn_piper",
            output="screen",
            arguments=[
                "-entity", "piper",
                "-file", urdf_path,
                "-x", "0", "-y", "0", "-z", "0.01",
            ],
        ),
    ])
