import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    piper_desc_dir = get_package_share_directory("piper_description")
    piper_ctrl_dir = get_package_share_directory("piper_control")
    piper_moveit_dir = get_package_share_directory("piper_moveit_config")

    urdf_path = os.path.join(piper_desc_dir, "urdf", "piper.urdf")
    rviz_path = os.path.join(piper_moveit_dir, "rviz", "moveit.rviz")
    controllers_yaml = os.path.join(piper_ctrl_dir, "config", "piper_controllers.yaml")

    with open(urdf_path, "r") as f:
        robot_desc = f.read()

    return LaunchDescription([
        DeclareLaunchArgument("can", default_value="can0", description="CAN interface"),

        # Robot state publisher (TF from URDF)
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_desc}],
            output="screen",
        ),

        # ros2_control controller manager – loads piper_control/PiperHardware plugin
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            name="controller_manager",
            output="screen",
            parameters=[
                {"robot_description": robot_desc},
                controllers_yaml,
            ],
        ),

        # Delay spawners to wait for controller_manager to be ready
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                    output="screen",
                ),
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["piper_arm_controller", "--controller-manager", "/controller_manager"],
                    output="screen",
                ),
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["piper_gripper_controller", "--controller-manager", "/controller_manager"],
                    output="screen",
                ),
            ],
        ),

        # RViz2
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_path],
            output="screen",
        ),
    ])
