import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    piper_desc_dir = get_package_share_directory("piper_description")
    piper_calib_dir = get_package_share_directory("piper_calibration")
    piper_ctrl_dir = get_package_share_directory("piper_control")

    urdf_path = os.path.join(piper_desc_dir, "urdf", "piper.urdf")
    rviz_path = os.path.join(piper_desc_dir, "rviz", "piper.rviz")
    sim_params = os.path.join(
        piper_calib_dir, "config", "calibration_params.yaml")
    real_params = os.path.join(
        piper_calib_dir, "config", "real_calibration_params.yaml")
    controllers_yaml = os.path.join(
        piper_ctrl_dir, "config", "piper_controllers.yaml")

    with open(urdf_path, "r") as f:
        robot_desc = f.read()

    is_sim = IfCondition(
        PythonExpression(["'", LaunchConfiguration("mode"), "' == 'sim'"]))
    is_real = IfCondition(
        PythonExpression(["'", LaunchConfiguration("mode"), "' == 'real'"]))

    return LaunchDescription([
        DeclareLaunchArgument(
            "mode", default_value="sim",
            description="sim (Gazebo + random poses) or real (RealSense + manual)"),

        # Robot state publisher (always)
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_desc}],
        ),

        # --- Simulation only: controller_manager + spawners ---
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            name="controller_manager",
            output="screen",
            condition=is_sim,
            parameters=[
                {"robot_description": robot_desc},
                controllers_yaml,
            ],
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster",
                       "--controller-manager", "/controller_manager"],
            output="screen",
            condition=is_sim,
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["forward_position_controller",
                       "--controller-manager", "/controller_manager"],
            output="screen",
            condition=is_sim,
        ),

        # Calibration node (different config per mode)
        Node(
            package="piper_calibration",
            executable="calibration_node",
            name="piper_calibration",
            output="screen",
            parameters=[sim_params],
            condition=is_sim,
        ),

        Node(
            package="piper_calibration",
            executable="calibration_node",
            name="piper_calibration",
            output="screen",
            parameters=[real_params],
            condition=is_real,
        ),

        # RViz2 (always)
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_path],
        ),
    ])
