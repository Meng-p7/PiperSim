import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    piper_desc_dir = get_package_share_directory("piper_description")
    piper_ctrl_dir = get_package_share_directory("piper_control")

    urdf_path = os.path.join(piper_desc_dir, "urdf", "piper.urdf")
    rviz_path = os.path.join(piper_desc_dir, "rviz", "piper.rviz")
    controllers_yaml = os.path.join(piper_ctrl_dir, "config", "piper_controllers.yaml")
    activate_script = os.path.join(piper_ctrl_dir, "scripts", "activate_controllers.sh")

    return LaunchDescription([
        DeclareLaunchArgument("can", default_value="can0", description="CAN interface"),

        # Robot state publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": open(urdf_path).read()}],
            output="screen",
        ),

        # ros2_control controller manager — launched via ExecuteProcess for correct --params-file handling
        ExecuteProcess(
            cmd=[
                "ros2", "run", "controller_manager", "ros2_control_node",
                "--ros-args",
                "-p", f"robot_description:={open(urdf_path).read()}",
                "-p", "use_sim_time:=false",
                "--params-file", controllers_yaml,
            ],
            output="screen",
        ),

        # Activate controllers after controller_manager is ready
        TimerAction(
            period=10.0,
            actions=[
                ExecuteProcess(
                    cmd=["bash", activate_script],
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
