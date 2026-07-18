# 真机硬件启动文件
# 启动 ros2_control_node（PiperHardware 插件）+ 控制器激活

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
)
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    piper_desc_share = FindPackageShare("piper_description")

    # 加载 URDF（xacro），使用真机模式
    xacro_file = PathJoinSubstitution([piper_desc_share, "urdf", "piper.urdf.xacro"])
    robot_description = Command(["xacro ", xacro_file, " real_hardware:=true"])

    # 真机控制器配置文件
    controllers_yaml = os.path.join(
        get_package_share_directory("piper_control"),
        "config", "piper_controllers.yaml",
    )

    # 机器人状态发布器
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # ros2_control 控制管理器（真机硬件）
    # 注意：不设置 name（保持默认 ros2_control_node），
    # 这样 --params-file 的顶层 controller_manager: 不会被剥离，
    # 控制器才能正确读取到参数（与 XML <param from> 行为一致）
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": False},
            controllers_yaml,
        ],
    )

    # 使用 spawner 加载并激活控制器
    spawn_joint_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_joint_state_broadcaster",
        arguments=["joint_state_broadcaster", "--controller-manager-timeout", "30", "-c", "/controller_manager"],
        output="screen",
    )

    spawn_joint_trajectory = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_joint_trajectory_controller",
        arguments=["joint_trajectory_controller", "--controller-manager-timeout", "30", "-c", "/controller_manager"],
        output="screen",
    )

    spawn_gripper = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_gripper_controller",
        arguments=["gripper_controller", "--controller-manager-timeout", "30", "-c", "/controller_manager"],
        output="screen",
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        spawn_joint_broadcaster,
        TimerAction(period=3.0, actions=[spawn_joint_trajectory]),
        TimerAction(period=6.0, actions=[spawn_gripper]),
    ])
