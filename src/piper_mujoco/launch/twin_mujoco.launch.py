#!/usr/bin/env python3
"""
数字孪生MuJoCo启动文件

使用MuJoCo物理引擎 + joint_trajectory_controller
通过同步脚本实时跟随真机位置

注意：不启动robot_state_publisher，避免与真机启动文件冲突
"""

from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument, LogInfo
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    piper_desc_share = FindPackageShare("piper_description")
    piper_mujoco_share = FindPackageShare("piper_mujoco")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # 使用MuJoCo硬件接口
    robot_description = Command([
        "xacro ",
        PathJoinSubstitution([piper_desc_share, "urdf", "piper.urdf.xacro"]),
        " sim_mujoco:=true",
    ])

    # 控制器配置
    twin_yaml = PathJoinSubstitution([
        piper_mujoco_share, "config", "twin_controllers.yaml"
    ])

    # MuJoCo ros2_control节点（启用GUI）
    # 注意：使用不同的节点名称，避免与真机冲突
    mujoco_node = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        name="mujoco_ros2_control_node",  # 使用特定名称
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": use_sim_time},
            {"headless": False},  # 显示MuJoCo窗口
            twin_yaml,
        ],
    )

    # 启动控制器（指定MuJoCo的controller_manager）
    spawn_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout", "30",
            "-c", "/mujoco_ros2_control_node/controller_manager",  # 指定节点
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    spawn_trajectory = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "mujoco_joint_trajectory_controller",  # 使用特定名称
            "--controller-manager-timeout", "30",
            "-c", "/mujoco_ros2_control_node/controller_manager",  # 指定节点
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        LogInfo(msg="=== Digital Twin MuJoCo Mode ==="),
        LogInfo(msg="MuJoCo physics simulation + trajectory controller"),
        LogInfo(msg="Node: mujoco_ros2_control_node (avoid conflict with real hardware)"),

        mujoco_node,

        TimerAction(period=2.0, actions=[spawn_broadcaster]),
        TimerAction(period=4.0, actions=[spawn_trajectory]),
    ])