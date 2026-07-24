#!/usr/bin/env python3
"""
数字孪生MuJoCo启动文件

架构：
  真机(PiperHardware) → /joint_states → 同步脚本 → mujoco Python API → MuJoCo GUI

关键设计：
  - 不启动MuJoCo的ros2_control_node，避免与真机controller_manager冲突
  - 同步脚本直接使用mujoco Python包控制仿真模型
  - 真机的/joint_states由PiperHardware发布
  - MuJoCo窗口由同步脚本自动打开
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # 数字孪生同步脚本（直接控制MuJoCo，不依赖ros2_control_node）
    sync_node = Node(
        package="piper_mujoco",
        executable="digital_twin_sync_realtime.py",
        name="digital_twin_sync",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"sync_frequency": 50.0},
            {"lpf_alpha": 0.6},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        LogInfo(msg="=== Digital Twin MuJoCo Mode ==="),
        LogInfo(msg="Architecture: Real robot -> /joint_states -> Direct MuJoCo control"),
        LogInfo(msg="No MuJoCo ros2_control_node (avoid controller_manager conflict)"),
        LogInfo(msg="MuJoCo window will open automatically"),

        sync_node,
    ])
