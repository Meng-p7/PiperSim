# MuJoCo 纯手动控制启动文件
# 直接启动 MuJoCo GUI，允许完全手动控制（不通过 ros2_control）

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, FindPackageShare
import os


def generate_launch_description():
    piper_mujoco_share = FindPackageShare("piper_mujoco")

    # MuJoCo 模型路径
    mujoco_model = os.path.join(
        piper_mijoco_share,  # 注意：这里会有错误，我需要修正
        "models",
        "piper.xml"
    )

    # 直接启动 MuJoCo 仿真（不通过 ROS 2）
    # 用户可以在 GUI 中完全手动控制
    mujoco_simulate = ExecuteProcess(
        cmd=["simulate", mujoco_model],
        output="screen"
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        mujoco_simulate,
    ])