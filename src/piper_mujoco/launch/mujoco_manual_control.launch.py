# MuJoCo 手动控制启动文件
# 只启动 joint_state_broadcaster，允许在 MuJoCo GUI 中手动拖动滑块

from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    piper_desc_share = FindPackageShare("piper_description")
    piper_mujoco_share = FindPackageShare("piper_mujoco")

    # 启动参数
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    gui = LaunchConfiguration("gui", default="true")

    # 加载 URDF（xacro），使用 MuJoCo 仿真模式
    robot_description = Command([
        "xacro ",
        PathJoinSubstitution([piper_desc_share, "urdf", "piper.urdf.xacro"]),
        " sim_mujoco:=true",
    ])

    # 机器人状态发布器
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": use_sim_time},
        ],
    )

    # 手动控制配置 - 只启动 joint_state_broadcaster
    manual_control_yaml = PathJoinSubstitution([
        piper_mujoco_share, "config", "manual_control.yaml",
    ])

    # MuJoCo ros2_control 节点（启用GUI）
    ros2_control_node = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": use_sim_time},
            {"headless": False},  # 启用 MuJoCo GUI 窗口
            manual_control_yaml,
        ],
    )

    # 只启动状态发布器（不启动trajectory controller）
    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout", "30",
            "-c", "/controller_manager",
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true", description="Enable MuJoCo GUI window"),

        robot_state_publisher,
        ros2_control_node,

        # 只启动状态发布器，允许手动控制
        TimerAction(period=2.0, actions=[spawn_joint_state_broadcaster]),
    ])