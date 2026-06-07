# Gazebo Classic + MoveIt 启动文件

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def generate_launch_description():
    piper_desc_share = FindPackageShare("piper_description")

    # 启动参数
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    gui = LaunchConfiguration("gui", default="true")

    # 加载 URDF（xacro），使用 Gazebo Classic 仿真模式
    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([piper_desc_share, "urdf", "piper.urdf.xacro"]),
        " sim_gazebo_classic:=true"
    ])

    # 机器人状态发布器
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_content, "use_sim_time": use_sim_time}],
    )

    # Gazebo 插件路径
    gazebo_plugin_path = os.environ.get("GAZEBO_PLUGIN_PATH", "")
    for pkg in ["gazebo_ros", "gazebo_ros2_control"]:
        try:
            pkg_lib = os.path.join(get_package_prefix(pkg), "lib")
            if pkg_lib not in gazebo_plugin_path:
                gazebo_plugin_path = pkg_lib + (
                    ":" + gazebo_plugin_path if gazebo_plugin_path else ""
                )
        except Exception:
            pass

    # 修复 Wayland 下 Gazebo GUI 问题：强制使用 X11
    gazebo_env = {"GAZEBO_PLUGIN_PATH": gazebo_plugin_path, "QT_QPA_PLATFORM": "xcb"}

    # 仅服务端（无界面）
    gazebo_server = ExecuteProcess(
        cmd=[
            "gzserver",
            "--verbose",
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
        ],
        output="screen",
        condition=UnlessCondition(gui),
        additional_env=gazebo_env,
    )

    # 服务端 + 图形界面
    gazebo_gui = ExecuteProcess(
        cmd=[
            "gazebo",
            "--verbose",
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
        ],
        output="screen",
        condition=IfCondition(gui),
        additional_env=gazebo_env,
    )

    # 生成机器人模型
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_piper",
        output="screen",
        arguments=[
            "-entity", "piper",
            "-topic", "/robot_description",
            "-x", "0", "-y", "0", "-z", "0.01",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # 控制器加载器
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # 延迟加载控制器：等待 Gazebo 和机器人生成就绪
    delayed_spawners = TimerAction(
        period=10.0,
        actions=[
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            gripper_controller_spawner,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "gui", default_value="true", description="Enable Gazebo GUI"
        ),
        robot_state_publisher,
        gazebo_server,
        gazebo_gui,
        spawn_entity,
        delayed_spawners,
    ])
