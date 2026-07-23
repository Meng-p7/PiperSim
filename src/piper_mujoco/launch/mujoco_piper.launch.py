# MuJoCo 仿真启动文件
# 使用 mujoco_ros2_control 实现 MoveIt 与 MuJoCo 的集成

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
    gui = LaunchConfiguration("gui", default="true")  # 启用MuJoCo GUI窗口

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

    # 控制器配置
    controllers_yaml = PathJoinSubstitution([
        piper_mujoco_share, "config", "mujoco_controllers.yaml",
    ])

    # MuJoCo ros2_control 节点
    ros2_control_node = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": use_sim_time},
            controllers_yaml,
        ],
    )

    # 控制器启动器（带延迟）
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

    spawn_joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager-timeout", "30",
            "-c", "/controller_manager",
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    spawn_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
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

        # 顺序启动控制器
        TimerAction(period=2.0, actions=[spawn_joint_state_broadcaster]),
        TimerAction(period=4.0, actions=[spawn_joint_trajectory_controller]),
        TimerAction(period=6.0, actions=[spawn_gripper_controller]),
    ])