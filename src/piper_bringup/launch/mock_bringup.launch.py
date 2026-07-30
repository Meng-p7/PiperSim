# Mock 硬件启动文件（无 Gazebo、无真机）
# 启动 robot_state_publisher + ros2_control_node + 控制器（带时序避免竞态）

from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    piper_desc_share = FindPackageShare("piper_description")

    # 加载 URDF（xacro），使用 mock 硬件模式
    robot_description = Command([
        "xacro ",
        PathJoinSubstitution([piper_desc_share, "urdf", "piper.urdf.xacro"]),
        " mock_hardware:=true",
    ])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    controllers_yaml = PathJoinSubstitution([
        piper_desc_share, "config", "mock_controllers.yaml",
    ])

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[controllers_yaml],
        # Humble subscribes on ~/robot_description; Jazzy uses
        # robot_description. Both are fed by robot_state_publisher.
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("robot_description", "/robot_description"),
        ],
    )

    # 顺序加载并激活控制器，避免并行 spawner 对 controller_manager 的竞态
    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout", "30",
            "-c", "/controller_manager",
            "--param-file", controllers_yaml,
        ],
        output="screen",
    )

    spawn_joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager-timeout", "30",
            "-c", "/controller_manager",
            "--param-file", controllers_yaml,
        ],
        output="screen",
    )

    spawn_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager-timeout", "30",
            "-c", "/controller_manager",
            "--param-file", controllers_yaml,
        ],
        output="screen",
    )

    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        TimerAction(period=2.0, actions=[spawn_joint_state_broadcaster]),
        TimerAction(period=4.0, actions=[spawn_joint_trajectory_controller]),
        TimerAction(period=6.0, actions=[spawn_gripper_controller]),
    ])
