# 真机硬件启动文件
# 启动 ros2_control_node（PiperHardware 插件）+ 控制器激活

import os
import re
import subprocess
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    TimerAction,
)
from launch.events import Shutdown
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def validate_can_interface(context):
    """Fail before starting hardware/RViz when SocketCAN is not ready."""
    can_interface = LaunchConfiguration("can").perform(context)
    if not re.fullmatch(r"[A-Za-z0-9_][A-Za-z0-9_.-]{0,14}", can_interface):
        raise RuntimeError(f"Invalid SocketCAN interface name: {can_interface!r}")

    try:
        result = subprocess.run(
            ["ip", "-details", "link", "show", can_interface],
            check=False,
            capture_output=True,
            text=True,
        )
    except FileNotFoundError as exc:
        raise RuntimeError(
            "SocketCAN preflight requires iproute2 (`ip` command)"
        ) from exc

    if result.returncode != 0:
        raise RuntimeError(
            f"SocketCAN interface {can_interface!r} does not exist. "
            f"Run: bash src/piper_control/scripts/can_activate.sh "
            f"{can_interface} 1000000"
        )

    output = result.stdout
    first_line = output.splitlines()[0] if output.splitlines() else ""
    flags_match = re.search(r"<([^>]*)>", first_line)
    flags = set(flags_match.group(1).split(",")) if flags_match else set()
    state_match = re.search(r"\bcan state ([A-Z-]+)", output)
    bitrate_match = re.search(r"\bbitrate\s+([0-9]+)", output)
    can_state = state_match.group(1) if state_match else "unknown"
    bitrate = int(bitrate_match.group(1)) if bitrate_match else None

    problems = []
    if "link/can" not in output:
        problems.append("not a SocketCAN device")
    if "UP" not in flags:
        problems.append("interface is DOWN")
    if can_state != "ERROR-ACTIVE":
        problems.append(f"CAN state is {can_state}")
    if bitrate != 1000000:
        problems.append(f"bitrate is {bitrate or 'unset'}, expected 1000000")

    if problems:
        raise RuntimeError(
            f"SocketCAN preflight failed for {can_interface}: "
            f"{'; '.join(problems)}. Run: "
            f"bash src/piper_control/scripts/can_activate.sh "
            f"{can_interface} 1000000"
        )

    return []


def generate_launch_description():
    piper_desc_share = FindPackageShare("piper_description")

    # 参数：是否启动robot_state_publisher（默认启动，但在Twin模式下不启动）
    start_rsp_arg = DeclareLaunchArgument(
        'start_robot_state_publisher',
        default_value='true',
        description='Whether to start robot_state_publisher (false in Twin mode)'
    )
    start_rsp = LaunchConfiguration('start_robot_state_publisher')

    can_arg = DeclareLaunchArgument(
        "can",
        default_value="can0",
        description="SocketCAN interface passed to the Piper hardware plugin",
    )
    can_interface = LaunchConfiguration("can")

    speed_percent_arg = DeclareLaunchArgument(
        "speed_percent",
        default_value="20",
        description="Piper CAN motion speed percentage (1-100)",
    )
    feedback_timeout_arg = DeclareLaunchArgument(
        "feedback_timeout_ms",
        default_value="250",
        description="Maximum age of complete joint/gripper feedback",
    )
    max_arm_step_arg = DeclareLaunchArgument(
        "max_arm_step",
        default_value="0.02",
        description="Maximum arm command change per 50 Hz write cycle (rad)",
    )
    max_gripper_step_arg = DeclareLaunchArgument(
        "max_gripper_step",
        default_value="0.002",
        description="Maximum gripper command change per write cycle (m)",
    )
    speed_percent = LaunchConfiguration("speed_percent")
    feedback_timeout_ms = LaunchConfiguration("feedback_timeout_ms")
    max_arm_step = LaunchConfiguration("max_arm_step")
    max_gripper_step = LaunchConfiguration("max_gripper_step")

    # 参数：是否启动轨迹控制器（标定模式时可禁用，允许手动拖动）
    calibration_mode_arg = DeclareLaunchArgument(
        'calibration_mode',
        default_value='false',
        description='If true, only start joint_state_broadcaster (no trajectory control)'
    )
    calibration_mode = LaunchConfiguration('calibration_mode')

    # 加载 URDF（xacro），使用真机模式
    xacro_file = PathJoinSubstitution([piper_desc_share, "urdf", "piper.urdf.xacro"])
    robot_description = Command([
        "xacro ", xacro_file,
        " real_hardware:=true",
        " calibration_mode:=", calibration_mode,
        " can_interface:=", can_interface,
        " speed_percent:=", speed_percent,
        " feedback_timeout_ms:=", feedback_timeout_ms,
        " max_arm_step:=", max_arm_step,
        " max_gripper_step:=", max_gripper_step,
    ])

    # 真机控制器配置文件
    controllers_yaml = os.path.join(
        get_package_share_directory("piper_control"),
        "config", "piper_controllers.yaml",
    )

    # 机器人状态发布器（仅在非Twin模式下启动）
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
        condition=IfCondition(start_rsp),
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
            {"use_sim_time": False},
            controllers_yaml,
        ],
        # Humble subscribes on ~/robot_description; Jazzy uses
        # robot_description. Both are fed by robot_state_publisher.
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("robot_description", "/robot_description"),
        ],
        on_exit=[
            EmitEvent(
                event=Shutdown(
                    reason="Real hardware controller_manager exited",
                )
            )
        ],
    )

    # 使用 spawner 加载并激活控制器
    spawn_joint_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_joint_state_broadcaster",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout", "30",
            "-c", "/controller_manager",
            "--param-file", controllers_yaml,
        ],
        output="screen",
    )

    spawn_joint_trajectory = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_joint_trajectory_controller",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager-timeout", "30",
            "-c", "/controller_manager",
            "--param-file", controllers_yaml,
        ],
        output="screen",
    )

    spawn_gripper = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_gripper_controller",
        arguments=[
            "gripper_controller",
            "--controller-manager-timeout", "30",
            "-c", "/controller_manager",
            "--param-file", controllers_yaml,
        ],
        output="screen",
    )

    return LaunchDescription([
        start_rsp_arg,
        can_arg,
        speed_percent_arg,
        feedback_timeout_arg,
        max_arm_step_arg,
        max_gripper_step_arg,
        calibration_mode_arg,
        OpaqueFunction(function=validate_can_interface),
        robot_state_publisher,
        controller_manager,
        spawn_joint_broadcaster,
        # Evaluate calibration_mode while this included launch still owns its
        # scoped launch configurations. A condition on the delayed Node itself
        # would be evaluated after the include scope has already been popped.
        TimerAction(
            period=3.0,
            actions=[spawn_joint_trajectory],
            condition=UnlessCondition(calibration_mode),
        ),
        TimerAction(
            period=6.0,
            actions=[spawn_gripper],
            condition=UnlessCondition(calibration_mode),
        ),
    ])
