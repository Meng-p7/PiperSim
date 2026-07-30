#!/usr/bin/env python3

import os
import shutil
import subprocess
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def configure_twin_runtime(context, package_share):
    """Validate Twin-only dependencies before any real hardware node starts."""
    mode = LaunchConfiguration("mode").perform(context)
    if mode != "twin":
        return []

    if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        raise RuntimeError(
            "Twin mode requires DISPLAY or WAYLAND_DISPLAY before real hardware starts"
        )

    model_path = Path(package_share) / "models" / "piper.xml"
    if not model_path.is_file():
        raise RuntimeError(f"Twin MuJoCo model is missing: {model_path}")

    candidates = []
    configured_python = os.environ.get("PIPERSIM_MUJOCO_PYTHON")
    if configured_python:
        candidates.append(Path(configured_python))
    candidates.append(Path("/opt/pipersim-venv/bin/python"))
    candidates.append(Path.cwd() / ".venv-mujoco/bin/python")
    for parent in Path(package_share).parents:
        candidates.append(parent / ".venv-mujoco/bin/python")
    system_python = shutil.which("python3")
    if system_python:
        candidates.append(Path(system_python))

    checked = set()
    failures = []
    check_environment = os.environ.copy()
    check_environment["PYTHONNOUSERSITE"] = "1"
    for candidate in candidates:
        candidate_text = str(candidate)
        candidate_unusable = (
            not candidate.is_file() or not os.access(candidate_text, os.X_OK)
        )
        if candidate_text in checked or candidate_unusable:
            continue
        checked.add(candidate_text)
        try:
            result = subprocess.run(
                [
                    candidate_text,
                    "-c",
                    (
                        "import sys; import mujoco; import mujoco.viewer; "
                        "import rclpy; from sensor_msgs.msg import JointState; "
                        "mujoco.MjModel.from_xml_path(sys.argv[1])"
                    ),
                    str(model_path),
                ],
                check=False,
                capture_output=True,
                text=True,
                env=check_environment,
                timeout=10,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            failures.append(f"{candidate_text}: {exc}")
            continue
        if result.returncode == 0:
            return [
                SetEnvironmentVariable(
                    name="PIPERSIM_MUJOCO_PYTHON",
                    value=candidate_text,
                )
            ]
        detail = result.stderr.strip().splitlines()
        failures.append(
            f"{candidate_text}: {detail[-1] if detail else 'dependency check failed'}"
        )

    raise RuntimeError(
        "Twin preflight failed before real hardware startup. "
        "A Python interpreter must import mujoco.viewer, rclpy and sensor_msgs, "
        f"and parse {model_path}. Checked: {'; '.join(failures) or 'none'}. "
        "Create .venv-mujoco as documented in README."
    )


def generate_launch_description():
    # Declare arguments
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='mock',
        choices=['mock', 'sim', 'real', 'twin'],
        description='Runtime backend',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        choices=['true', 'false'],
    )
    can_arg = DeclareLaunchArgument('can', default_value='can0')
    speed_percent_arg = DeclareLaunchArgument(
        'speed_percent',
        default_value='20',
        description='Real-hardware CAN motion speed percentage (1-100)',
    )
    feedback_timeout_arg = DeclareLaunchArgument(
        'feedback_timeout_ms',
        default_value='250',
        description='Real-hardware feedback freshness timeout',
    )
    max_arm_step_arg = DeclareLaunchArgument(
        'max_arm_step',
        default_value='0.02',
        description='Maximum real arm command change per 50 Hz cycle (rad)',
    )
    max_gripper_step_arg = DeclareLaunchArgument(
        'max_gripper_step',
        default_value='0.002',
        description='Maximum real gripper command change per cycle (m)',
    )
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        choices=['true', 'false'],
        description='Disable the MuJoCo GUI in sim mode',
    )

    mode = LaunchConfiguration('mode')
    rviz = LaunchConfiguration('rviz')
    can = LaunchConfiguration('can')
    speed_percent = LaunchConfiguration('speed_percent')
    feedback_timeout_ms = LaunchConfiguration('feedback_timeout_ms')
    max_arm_step = LaunchConfiguration('max_arm_step')
    max_gripper_step = LaunchConfiguration('max_gripper_step')
    headless = LaunchConfiguration('headless')

    pkg_bringup = get_package_share_directory('piper_bringup')
    pkg_mujoco = get_package_share_directory('piper_mujoco')
    pkg_moveit = get_package_share_directory('piper_moveit_config')
    pkg_description = get_package_share_directory('piper_description')

    # Mode flags
    is_mock = PythonExpression(['"', mode, '" == "mock"'])
    is_sim = PythonExpression(['"', mode, '" == "sim"'])
    is_real = PythonExpression(['"', mode, '" == "real"'])
    is_twin = PythonExpression(['"', mode, '" == "twin"'])

    # URDF xacro command (based on mode)
    # Note: Twin mode uses real_hardware for MoveGroup (真机控制由real_bringup.launch.py处理)
    urdf_xacro = Command([
        'xacro ', os.path.join(pkg_description, 'urdf', 'piper.urdf.xacro'),
        ' mock_hardware:=', is_mock,
        ' real_hardware:=', PythonExpression([is_real, ' or ', is_twin]),
        ' sim_mujoco:=', is_sim,
        ' can_interface:=', can,
        ' speed_percent:=', speed_percent,
        ' feedback_timeout_ms:=', feedback_timeout_ms,
        ' max_arm_step:=', max_arm_step,
        ' max_gripper_step:=', max_gripper_step,
    ])

    # SRDF xacro command
    srdf_xacro = Command([
        'xacro ', os.path.join(pkg_moveit, 'config', 'piper.srdf.xacro')
    ])

    # MoveIt config files
    ompl_planning_config = os.path.join(pkg_moveit, 'config', 'ompl_planning.yaml')
    kinematics_config = os.path.join(pkg_moveit, 'config', 'kinematics.yaml')
    joint_limits_config = os.path.join(pkg_moveit, 'config', 'joint_limits.yaml')
    moveit_controllers_config = os.path.join(pkg_moveit, 'config', 'moveit_controllers.yaml')

    return LaunchDescription([
        mode_arg,
        rviz_arg,
        can_arg,
        speed_percent_arg,
        feedback_timeout_arg,
        max_arm_step_arg,
        max_gripper_step_arg,
        headless_arg,
        OpaqueFunction(
            function=configure_twin_runtime,
            args=[pkg_mujoco],
        ),

        # Mock mode - start hardware simulation
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'mock_bringup.launch.py'))
            ),
            LogInfo(msg='Starting in MOCK mode (RViz visualization only)')
        ], condition=IfCondition(is_mock)),

        # Sim mode - start MuJoCo simulation
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_mujoco, 'launch', 'mujoco_piper.launch.py')),
                launch_arguments={'headless': headless}.items(),
            ),
            LogInfo(msg='Starting in SIM mode (MuJoCo physics simulation)')
        ], condition=IfCondition(is_sim)),

        # Real mode - start real hardware
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'real_bringup.launch.py')),
                launch_arguments={
                    'can': can,
                    'speed_percent': speed_percent,
                    'feedback_timeout_ms': feedback_timeout_ms,
                    'max_arm_step': max_arm_step,
                    'max_gripper_step': max_gripper_step,
                }.items()
            ),
            LogInfo(msg='Starting in REAL mode (Real hardware control)')
        ], condition=IfCondition(is_real)),

        # Twin mode - digital twin (Real hardware + MuJoCo)
        GroupAction([
            # 启动真机控制（发布/joint_states，但不启动robot_state_publisher）
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'real_bringup.launch.py')),
                launch_arguments={
                    'can': can,
                    'start_robot_state_publisher': 'false',  # 不启动，由Twin模式统一管理
                    'speed_percent': speed_percent,
                    'feedback_timeout_ms': feedback_timeout_ms,
                    'max_arm_step': max_arm_step,
                    'max_gripper_step': max_gripper_step,
                }.items()
            ),
            # 启动MuJoCo + 同步节点（MuJoCo的joint_states发布到/mujoco/joint_states）
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_mujoco, 'launch', 'twin_mujoco.launch.py'))
            ),
            # robot_state_publisher：使用真机URDF，监听真机/joint_states发布TF
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[
                    {'robot_description': urdf_xacro},
                    {'use_sim_time': False},
                ],
                # 只订阅真机的/joint_states（MuJoCo的重映射到/mujoco/joint_states）
            ),
            LogInfo(msg='Starting in TWIN mode (Digital twin: Real + MuJoCo)'),
            LogInfo(msg='Real -> MuJoCo synchronization auto-enabled')
        ], condition=IfCondition(is_twin)),

        # MoveGroup - start for ALL modes (including real)
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                {'robot_description': urdf_xacro},
                {'robot_description_semantic': srdf_xacro},
                {'planning_plugin': 'ompl_interface/OMPLPlanner'},
                ompl_planning_config,
                kinematics_config,
                joint_limits_config,
                moveit_controllers_config,
                {'publish_robot_description_semantic': True},
                {'use_sim_time': PythonExpression(['"', mode, '" == "sim"'])},
                {'operation_mode': mode}
            ]
        ),

        # RViz - start for ALL modes (including real)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_moveit, 'config', 'moveit.rviz')],
            parameters=[
                {'robot_description': urdf_xacro},
                ompl_planning_config,
                kinematics_config,
                joint_limits_config,
                moveit_controllers_config,
                {'use_sim_time': PythonExpression(['"', mode, '" == "sim"'])}
            ],
            condition=IfCondition(rviz)
        )
    ])
