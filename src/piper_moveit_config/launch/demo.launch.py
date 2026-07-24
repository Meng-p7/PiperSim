#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, GroupAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare arguments
    mode_arg = DeclareLaunchArgument('mode', default_value='mock')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    can_arg = DeclareLaunchArgument('can', default_value='can0')

    mode = LaunchConfiguration('mode')
    rviz = LaunchConfiguration('rviz')
    can = LaunchConfiguration('can')

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
        ' sim_mujoco:=', is_sim
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
                PythonLaunchDescriptionSource(os.path.join(pkg_mujoco, 'launch', 'mujoco_piper.launch.py'))
            ),
            LogInfo(msg='Starting in SIM mode (MuJoCo physics simulation)')
        ], condition=IfCondition(is_sim)),

        # Real mode - start real hardware
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'real_bringup.launch.py')),
                launch_arguments={'can': can}.items()
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
                    'start_robot_state_publisher': 'false'  # 不启动，由Twin模式统一管理
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
                    {'robot_description': Command([
                        'xacro ', os.path.join(pkg_description, 'urdf', 'piper.urdf.xacro'),
                        ' real_hardware:=true'
                    ])},
                    {'use_sim_time': True},
                ],
                # 只订阅真机的/joint_states（MuJoCo的重映射到/mujoco/joint_states）
            ),
            LogInfo(msg='Starting in TWIN mode (Digital twin: Real + MuJoCo)'),
            LogInfo(msg='Real -> MuJoCo synchronization auto-enabled')
        ], condition=IfCondition(is_twin)),

        # Wait for hardware startup
        ExecuteProcess(cmd=['sleep', '3'], output='screen'),

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
                {'use_sim_time': PythonExpression(['"', mode, '" == "sim" or "', mode, '" == "twin"'])},
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
                {'use_sim_time': PythonExpression(['"', mode, '" == "sim" or "', mode, '" == "twin"'])}
            ],
            condition=IfCondition(rviz)
        )
    ])