import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    piper_desc_dir = get_package_share_directory("piper_description")
    piper_calib_dir = get_package_share_directory("piper_calibration")
    piper_ctrl_dir = get_package_share_directory("piper_control")

    urdf_xacro_path = os.path.join(piper_desc_dir, "urdf", "piper.urdf.xacro")
    rviz_path = os.path.join(piper_desc_dir, "rviz", "piper.rviz")
    sim_params = os.path.join(
        piper_calib_dir, "config", "calibration_params.yaml")
    real_params = os.path.join(
        piper_calib_dir, "config", "real_calibration_params.yaml")
    real_eye_to_hand_params = os.path.join(
        piper_calib_dir, "config", "real_eye_to_hand_params.yaml")
    controllers_yaml = os.path.join(
        piper_ctrl_dir, "config", "piper_controllers.yaml")

    # Use xacro to process the URDF
    robot_desc = Command([
        'xacro ', urdf_xacro_path,
        ' mock_hardware:=false',
        ' real_hardware:=true'
    ])

    is_sim = IfCondition(
        PythonExpression(["'", LaunchConfiguration("mode"), "' == 'sim'"]))
    # eye_in_hand模式：mode=real 且 eye_mode=eye_in_hand
    is_real_eye_in_hand = IfCondition(
        PythonExpression(["'", LaunchConfiguration("mode"), "' == 'real' and '", LaunchConfiguration("eye_mode"), "' == 'eye_in_hand'"]))
    # eye_to_hand模式：mode=real 且 eye_mode=eye_to_hand
    is_real_eye_to_hand = IfCondition(
        PythonExpression(["'", LaunchConfiguration("mode"), "' == 'real' and '", LaunchConfiguration("eye_mode"), "' == 'eye_to_hand'"]))

    return LaunchDescription([
        DeclareLaunchArgument(
            "mode", default_value="sim",
            description="sim (Gazebo + random poses) or real (RealSense + manual)"),
        DeclareLaunchArgument(
            "eye_mode", default_value="eye_in_hand",
            description="eye_in_hand or eye_to_hand calibration mode"),

        # 机器人状态发布器（仅仿真模式，真机模式由MoveIt提供）
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_desc}],
            condition=is_sim,
        ),

        # --- 仅仿真模式：控制器管理器 + 加载器 ---
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            name="controller_manager",
            output="screen",
            condition=is_sim,
            parameters=[
                {"robot_description": robot_desc},
                controllers_yaml,
            ],
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster",
                       "--controller-manager", "/controller_manager"],
            output="screen",
            condition=is_sim,
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["forward_position_controller",
                       "--controller-manager", "/controller_manager"],
            output="screen",
            condition=is_sim,
        ),

        # 标定节点（不同模式使用不同配置）
        Node(
            package="piper_calibration",
            executable="calibration_node",
            name="piper_calibration",
            output="screen",
            parameters=[sim_params],
            condition=is_sim,
        ),

        Node(
            package="piper_calibration",
            executable="calibration_node",
            name="piper_calibration",
            output="screen",
            parameters=[real_params],
            condition=is_real_eye_in_hand,
        ),

        # eye-to-hand 模式标定节点（真机）
        Node(
            package="piper_calibration",
            executable="calibration_node",
            name="piper_calibration",
            output="screen",
            parameters=[real_eye_to_hand_params],
            condition=is_real_eye_to_hand,
        ),

        # RViz2（仅仿真模式，真机模式由MoveIt提供）
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_path],
            condition=is_sim,
        ),
    ])
