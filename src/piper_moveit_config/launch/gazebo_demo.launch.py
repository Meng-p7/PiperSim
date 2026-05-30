import os
import subprocess
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


def generate_launch_description():
    piper_desc_share = FindPackageShare("piper_description")
    piper_moveit_share = FindPackageShare("piper_moveit_config")

    # Arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    gui = LaunchConfiguration("gui", default="false")

    # File paths
    urdf_path = PathJoinSubstitution([piper_desc_share, "urdf", "piper_gazebo.urdf"])
    srdf_path = PathJoinSubstitution([piper_moveit_share, "config", "piper.srdf"])
    moveit_rviz_config = PathJoinSubstitution([piper_moveit_share, "rviz", "moveit.rviz"])

    # Load gazebo_controllers.yaml absolute path for URDF placeholder replacement
    from ament_index_python.packages import get_package_share_directory
    gazebo_controllers_yaml = os.path.join(
        get_package_share_directory("piper_moveit_config"),
        "config", "gazebo_controllers.yaml"
    )

    # Load URDF from source and replace placeholder with actual path
    urdf_src = os.path.join(
        get_package_share_directory("piper_description"), "urdf", "piper_gazebo.urdf"
    )
    with open(urdf_src, "r") as f:
        urdf_full = f.read().replace("__GAZEBO_CONTROLLERS_YAML__", gazebo_controllers_yaml)

    # Write modified URDF to temp file for spawn_entity (-file mode)
    import tempfile
    urdf_tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".urdf", delete=False, prefix="piper_gazebo_"
    )
    urdf_tmp.write(urdf_full)
    urdf_tmp.close()
    urdf_tmp_path = urdf_tmp.name

    # Robot description for move_group/rviz (gazebo tags stripped by robot_state_publisher)
    robot_description = Command(["cat ", urdf_path])
    robot_description_semantic = Command(["cat ", srdf_path])

    # MoveIt parameters
    moveit_params = {
        "robot_description": robot_description,
        "robot_description_semantic": robot_description_semantic,
        "robot_description_kinematics": {
            "piper_arm": {
                "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
                "kinematics_solver_search_resolution": 0.005,
                "kinematics_solver_timeout": 0.05,
                "kinematics_solver_attempts": 3,
            },
        },
        "publish_robot_description_semantic": True,
        "moveit_manage_controllers": True,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "trajectory_execution.allowed_execution_duration_scaling": 1.5,
        "trajectory_execution.allowed_goal_duration_margin": 1.0,
        "trajectory_execution.trajectory_monitoring": True,
        "trajectory_execution.controller_connection_timeout": 30.0,
        "planning_scene_monitor.publish_robot_description": True,
        "planning_scene_monitor.publish_robot_description_semantic": True,
        "planning_scene_monitor.publish_planning_scene": True,
        "planning_scene_monitor.wait_for_initial_state_timeout": 10.0,
        "planning_pipelines.pipelines": ["ompl"],
        "planning_pipelines.ompl.planning_plugin": "ompl_interface/OMPLPlanner",
        "planning_pipelines.ompl.request_adapters": (
            "default_planner_request_adapters/AddTimeOptimalParameterization "
            "default_planner_request_adapters/FixWorkspaceBounds "
            "default_planner_request_adapters/FixStartStateBounds "
            "default_planner_request_adapters/FixStartStateCollision "
            "default_planner_request_adapters/FixStartStatePathConstraints"
        ),
        "default_planner_config": "RRTConnect",
        "use_sim_time": use_sim_time,
    }

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
    )

    # Gazebo plugin path
    from ament_index_python.packages import get_package_prefix
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

    # Gazebo server only (headless)
    gazebo_server = ExecuteProcess(
        cmd=[
            "gzserver",
            "--verbose",
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
        ],
        output="screen",
        condition=UnlessCondition(gui),
        additional_env={"GAZEBO_PLUGIN_PATH": gazebo_plugin_path},
    )

    # Gazebo full (server + client GUI)
    gazebo_gui = ExecuteProcess(
        cmd=[
            "gazebo",
            "--verbose",
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
        ],
        output="screen",
        condition=IfCondition(gui),
        additional_env={"GAZEBO_PLUGIN_PATH": gazebo_plugin_path},
    )

    # Spawn robot - use -file to pass full URDF (with gazebo tags) directly
    # robot_state_publisher strips <gazebo> tags from the published topic,
    # so we pass the file directly to ensure the gazebo_ros2_control plugin loads
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_piper",
        output="screen",
        arguments=[
            "-entity", "piper",
            "-file", urdf_tmp_path,
            "-x", "0", "-y", "0", "-z", "0.01",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Controller spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["piper_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["piper_gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Move group
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            moveit_params,
            {"moveit_simple_controller_manager": {
                "controller_names": ["piper_arm_controller", "piper_gripper_controller"],
                "piper_arm_controller": {
                    "action_ns": "follow_joint_trajectory",
                    "type": "FollowJointTrajectory",
                    "default": True,
                    "joints": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
                },
                "piper_gripper_controller": {
                    "action_ns": "follow_joint_trajectory",
                    "type": "FollowJointTrajectory",
                    "joints": ["joint7"],
                },
            }},
        ],
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", moveit_rviz_config],
        parameters=[
            {
                "robot_description": robot_description,
                "robot_description_semantic": robot_description_semantic,
                "robot_description_kinematics": {
                    "piper_arm": {
                        "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
                        "kinematics_solver_search_resolution": 0.005,
                        "kinematics_solver_timeout": 0.05,
                        "kinematics_solver_attempts": 3,
                    },
                },
            }
        ],
    )

    # Delay controller spawners: wait for Gazebo and spawn to be ready
    delayed_spawners = TimerAction(
        period=12.0,
        actions=[
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            gripper_controller_spawner,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "gui", default_value="false", description="Enable Gazebo GUI (needs GPU)"
        ),
        robot_state_publisher,
        gazebo_server,
        gazebo_gui,
        spawn_entity,
        delayed_spawners,
        move_group_node,
        rviz_node,
    ])
