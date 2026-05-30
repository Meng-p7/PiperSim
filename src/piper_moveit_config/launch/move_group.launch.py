import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package directories
    piper_desc_share = FindPackageShare("piper_description")
    piper_moveit_share = FindPackageShare("piper_moveit_config")

    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    use_rviz = LaunchConfiguration("use_rviz", default="true")

    # File paths
    urdf_path = PathJoinSubstitution([piper_desc_share, "urdf", "piper.urdf"])
    srdf_path = PathJoinSubstitution([piper_moveit_share, "config", "piper.srdf"])
    moveit_rviz_config = PathJoinSubstitution([piper_moveit_share, "rviz", "moveit.rviz"])

    # Robot description
    robot_description = Command(["cat ", urdf_path])
    robot_description_semantic = Command(["cat ", srdf_path])

    # Move group node (planning-only, no ros2_control / no execution)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
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
                "publish_robot_description_semantic": True,
                "capabilities": "",
                "disable_capabilities": "",
                "monitor_dynamics": False,
                "moveit_manage_controllers": True,
                "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
                "trajectory_execution.allowed_execution_duration_scaling": 1.2,
                "trajectory_execution.allowed_goal_duration_margin": 0.5,
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
                "use_sim_time": use_sim_time,
                "moveit_simple_controller_manager": {
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
                },
            },
        ],
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
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
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        robot_state_publisher,
        move_group_node,
        rviz_node,
    ])
