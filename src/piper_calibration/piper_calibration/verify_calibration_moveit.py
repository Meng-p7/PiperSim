"""
标定精度验证：AprilTag 检测 + MoveIt 运动规划。
通过 MoveIt 进行 IK 求解和轨迹规划，通过 ros2_control 执行。

用法：
  终端 1: ros2 launch piper_control real_bringup.launch.py can:=can0
  终端 2: python3 src/piper_calibration/piper_calibration/verify_calibration.py \\
            --result-file real_eye_in_hand_result.yaml --tag-id 1 --tag-size 0.057
"""
import argparse
import os
import sys
import time
import threading

import cv2
import cv2.aruco as aruco
import numpy as np
import pyrealsense2 as rs

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
    WorkspaceParameters
)
from moveit_msgs.srv import GetPositionIK, GetPositionFK
import tf2_ros

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
APRILTAG_DICT = cv2.aruco.DICT_APRILTAG_36h11


def load_calibration(path):
    import yaml
    with open(path, "r") as f:
        data = yaml.safe_load(f)
    T = np.eye(4)
    T[:3, :3] = np.array(data["rotation_matrix"])
    T[0, 3] = data["translation"]["x"]
    T[1, 3] = data["translation"]["y"]
    T[2, 3] = data["translation"]["z"]
    print(f"Loaded calibration: method={data.get('method')}, error={data.get('error'):.6f}")
    return T


class MoveItClient(Node):
    """MoveIt 运动规划客户端"""
    
    def __init__(self):
        super().__init__("moveit_verify_calibration")
        self._current_joints = None
        
        # 订阅关节状态
        self.create_subscription(JointState, "/joint_states", self._joint_cb, 10)
        
        # MoveIt MoveGroup action client
        self._move_group_client = ActionClient(self, MoveGroup, "/move_action")
        
        # IK 服务客户端
        self._ik_client = self.create_client(GetPositionIK, "/compute_ik")
        
        # FK 服务客户端
        self._fk_client = self.create_client(GetPositionFK, "/compute_fk")
        
        # TF
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        
        self.get_logger().info("MoveIt client initialized")
    
    def _joint_cb(self, msg):
        d = {}
        for name, pos in zip(msg.name, msg.position):
            d[name] = pos
        self._current_joints = d
    
    def get_current_q(self):
        if self._current_joints is None:
            return None
        return [self._current_joints.get(n, 0.0) for n in JOINT_NAMES]
    
    def get_ee_pose(self):
        """获取末端执行器位姿"""
        try:
            t = self._tf_buffer.lookup_transform(
                "base_link", "tool_0",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            pose = PoseStamped()
            pose.header.frame_id = "base_link"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = t.transform.translation.x
            pose.pose.position.y = t.transform.translation.y
            pose.pose.position.z = t.transform.translation.z
            pose.pose.orientation = t.transform.rotation
            return pose
        except Exception as e:
            self.get_logger().warn(f"Failed to get EE pose: {e}")
            return None
    
    def move_to_pose(self, target_pose, planning_time=5.0):
        """
        使用 MoveIt 移动到目标位姿
        
        Args:
            target_pose: PoseStamped 目标位姿
            planning_time: 规划时间限制（秒）
        
        Returns:
            bool: 是否成功
        """
        if not self._move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("MoveIt MoveGroup server not available")
            return False
        
        # 创建 MoveGroup goal
        goal = MoveGroup.Goal()
        
        # 设置规划请求
        request = MotionPlanRequest()
        request.group_name = "manipulator"
        request.num_planning_attempts = 10
        request.allowed_planning_time = planning_time
        request.max_velocity_scaling_factor = 0.5
        request.max_acceleration_scaling_factor = 0.5
        
        # 设置目标约束
        constraints = Constraints()
        constraints.name = "move_to_target"
        
        # 位置约束
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = target_pose.header.frame_id
        pos_constraint.link_name = "tool_0"
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0
        
        # 设置位置约束区域（球形）
        from shape_msgs.msg import SolidPrimitive
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]  # 1cm 精度
        
        pos_constraint.constraint_region.primitives.append(sphere)
        pos_constraint.constraint_region.primitive_poses.append(target_pose.pose)
        pos_constraint.weight = 1.0
        
        constraints.position_constraints.append(pos_constraint)
        
        # 姿态约束
        orient_constraint = OrientationConstraint()
        orient_constraint.header.frame_id = target_pose.header.frame_id
        orient_constraint.link_name = "tool_0"
        orient_constraint.orientation = target_pose.pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 0.1  # ~6度
        orient_constraint.absolute_y_axis_tolerance = 0.1
        orient_constraint.absolute_z_axis_tolerance = 0.1
        orient_constraint.weight = 1.0
        
        constraints.orientation_constraints.append(orient_constraint)
        
        request.goal_constraints.append(constraints)
        
        # 设置工作空间
        workspace = WorkspaceParameters()
        workspace.header.frame_id = "base_link"
        workspace.min_corner.x = -1.0
        workspace.min_corner.y = -1.0
        workspace.min_corner.z = -0.1
        workspace.max_corner.x = 1.0
        workspace.max_corner.y = 1.0
        workspace.max_corner.z = 1.0
        request.workspace_parameters = workspace
        
        goal.request = request
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3
        
        # 发送目标
        self.get_logger().info("Sending goal to MoveIt...")
        future = self._move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by MoveIt")
            return False
        
        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
        
        result = result_future.result()
        if result.result.error_code.val == 1:  # SUCCESS
            self.get_logger().info("MoveIt motion completed successfully")
            return True
        else:
            self.get_logger().error(f"MoveIt motion failed with error code: {result.result.error_code.val}")
            return False
    
    def move_to_joints(self, joint_positions, planning_time=5.0):
        """
        使用 MoveIt 移动到目标关节角度
        
        Args:
            joint_positions: list[float] 目标关节角度
            planning_time: 规划时间限制（秒）
        
        Returns:
            bool: 是否成功
        """
        if not self._move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("MoveIt MoveGroup server not available")
            return False
        
        # 创建 MoveGroup goal
        goal = MoveGroup.Goal()
        
        # 设置规划请求
        request = MotionPlanRequest()
        request.group_name = "manipulator"
        request.num_planning_attempts = 10
        request.allowed_planning_time = planning_time
        request.max_velocity_scaling_factor = 0.5
        request.max_acceleration_scaling_factor = 0.5
        
        # 设置关节目标约束
        constraints = Constraints()
        constraints.name = "move_to_joints"
        
        for i, (name, pos) in enumerate(zip(JOINT_NAMES, joint_positions)):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = name
            joint_constraint.position = pos
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        
        request.goal_constraints.append(constraints)
        
        # 设置工作空间
        workspace = WorkspaceParameters()
        workspace.header.frame_id = "base_link"
        workspace.min_corner.x = -1.0
        workspace.min_corner.y = -1.0
        workspace.min_corner.z = -0.1
        workspace.max_corner.x = 1.0
        workspace.max_corner.y = 1.0
        workspace.max_corner.z = 1.0
        request.workspace_parameters = workspace
        
        goal.request = request
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3
        
        # 发送目标
        self.get_logger().info("Sending joint goal to MoveIt...")
        future = self._move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Joint goal rejected by MoveIt")
            return False
        
        self.get_logger().info("Joint goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
        
        result = result_future.result()
        if result.result.error_code.val == 1:  # SUCCESS
            self.get_logger().info("MoveIt joint motion completed successfully")
            return True
        else:
            self.get_logger().error(f"MoveIt joint motion failed with error code: {result.result.error_code.val}")
            return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--result-file", default="real_eye_in_hand_result.yaml")
    parser.add_argument("--tag-id", type=int, default=0)
    parser.add_argument("--tag-size", type=float, default=0.057)
    args = parser.parse_args()

    T_cam_ee = load_calibration(args.result_file)

    # Camera
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    time.sleep(2)

    profile = pipeline.get_active_profile()
    intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    K = np.array([[intrinsics.fx, 0, intrinsics.ppx],
                   [0, intrinsics.fy, intrinsics.ppy],
                   [0, 0, 1]], dtype=np.float64)
    D = np.array(intrinsics.coeffs, dtype=np.float64)

    dictionary = cv2.aruco.getPredefinedDictionary(APRILTAG_DICT)
    params = cv2.aruco.DetectorParameters_create()

    # ROS2
    rclpy.init()
    node = MoveItClient()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    time.sleep(2)

    q = node.get_current_q()
    if q is not None:
        print(f"Current joints: [{', '.join(f'{j:.3f}' for j in q)}]")

    ee_pose = node.get_ee_pose()
    if ee_pose is not None:
        p = ee_pose.pose.position
        print(f"Current EE: [{p.x:.4f}, {p.y:.4f}, {p.z:.4f}]")

    print(f"\nAprilTag Verify (MoveIt) | ID={args.tag_id} Size={args.tag_size}m")
    print("Space = move arm | Q = quit\n")

    try:
        while True:
            frames = pipeline.wait_for_frames()
            cf = frames.get_color_frame()
            if not cf:
                continue
            image = np.asanyarray(cf.get_data())
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)

            vis = image.copy()
            T_cam_tag = None

            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(vis, corners, ids)
                for i, tid in enumerate(ids.flatten().tolist()):
                    half = args.tag_size / 2.0
                    obj = np.array([[-half,-half,0],[half,-half,0],[half,half,0],[-half,half,0]], dtype=np.float32)
                    ok, rvec, tvec = cv2.solvePnP(obj, corners[i].reshape(4,2).astype(np.float32), K, D)
                    if ok:
                        cv2.drawFrameAxes(vis, K, D, rvec, tvec, 0.05)
                        cv2.putText(vis, f"ID={tid} {np.linalg.norm(tvec):.3f}m", (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        if tid == args.tag_id:
                            R_m, _ = cv2.Rodrigues(rvec)
                            T_cam_tag = np.eye(4)
                            T_cam_tag[:3, :3] = R_m
                            T_cam_tag[:3, 3] = tvec.flatten()
                            cv2.putText(vis, "[SPACE] Move", (10, 60),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            if T_cam_tag is None:
                cv2.putText(vis, f"Looking for ID={args.tag_id}...", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            cv2.imshow("Calibration Verify (MoveIt)", vis)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            elif key == ord(' ') and T_cam_tag is not None:
                print("\nMoving with MoveIt...")
                
                # 计算目标位姿
                ee_pose = node.get_ee_pose()
                if ee_pose is None:
                    print("ERROR: No EE pose available")
                    continue
                
                # 获取当前末端位姿的变换矩阵
                T_ee = np.eye(4)
                T_ee[:3, 3] = [ee_pose.pose.position.x, 
                               ee_pose.pose.position.y, 
                               ee_pose.pose.position.z]
                q_ee = ee_pose.pose.orientation
                # 四元数转旋转矩阵
                x, y, z, w = q_ee.x, q_ee.y, q_ee.z, q_ee.w
                T_ee[:3, :3] = np.array([
                    [1-2*y*y-2*z*z, 2*x*y-2*w*z, 2*x*z+2*w*y],
                    [2*x*y+2*w*z, 1-2*x*x-2*z*z, 2*y*z-2*w*x],
                    [2*x*z-2*w*y, 2*y*z+2*w*x, 1-2*x*x-2*y*y]])
                
                T_ee_cam = np.linalg.inv(T_cam_ee)
                
                # 计算目标位姿（在标定板上方 10cm）
                T_base_tag = T_ee @ T_ee_cam @ T_cam_tag
                target_pos = T_base_tag[:3, 3].copy()
                target_pos[2] += 0.10  # 上方 10cm
                
                if target_pos[2] < 0.08:
                    target_pos[2] = 0.08  # 最小高度 8cm
                
                print(f"Tag position: [{T_base_tag[0,3]:.4f}, {T_base_tag[1,3]:.4f}, {T_base_tag[2,3]:.4f}]")
                print(f"Target (above): [{target_pos[0]:.4f}, {target_pos[1]:.4f}, {target_pos[2]:.4f}]")
                
                # 创建目标位姿
                target_pose = PoseStamped()
                target_pose.header.frame_id = "base_link"
                target_pose.header.stamp = node.get_clock().now().to_msg()
                target_pose.pose.position.x = target_pos[0]
                target_pose.pose.position.y = target_pos[1]
                target_pose.pose.position.z = target_pos[2]
                
                # 保持当前姿态（末端朝下）
                target_pose.pose.orientation = ee_pose.pose.orientation
                
                # 使用 MoveIt 移动
                success = node.move_to_pose(target_pose)
                
                if success:
                    time.sleep(1)
                    # 报告误差
                    actual_pose = node.get_ee_pose()
                    if actual_pose is not None:
                        actual = np.array([actual_pose.pose.position.x,
                                          actual_pose.pose.position.y,
                                          actual_pose.pose.position.z])
                        err = np.linalg.norm(actual - target_pos)
                        print(f"Actual EE: [{actual[0]:.4f}, {actual[1]:.4f}, {actual[2]:.4f}]")
                        print(f"Error: {err*1000:.1f} mm")
                else:
                    print("MoveIt motion failed!")
                
                print("Done.\n")
            time.sleep(0.01)
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
