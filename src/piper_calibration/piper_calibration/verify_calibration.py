"""
标定精度验证：AprilTag 检测 + MuJoCo IK + 轨迹控制器。
不需要 MoveIt。

用法：
  终端 1: ros2 launch piper_control real_bringup.launch.py can:=can0
  终端 2: python3 src/piper_calibration/piper_calibration/verify_calibration.py \
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
import mujoco
from scipy.optimize import minimize

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import tf2_ros

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
APRILTAG_DICT = cv2.aruco.DICT_APRILTAG_36h11

# MuJoCo model path
MUJOCO_MODEL = os.path.expanduser("~/桌面/PiperSim/models/scene.xml")


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


class MuJoCoIK:
    def __init__(self, model_path):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.ee_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "link6")

        # Fix joint5 axis: MuJoCo model has opposite direction to real arm
        j5_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "joint5")
        self.model.jnt_axis[j5_id] = [0, 0, -1]  # flip Z axis
        self.model.jnt_range[j5_id] = [-1.22, 1.22]  # limits stay the same (symmetric)

        self.joint_qpos = []
        self.joint_limits = []
        for name in JOINT_NAMES:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            self.joint_qpos.append(self.model.jnt_qposadr[jid])
            lo = self.model.jnt_range[jid, 0]
            hi = self.model.jnt_range[jid, 1]
            self.joint_limits.append((lo, hi))
        print(f"MuJoCo IK ready: {len(self.joint_qpos)} joints, EE body={self.ee_id}")

    def fk(self, q):
        for i, adr in enumerate(self.joint_qpos):
            self.data.qpos[adr] = q[i]
        mujoco.mj_forward(self.model, self.data)
        return self.data.xpos[self.ee_id].copy()

    def solve(self, target_pos, q_init, retries=5):
        bounds = self.joint_limits
        best_q = None
        best_err = float('inf')

        for attempt in range(retries):
            q0 = q_init + np.random.randn(6) * 0.1 if attempt > 0 else q_init.copy()
            q0 = np.clip(q0, [b[0] for b in bounds], [b[1] for b in bounds])

            def cost(q):
                pos = self.fk(q)
                return np.sum((pos - target_pos) ** 2)

            res = minimize(cost, q0, method='L-BFGS-B', bounds=bounds,
                           options={'maxiter': 300, 'ftol': 1e-12})
            if res.fun < best_err:
                best_err = res.fun
                best_q = res.x

            if best_err < 1e-8:
                break

        if best_err < 0.005 ** 2:  # < 5mm
            return best_q
        return None


class VerifyNode(Node):
    def __init__(self, T_cam_ee, ik_solver):
        super().__init__("verify_calibration")
        self._T_cam_ee = T_cam_ee
        self._T_ee_cam = np.linalg.inv(T_cam_ee)
        self._ik = ik_solver
        self._current_joints = None

        self.create_subscription(JointState, "/joint_states", self._joint_cb, 10)
        self._action_client = ActionClient(
            self, FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory")

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

    def _joint_cb(self, msg):
        d = {}
        for name, pos in zip(msg.name, msg.position):
            d[name] = pos
        self._current_joints = d

    def get_current_q(self):
        if self._current_joints is None:
            return None
        return np.array([self._current_joints.get(n, 0.0) for n in JOINT_NAMES])

    def get_ee_pose(self):
        try:
            t = self._tf_buffer.lookup_transform("base_link", "link6",
                                                  rclpy.time.Time(),
                                                  timeout=rclpy.duration.Duration(seconds=1.0))
            T = np.eye(4)
            T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
            q = t.transform.rotation
            x, y, z, w = q.x, q.y, q.z, q.w
            T[:3, :3] = np.array([
                [1-2*y*y-2*z*z, 2*x*y-2*w*z, 2*x*z+2*w*y],
                [2*x*y+2*w*z, 1-2*x*x-2*z*z, 2*y*z-2*w*x],
                [2*x*z-2*w*y, 2*y*z+2*w*x, 1-2*x*x-2*y*y]])
            return T
        except Exception:
            return None

    def send_trajectory(self, q_target, sec=3):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("No trajectory server")
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = [float(q) for q in q_target]
        pt.time_from_start = Duration(sec=int(sec), nanosec=int((sec % 1) * 1e9))
        goal.trajectory.points = [pt]
        future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        h = future.result()
        if not h.accepted:
            return False
        r = h.get_result_async()
        rclpy.spin_until_future_complete(self, r, timeout_sec=sec + 5.0)
        return True

    def move_to_tag(self, T_cam_tag):
        T_ee = self.get_ee_pose()
        if T_ee is None:
            self.get_logger().error("No EE pose")
            return False

        T_base_tag = T_ee @ self._T_ee_cam @ T_cam_tag
        target = T_base_tag[:3, 3]

        if np.linalg.norm(target) > 2.0:
            self.get_logger().error(f"Too far: {target}")
            return False

        # Safety: always move ABOVE the tag, never directly to it
        # This prevents the arm from crashing into the table
        safe_target = target.copy()
        safe_target[2] += 0.10  # 10cm above tag
        if safe_target[2] < 0.08:
            safe_target[2] = 0.08  # minimum height 8cm

        self.get_logger().info(f"Tag pos:    [{target[0]:.4f}, {target[1]:.4f}, {target[2]:.4f}]")
        self.get_logger().info(f"Safe target:[{safe_target[0]:.4f}, {safe_target[1]:.4f}, {safe_target[2]:.4f}]")

        q_current = self.get_current_q()
        if q_current is None:
            self.get_logger().error("No joint states")
            return False

        q_target = self._ik.solve(safe_target, q_current)
        if q_target is None:
            self.get_logger().warn("IK failed, trying higher")
            safe_target[2] += 0.05
            q_target = self._ik.solve(safe_target, q_current)
            if q_target is None:
                self.get_logger().error("IK failed completely")
                return False

        self.get_logger().info(f"IK: [{', '.join(f'{q:.3f}' for q in q_target)}]")
        self.send_trajectory(q_target, sec=3)
        time.sleep(2)

        # Report error
        T_actual = self.get_ee_pose()
        if T_actual is not None:
            actual = T_actual[:3, 3]
            # Compare actual EE position to where we wanted to be (safe_target)
            err_to_safe = np.linalg.norm(actual - safe_target)
            # Compare actual EE position to tag position
            err_to_tag = np.linalg.norm(actual - target)
            self.get_logger().info(f"Safe target:[{safe_target[0]:.4f}, {safe_target[1]:.4f}, {safe_target[2]:.4f}]")
            self.get_logger().info(f"Actual EE:  [{actual[0]:.4f}, {actual[1]:.4f}, {actual[2]:.4f}]")
            self.get_logger().info(f"Error to safe point: {err_to_safe*1000:.1f} mm")
            self.get_logger().info(f"Error to tag:        {err_to_tag*1000:.1f} mm")
        return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--result-file", default="real_eye_in_hand_result.yaml")
    parser.add_argument("--tag-id", type=int, default=0)
    parser.add_argument("--tag-size", type=float, default=0.057)
    parser.add_argument("--model", default=MUJOCO_MODEL)
    args = parser.parse_args()

    T_cam_ee = load_calibration(args.result_file)

    # Init MuJoCo IK
    print(f"Loading MuJoCo model: {args.model}")
    ik = MuJoCoIK(args.model)

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
    node = VerifyNode(T_cam_ee, ik)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    time.sleep(2)

    q = node.get_current_q()
    if q is not None:
        print(f"Current joints: [{', '.join(f'{j:.3f}' for j in q)}]")
        fk_pos = ik.fk(q)
        print(f"FK check: [{fk_pos[0]:.4f}, {fk_pos[1]:.4f}, {fk_pos[2]:.4f}]")

    print(f"\nAprilTag Verify | ID={args.tag_id} Size={args.tag_size}m")
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

            cv2.imshow("Calibration Verify", vis)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            elif key == ord(' ') and T_cam_tag is not None:
                print("\nMoving...")
                node.move_to_tag(T_cam_tag)
                print("Done.\n")
            time.sleep(0.01)
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
