import os
import yaml
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def _quaternion_from_matrix(T: np.ndarray):
    w = np.sqrt(max(0, 1 + T[0,0] + T[1,1] + T[2,2])) / 2
    x = np.sqrt(max(0, 1 + T[0,0] - T[1,1] - T[2,2])) / 2
    y = np.sqrt(max(0, 1 - T[0,0] + T[1,1] - T[2,2])) / 2
    z = np.sqrt(max(0, 1 - T[0,0] - T[1,1] + T[2,2])) / 2
    x = np.copysign(x, T[2,1] - T[1,2])
    y = np.copysign(y, T[0,2] - T[2,0])
    z = np.copysign(z, T[1,0] - T[0,1])
    return np.array([x, y, z, w])


_METHOD_MAP = {
    "tsai": cv2.CALIB_HAND_EYE_TSAI,
    "park": cv2.CALIB_HAND_EYE_PARK,
    "horaud": cv2.CALIB_HAND_EYE_HORAUD,
    "andreff": cv2.CALIB_HAND_EYE_ANDREFF,
    "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


class HandEyeCalibrator:
    def __init__(self, node: Node, method: str = "park", eye_mode: str = "eye_in_hand"):
        self.node = node
        self.method = method
        self.eye_mode = eye_mode
        self.robot_poses: list[np.ndarray] = []
        self.camera_poses: list[np.ndarray] = []
        self.result: np.ndarray | None = None

        if eye_mode not in ("eye_in_hand", "eye_to_hand"):
            raise ValueError("eye_mode must be 'eye_in_hand' or 'eye_to_hand'")
        if method not in _METHOD_MAP:
            raise ValueError(f"method must be one of {list(_METHOD_MAP.keys())}")

        self._tf_broadcaster = StaticTransformBroadcaster(node)

    def add_sample(self, robot_pose: np.ndarray, camera_pose: np.ndarray):
        self.robot_poses.append(robot_pose)
        self.camera_poses.append(camera_pose)

    def clear(self):
        self.robot_poses.clear()
        self.camera_poses.clear()

    @property
    def num_samples(self) -> int:
        return len(self.robot_poses)

    def calibrate(self) -> tuple[np.ndarray, float]:
        n = len(self.robot_poses)
        if n < 3:
            raise ValueError(f"Need at least 3 samples, got {n}")

        if self.eye_mode == "eye_in_hand":
            R_gripper2base = [p[:3, :3] for p in self.robot_poses]
            t_gripper2base = [p[:3, 3] for p in self.robot_poses]
            R_target2cam = [p[:3, :3] for p in self.camera_poses]
            t_target2cam = [p[:3, 3] for p in self.camera_poses]
        else:
            R_gripper2base = [p[:3, :3].T for p in self.robot_poses]
            t_gripper2base = [-p[:3, :3].T @ p[:3, 3] for p in self.robot_poses]
            R_target2cam = [p[:3, :3].T for p in self.camera_poses]
            t_target2cam = [-p[:3, :3].T @ p[:3, 3] for p in self.camera_poses]

        self._log_diagnostic()

        cv_method = _METHOD_MAP[self.method]
        R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
            R_gripper2base, t_gripper2base,
            R_target2cam, t_target2cam,
            method=cv_method,
        )

        U, S, Vt = np.linalg.svd(R_cam2gripper)
        R_fixed = U @ Vt
        if np.linalg.det(R_fixed) < 0:
            Vt[-1, :] *= -1
            R_fixed = U @ Vt

        T_result = np.eye(4)
        T_result[:3, :3] = R_fixed
        T_result[:3, 3] = t_cam2gripper.flatten()

        error = self._compute_error(T_result)
        self.result = T_result

        self.node.get_logger().info(
            f"Calibration done [{self.method}/{self.eye_mode}] error: {error:.8f}"
        )
        return T_result, error

    def _log_diagnostic(self):
        self.node.get_logger().info("Relative rotation diagnostics:")
        for i in range(len(self.robot_poses) - 1):
            A = np.linalg.inv(self.robot_poses[i]) @ self.robot_poses[i + 1]
            B = self.camera_poses[i] @ np.linalg.inv(self.camera_poses[i + 1])
            ra = cv2.Rodrigues(A[:3, :3])[0]
            rb = cv2.Rodrigues(B[:3, :3])[0]
            angle_a = np.degrees(np.linalg.norm(ra))
            angle_b = np.degrees(np.linalg.norm(rb))
            self.node.get_logger().info(
                f"  pair {i}->{i+1}: robot rot {angle_a:.2f}deg | camera rot {angle_b:.2f}deg"
            )

    def _compute_error(self, T_x: np.ndarray) -> float:
        X = T_x.copy()
        errors = []
        for i in range(len(self.robot_poses) - 1):
            if self.eye_mode == "eye_in_hand":
                A = np.linalg.inv(self.robot_poses[i]) @ self.robot_poses[i + 1]
                B = self.camera_poses[i] @ np.linalg.inv(self.camera_poses[i + 1])
            else:
                A = self.robot_poses[i] @ np.linalg.inv(self.robot_poses[i + 1])
                B = np.linalg.inv(self.camera_poses[i]) @ self.camera_poses[i + 1]
            residual = A @ X - X @ B
            errors.append(np.linalg.norm(residual, "fro"))
        return float(np.mean(errors))

    def publish_tf(self, parent_frame: str, child_frame: str):
        if self.result is None:
            raise ValueError("Run calibrate() first")
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = float(self.result[0, 3])
        t.transform.translation.y = float(self.result[1, 3])
        t.transform.translation.z = float(self.result[2, 3])
        q = _quaternion_from_matrix(self.result)
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])
        self._tf_broadcaster.sendTransform(t)
        self.node.get_logger().info(
            f"Published static TF: {parent_frame} -> {child_frame}"
        )

    def save_result(self, filepath: str):
        if self.result is None:
            raise ValueError("Run calibrate() first")
        data = {
            "method": self.method,
            "eye_mode": self.eye_mode,
            "num_samples": len(self.robot_poses),
            "error": float(self._compute_error(self.result)),
            "translation": {
                "x": float(self.result[0, 3]),
                "y": float(self.result[1, 3]),
                "z": float(self.result[2, 3]),
            },
            "rotation_matrix": self.result[:3, :3].tolist(),
        }
        os.makedirs(os.path.dirname(filepath) or ".", exist_ok=True)
        with open(filepath, "w") as f:
            yaml.dump(data, f, default_flow_style=False)
        self.node.get_logger().info(f"Result saved to {filepath}")

    def load_result(self, filepath: str) -> np.ndarray:
        with open(filepath, "r") as f:
            data = yaml.safe_load(f)
        T = np.eye(4)
        T[:3, :3] = np.array(data["rotation_matrix"])
        T[0, 3] = data["translation"]["x"]
        T[1, 3] = data["translation"]["y"]
        T[2, 3] = data["translation"]["z"]
        self.result = T
        self.node.get_logger().info(f"Result loaded from {filepath}")
        return T
