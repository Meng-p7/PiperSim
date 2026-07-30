"""Eye-to-hand calibration solver for a fixed external camera."""

import os

import cv2
import numpy as np
import yaml
from rclpy.node import Node


_METHOD_MAP = {
    "tsai": cv2.CALIB_HAND_EYE_TSAI,
    "park": cv2.CALIB_HAND_EYE_PARK,
    "horaud": cv2.CALIB_HAND_EYE_HORAUD,
    "andreff": cv2.CALIB_HAND_EYE_ANDREFF,
    "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
}
_MIN_ROTATION_DEG = 5.0


def _require_hand_eye_api():
    if not hasattr(cv2, "calibrateHandEye"):
        raise RuntimeError(
            "This OpenCV build does not provide cv2.calibrateHandEye. "
            "Use the ROS/Ubuntu system Python and install python3-opencv; "
            "remove any user-site OpenCV wheel that shadows it."
        )


class EyeToHandCalibrator:
    """Solve ``T_base_camera`` from robot poses and target observations.

    For every sample the calibration target is rigidly attached to the robot:

        T_base_gripper * T_gripper_target
          = T_base_camera * T_camera_target

    OpenCV's hand-eye solver is used with only the robot pose inverted. The
    returned matrix is the camera pose expressed in ``base_frame``.
    """

    def __init__(self, node: Node, method: str = "park"):
        if method not in _METHOD_MAP:
            raise ValueError(f"method must be one of {list(_METHOD_MAP)}")
        _require_hand_eye_api()
        self.node = node
        self.method = method
        self.robot_poses: list[np.ndarray] = []
        self.camera_poses: list[np.ndarray] = []
        self.result: np.ndarray | None = None
        self.error: float | None = None

    def add_sample(self, T_base_gripper: np.ndarray, T_camera_target: np.ndarray):
        robot_pose = self._validated_transform(
            "T_base_gripper", T_base_gripper
        )
        camera_pose = self._validated_transform(
            "T_camera_target", T_camera_target
        )
        self.robot_poses.append(robot_pose)
        self.camera_poses.append(camera_pose)

    @staticmethod
    def _validated_transform(name: str, value: np.ndarray) -> np.ndarray:
        transform = np.asarray(value, dtype=float)
        if transform.shape != (4, 4):
            raise ValueError(f"{name} must be a 4x4 matrix")
        if not np.isfinite(transform).all():
            raise ValueError(f"{name} contains NaN or infinity")
        if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-6):
            raise ValueError(f"{name} has an invalid homogeneous bottom row")

        rotation = transform[:3, :3]
        orthogonality_error = np.linalg.norm(
            rotation.T @ rotation - np.eye(3), "fro"
        )
        determinant = float(np.linalg.det(rotation))
        if orthogonality_error > 1e-3 or not np.isclose(
            determinant, 1.0, atol=1e-3
        ):
            raise ValueError(
                f"{name} rotation is invalid "
                f"(orthogonality error={orthogonality_error:.3g}, "
                f"det={determinant:.6g})"
            )
        return transform.copy()

    @property
    def num_samples(self) -> int:
        return len(self.robot_poses)

    def calibrate(self) -> tuple[np.ndarray, float]:
        if self.num_samples < 3:
            raise ValueError(f"Need at least 3 samples, got {self.num_samples}")
        _require_hand_eye_api()
        self._validate_motion_diversity()

        T_gripper_base = [np.linalg.inv(T) for T in self.robot_poses]
        R_gripper2base = [T[:3, :3] for T in T_gripper_base]
        t_gripper2base = [T[:3, 3] for T in T_gripper_base]
        R_target2cam = [T[:3, :3] for T in self.camera_poses]
        t_target2cam = [T[:3, 3] for T in self.camera_poses]

        self._log_diagnostic()
        R_base_camera, t_base_camera = cv2.calibrateHandEye(
            R_gripper2base,
            t_gripper2base,
            R_target2cam,
            t_target2cam,
            method=_METHOD_MAP[self.method],
        )

        U, _, Vt = np.linalg.svd(R_base_camera)
        R_fixed = U @ Vt
        if np.linalg.det(R_fixed) < 0:
            Vt[-1, :] *= -1
            R_fixed = U @ Vt

        self.result = np.eye(4)
        self.result[:3, :3] = R_fixed
        self.result[:3, 3] = np.asarray(t_base_camera).reshape(3)
        if not np.isfinite(self.result).all():
            raise ValueError("Hand-eye solver returned NaN or infinity")
        self.error = self._compute_motion_residual(self.result)
        if not np.isfinite(self.error):
            raise ValueError("Calibration residual is NaN or infinity")
        self.node.get_logger().info(
            f"Eye-to-hand calibration done [{self.method}], "
            f"motion residual={self.error:.8f}"
        )
        return self.result.copy(), self.error

    @staticmethod
    def _maximum_relative_rotation_deg(poses: list[np.ndarray]) -> float:
        maximum = 0.0
        for i in range(len(poses) - 1):
            for j in range(i + 1, len(poses)):
                relative = np.linalg.inv(poses[i]) @ poses[j]
                angle = np.degrees(
                    np.linalg.norm(cv2.Rodrigues(relative[:3, :3])[0])
                )
                maximum = max(maximum, float(angle))
        return maximum

    def _validate_motion_diversity(self):
        robot_rotation = self._maximum_relative_rotation_deg(self.robot_poses)
        camera_rotation = self._maximum_relative_rotation_deg(self.camera_poses)
        if (
            robot_rotation < _MIN_ROTATION_DEG
            or camera_rotation < _MIN_ROTATION_DEG
        ):
            raise ValueError(
                "Calibration poses do not contain enough rotational diversity: "
                f"robot={robot_rotation:.2f}deg, "
                f"target={camera_rotation:.2f}deg; move the wrist through at "
                f"least {_MIN_ROTATION_DEG:.0f}deg between distinct poses"
            )

    def _relative_motion_pairs(self):
        for i in range(self.num_samples - 1):
            A = self.robot_poses[i + 1] @ np.linalg.inv(self.robot_poses[i])
            B = self.camera_poses[i + 1] @ np.linalg.inv(self.camera_poses[i])
            yield i, A, B

    def _log_diagnostic(self):
        self.node.get_logger().info("Relative rotation diagnostics:")
        for i, A, B in self._relative_motion_pairs():
            robot_angle = np.degrees(np.linalg.norm(cv2.Rodrigues(A[:3, :3])[0]))
            camera_angle = np.degrees(np.linalg.norm(cv2.Rodrigues(B[:3, :3])[0]))
            self.node.get_logger().info(
                f"  pair {i}->{i + 1}: robot={robot_angle:.2f}deg, "
                f"camera={camera_angle:.2f}deg"
            )

    def _compute_motion_residual(self, T_base_camera: np.ndarray) -> float:
        residuals = [
            np.linalg.norm(A @ T_base_camera - T_base_camera @ B, "fro")
            for _, A, B in self._relative_motion_pairs()
        ]
        return float(np.mean(residuals))

    def result_dict(self, parent_frame: str, child_frame: str) -> dict:
        if self.result is None:
            raise ValueError("Run calibrate() first")
        return {
            "schema_version": 2,
            "calibration_type": "eye_to_hand",
            "method": self.method,
            "num_samples": self.num_samples,
            "motion_residual": float(self.error),
            "transform": {
                "parent_frame": parent_frame,
                "child_frame": child_frame,
                "meaning": f"T_{parent_frame}_{child_frame}",
            },
            "translation": {
                "x": float(self.result[0, 3]),
                "y": float(self.result[1, 3]),
                "z": float(self.result[2, 3]),
                "unit": "m",
            },
            "rotation_matrix": self.result[:3, :3].tolist(),
            "matrix_4x4": self.result.tolist(),
        }

    def save_result(self, filepath: str, parent_frame: str, child_frame: str):
        os.makedirs(os.path.dirname(filepath) or ".", exist_ok=True)
        with open(filepath, "w", encoding="utf-8") as f:
            yaml.safe_dump(
                self.result_dict(parent_frame, child_frame),
                f,
                default_flow_style=False,
                sort_keys=False,
                allow_unicode=True,
            )
        self.node.get_logger().info(f"Result saved to {filepath}")
