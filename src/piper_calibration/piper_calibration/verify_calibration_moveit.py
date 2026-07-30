"""Verify the real eye-to-hand calibration result with AprilTag and MoveIt.

The calibration result is T_base_camera. Therefore a detected tag pose T_camera_tag
is projected into the robot base by:

    T_base_tag = T_base_camera @ T_camera_tag

Run this script only with the normal real-robot MoveIt/control launch. Do not use
calibration_mode:=true while asking MoveIt to move the robot.
"""

import argparse
import os
from pathlib import Path
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
import tf2_ros
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    MotionPlanRequest,
    OrientationConstraint,
    PositionConstraint,
    WorkspaceParameters,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from shape_msgs.msg import SolidPrimitive


APRILTAG_DICT = cv2.aruco.DICT_APRILTAG_36h11


def _resolve_result_path(path: str) -> Path:
    candidate = Path(path).expanduser()
    if candidate.is_file():
        return candidate.resolve()
    if not candidate.is_absolute():
        workspace_file = Path(__file__).resolve().parents[3] / candidate
        if workspace_file.is_file():
            return workspace_file.resolve()
    raise FileNotFoundError(
        f"Calibration result not found: {path}. Run eye-to-hand calibration first "
        "and use data/real_eye_to_hand_result.yaml. The _samples.yaml file only "
        "contains raw observations."
    )


def load_calibration(path: str):
    result_path = _resolve_result_path(path)
    with result_path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if isinstance(data, list):
        raise ValueError(
            f"{result_path} is a legacy raw-samples YAML list. Its board_position "
            "values are target positions in the camera frame; it does not contain "
            "the base_link-to-camera transform required for verification."
        )
    if not isinstance(data, dict):
        raise ValueError(f"{result_path} must contain a YAML mapping")
    if "samples" in data and (
        "translation" not in data or "rotation_matrix" not in data
    ):
        raise ValueError(
            f"{result_path} is a raw-samples YAML. Use the final result_file "
            "referenced by that file instead."
        )
    if "translation" not in data or "rotation_matrix" not in data:
        raise ValueError(
            f"{result_path} has no translation/rotation_matrix fields. "
            "Pass the final real_eye_to_hand_result.yaml file."
        )
    if data.get("calibration_type") not in (None, "eye_to_hand"):
        raise ValueError(
            f"Unsupported calibration_type={data.get('calibration_type')!r}"
        )

    rotation = np.asarray(data["rotation_matrix"], dtype=float)
    translation = data["translation"]
    if rotation.shape != (3, 3):
        raise ValueError("rotation_matrix must be a 3x3 matrix")
    if not np.isfinite(rotation).all():
        raise ValueError("rotation_matrix contains NaN or infinity")
    if not isinstance(translation, dict):
        raise ValueError("translation must be a mapping with x, y and z")
    if not all(axis in translation for axis in ("x", "y", "z")):
        raise ValueError("translation must contain x, y and z")
    if translation.get("unit", "m") != "m":
        raise ValueError("translation.unit must be 'm'")

    position = np.asarray(
        [float(translation[axis]) for axis in ("x", "y", "z")], dtype=float
    )
    if not np.isfinite(position).all():
        raise ValueError("translation contains NaN or infinity")
    orthogonality_error = np.linalg.norm(rotation.T @ rotation - np.eye(3), "fro")
    determinant = float(np.linalg.det(rotation))
    if orthogonality_error > 1e-3 or determinant <= 0.0:
        raise ValueError(
            "rotation_matrix is not a valid right-handed rotation "
            f"(orthogonality error={orthogonality_error:.3g}, det={determinant:.6g})"
        )

    T_base_camera = np.eye(4)
    T_base_camera[:3, :3] = rotation
    T_base_camera[:3, 3] = position

    if "matrix_4x4" in data:
        stored_matrix = np.asarray(data["matrix_4x4"], dtype=float)
        if stored_matrix.shape != (4, 4):
            raise ValueError("matrix_4x4 must be a 4x4 matrix")
        if not np.allclose(stored_matrix, T_base_camera, atol=1e-6):
            raise ValueError(
                "matrix_4x4 is inconsistent with translation/rotation_matrix"
            )
    return T_base_camera, data, result_path


class VerificationNode(Node):
    def __init__(self, camera_topic, camera_info_topic,
                 base_frame, ee_link, move_group, camera_timeout,
                 expected_camera_frame=None):
        super().__init__("verify_eye_to_hand_calibration")
        self.base_frame = base_frame
        self.ee_link = ee_link
        self.move_group = move_group
        self.camera_timeout = camera_timeout
        self.expected_camera_frame = expected_camera_frame
        self._bridge = CvBridge()
        self._image = None
        self._K = None
        self._D = None
        self._camera_frame = None
        self._camera_error = None
        self._image_received_monotonic = None
        self._lock = threading.Lock()

        self.create_subscription(
            Image, camera_topic, self._image_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo, camera_info_topic, self._camera_info_callback,
            qos_profile_sensor_data
        )
        self._move_group_client = ActionClient(self, MoveGroup, "/move_action")
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

    def _image_callback(self, msg):
        try:
            image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Image conversion failed: {exc}")
            return
        with self._lock:
            self._image = image
            self._image_received_monotonic = time.monotonic()

    def _camera_info_callback(self, msg):
        with self._lock:
            if self._K is None:
                message_frame = msg.header.frame_id.strip()
                if not message_frame:
                    self._camera_error = "CameraInfo.header.frame_id is empty"
                    return
                if (
                    self.expected_camera_frame
                    and message_frame != self.expected_camera_frame
                ):
                    self._camera_error = (
                        f"Calibration result is for camera frame "
                        f"{self.expected_camera_frame!r}, but CameraInfo uses "
                        f"{message_frame!r}"
                    )
                    return
                self._camera_frame = message_frame
                self._K = np.asarray(msg.k, dtype=float).reshape(3, 3)
                self._D = np.asarray(msg.d, dtype=float)

    def wait_for_camera(self, timeout=30.0):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            with self._lock:
                ready = (
                    self._image is not None
                    and self._K is not None
                    and self._image_received_monotonic is not None
                    and time.monotonic() - self._image_received_monotonic
                    <= self.camera_timeout
                )
            if ready:
                return True
            time.sleep(0.1)
        return False

    def camera_snapshot(self):
        with self._lock:
            if self._image is None or self._K is None:
                return None
            return self._image.copy(), self._K.copy(), self._D.copy()

    def camera_is_fresh(self):
        with self._lock:
            return (
                self._image_received_monotonic is not None
                and time.monotonic() - self._image_received_monotonic
                <= self.camera_timeout
            )

    def camera_error(self):
        with self._lock:
            return self._camera_error

    def get_ee_pose(self):
        try:
            transform = self._tf_buffer.lookup_transform(
                self.base_frame, self.ee_link, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except Exception as exc:
            self.get_logger().error(
                f"TF {self.base_frame} -> {self.ee_link} unavailable: {exc}"
            )
            return None
        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    @staticmethod
    def _wait_future(future, timeout):
        # The node is already owned by the background executor. Calling
        # spin_until_future_complete here would add it to a second executor.
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            return None
        return future.result()

    def move_to_pose(self, target_pose, execute=False, planning_time=8.0):
        if not self._move_group_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                "MoveIt /move_action is unavailable; start normal real mode "
                "and do not use calibration_mode:=true for verification."
            )
            return False

        request = MotionPlanRequest()
        request.group_name = self.move_group
        request.num_planning_attempts = 10
        request.allowed_planning_time = planning_time
        request.max_velocity_scaling_factor = 0.2
        request.max_acceleration_scaling_factor = 0.2

        constraints = Constraints()
        position = PositionConstraint()
        position.header.frame_id = target_pose.header.frame_id
        position.link_name = self.ee_link
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]
        position.constraint_region.primitives.append(sphere)
        position.constraint_region.primitive_poses.append(target_pose.pose)
        position.weight = 1.0
        constraints.position_constraints.append(position)

        orientation = OrientationConstraint()
        orientation.header.frame_id = target_pose.header.frame_id
        orientation.link_name = self.ee_link
        orientation.orientation = target_pose.pose.orientation
        orientation.absolute_x_axis_tolerance = 0.15
        orientation.absolute_y_axis_tolerance = 0.15
        orientation.absolute_z_axis_tolerance = 0.15
        orientation.weight = 1.0
        constraints.orientation_constraints.append(orientation)
        request.goal_constraints.append(constraints)

        workspace = WorkspaceParameters()
        workspace.header.frame_id = self.base_frame
        workspace.min_corner.x = -1.0
        workspace.min_corner.y = -1.0
        workspace.min_corner.z = -0.1
        workspace.max_corner.x = 1.0
        workspace.max_corner.y = 1.0
        workspace.max_corner.z = 1.2
        request.workspace_parameters = workspace

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = not execute
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        goal_handle = self._wait_future(
            self._move_group_client.send_goal_async(goal), 30.0
        )
        if goal_handle is None:
            self.get_logger().error("Timed out sending MoveIt goal")
            return False
        if not goal_handle.accepted:
            self.get_logger().error("MoveIt rejected the goal")
            return False

        wrapped_result = self._wait_future(goal_handle.get_result_async(), 90.0)
        if wrapped_result is None:
            self.get_logger().error(
                "Timed out waiting for MoveIt result; requesting goal cancellation"
            )
            cancel_result = self._wait_future(
                goal_handle.cancel_goal_async(), 5.0
            )
            if cancel_result is None:
                self.get_logger().error(
                    "MoveIt did not acknowledge cancellation; use the physical "
                    "emergency stop if the robot is still moving"
                )
            return False
        code = wrapped_result.result.error_code.val
        if code != 1:
            self.get_logger().error(f"MoveIt failed with error code {code}")
            return False
        if execute:
            self.get_logger().info("MoveIt verification motion completed")
        else:
            self.get_logger().info(
                "MoveIt verification plan succeeded; no command was sent to "
                "the robot. Re-run with --execute only after reviewing the plan."
            )
        return True


def _make_detector():
    dictionary = cv2.aruco.getPredefinedDictionary(APRILTAG_DICT)
    params = (
        cv2.aruco.DetectorParameters()
        if hasattr(cv2.aruco, "DetectorParameters")
        else cv2.aruco.DetectorParameters_create()
    )
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, params)
        return detector.detectMarkers
    return lambda gray: cv2.aruco.detectMarkers(
        gray, dictionary, parameters=params
    )


def _detect_tag(image, K, D, tag_id, tag_size, detect_markers):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detect_markers(gray)
    if ids is None:
        return None, corners, ids
    matches = np.flatnonzero(ids.reshape(-1).astype(int) == tag_id)
    if not len(matches):
        return None, corners, ids

    half = tag_size / 2.0
    obj = np.array([
        [-half, -half, 0], [half, -half, 0],
        [half, half, 0], [-half, half, 0]
    ], dtype=np.float32)
    img = corners[int(matches[0])].reshape(4, 2).astype(np.float32)
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, D)
    if not ok:
        return None, corners, ids
    T_camera_tag = np.eye(4)
    T_camera_tag[:3, :3] = cv2.Rodrigues(rvec)[0]
    T_camera_tag[:3, 3] = tvec.reshape(3)
    return T_camera_tag, corners, ids


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--result-file", default="data/real_eye_to_hand_result.yaml")
    parser.add_argument("--tag-id", type=int, default=0)
    parser.add_argument("--tag-size", type=float, default=0.057)
    parser.add_argument("--standoff", type=float, default=0.10)
    parser.add_argument("--camera-topic", default="/camera/color/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera/color/camera_info")
    parser.add_argument("--camera-timeout", type=float, default=1.0)
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--ee-link", default="tool_0")
    parser.add_argument("--move-group", default="manipulator")
    parser.add_argument(
        "--execute",
        action="store_true",
        help=(
            "allow SPACE to execute the MoveIt trajectory on the real robot; "
            "without this flag the tool only plans"
        ),
    )
    parser.add_argument(
        "--inspect-only",
        action="store_true",
        help="validate and print the result YAML without connecting to ROS or moving",
    )
    parsed = parser.parse_args(args)
    if parsed.tag_size <= 0.0:
        parser.error("--tag-size must be greater than zero")
    if parsed.standoff <= 0.0:
        parser.error("--standoff must be greater than zero")
    if parsed.camera_timeout <= 0.0:
        parser.error("--camera-timeout must be greater than zero")

    try:
        T_base_camera, info, result_path = load_calibration(parsed.result_file)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.error(str(exc))

    p = T_base_camera[:3, 3]
    residual = info.get("motion_residual", info.get("error"))
    print(f"Loaded: {result_path}")
    print(f"Camera origin in base frame: [{p[0]:.6f}, {p[1]:.6f}, {p[2]:.6f}] m")
    transform_info = info.get("transform")
    calibration_camera_frame = None
    if isinstance(transform_info, dict):
        parent = transform_info.get("parent_frame")
        child = transform_info.get("child_frame")
        calibration_camera_frame = child
        print(f"Transform: {parent} -> {child}")
        if parent and parent != parsed.base_frame:
            parser.error(
                f"result parent_frame={parent!r} does not match "
                f"--base-frame={parsed.base_frame!r}"
            )
    if residual is not None:
        print(f"Calibration residual: {float(residual):.8f}")
    if parsed.inspect_only:
        print("Result schema and transform are valid.")
        return 0
    if (
        sys.platform.startswith("linux")
        and not os.environ.get("DISPLAY")
        and not os.environ.get("WAYLAND_DISPLAY")
    ):
        print(
            "[FAIL] 交互验证需要图形会话（DISPLAY 或 WAYLAND_DISPLAY）",
            file=sys.stderr,
        )
        print(
            "       修复: Docker 使用 ./docker/run.sh up --gui；"
            "无界面主机仅使用 --inspect-only",
            file=sys.stderr,
        )
        return 1

    rclpy.init(args=None)
    node = VerificationNode(
        parsed.camera_topic, parsed.camera_info_topic,
        parsed.base_frame, parsed.ee_link, parsed.move_group,
        parsed.camera_timeout,
        calibration_camera_frame,
    )
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    detect_markers = _make_detector()
    action = "move" if parsed.execute else "plan"
    window = f"Eye-to-Hand Verify [Space={action}, Q=quit]"

    status = 0
    try:
        if not node.wait_for_camera():
            raise RuntimeError(
                node.camera_error() or "Timed out waiting for ROS camera topics"
            )
        if parsed.execute:
            print(
                "EXECUTION ENABLED: SPACE plans and moves the real robot; "
                "keep the physical emergency stop ready."
            )
        else:
            print(
                "PLAN-ONLY: SPACE checks a trajectory without moving the robot. "
                "Use --execute only after reviewing the plan."
            )
        print("Q/Esc: quit")
        cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)
        while rclpy.ok():
            if not node.camera_is_fresh():
                raise RuntimeError(
                    "Camera stream became stale; verification stopped before "
                    "planning or execution."
                )
            snapshot = node.camera_snapshot()
            if snapshot is None:
                time.sleep(0.02)
                continue
            image, K, D = snapshot
            T_camera_tag, corners, ids = _detect_tag(
                image, K, D, parsed.tag_id, parsed.tag_size, detect_markers
            )
            display = image.copy()
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(display, corners, ids)

            T_base_tag = None
            if T_camera_tag is not None:
                T_base_tag = T_base_camera @ T_camera_tag
                p = T_base_tag[:3, 3]
                text = f"base tag: {p[0]:+.3f} {p[1]:+.3f} {p[2]:+.3f} m"
                cv2.putText(display, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 255, 0), 2)
            else:
                cv2.putText(display, f"Looking for tag ID={parsed.tag_id}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 0, 255), 2)

            cv2.imshow(window, display)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
            if key == ord(" ") and T_base_tag is not None:
                current = node.get_ee_pose()
                if current is None:
                    continue
                target = PoseStamped()
                target.header.frame_id = parsed.base_frame
                target.header.stamp = node.get_clock().now().to_msg()
                target.pose.position.x = float(T_base_tag[0, 3])
                target.pose.position.y = float(T_base_tag[1, 3])
                target.pose.position.z = max(
                    0.08, float(T_base_tag[2, 3] + parsed.standoff)
                )
                target.pose.orientation = current.pose.orientation
                if not node.move_to_pose(target, execute=parsed.execute):
                    status = 1
                    break
            time.sleep(0.01)
    except (RuntimeError, cv2.error) as exc:
        node.get_logger().error(str(exc))
        status = 1
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
    return status


if __name__ == "__main__":
    raise SystemExit(main())
