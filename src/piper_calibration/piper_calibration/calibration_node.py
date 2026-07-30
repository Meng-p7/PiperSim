"""Interactive real-robot eye-to-hand calibration node."""

import os
import sys
import threading
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import rclpy
import tf2_ros
import yaml
from cv_bridge import CvBridge
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from lifecycle_msgs.msg import State as LifecycleState
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from tf2_ros import TransformException

from .board_detector import BoardDetector
from .calibrator import EyeToHandCalibrator


JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]


class CalibrationNode(Node):
    def __init__(self):
        super().__init__("piper_calibration")

        self.declare_parameter("method", "park")
        self.declare_parameter("num_poses", 20)
        self.declare_parameter("end_effector_link", "tool_0")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_topic", "/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("camera_frame", "")
        self.declare_parameter("board_type", "aruco")
        self.declare_parameter("charuco_rows", 9)
        self.declare_parameter("charuco_cols", 14)
        self.declare_parameter("charuco_square_length", 0.02)
        self.declare_parameter("charuco_marker_length", 0.015)
        self.declare_parameter("chessboard_rows", 10)
        self.declare_parameter("chessboard_cols", 12)
        self.declare_parameter("aruco_dict", "DICT_APRILTAG_36H11")
        self.declare_parameter("aruco_marker_length", 0.057)
        self.declare_parameter("aruco_marker_id", 0)
        self.declare_parameter("result_file", "data/real_eye_to_hand_result.yaml")
        self.declare_parameter(
            "image_save_dir", "data/calibration_images_eye_to_hand"
        )
        self.declare_parameter("joint_state_timeout_sec", 1.0)
        self.declare_parameter("image_timeout_sec", 1.0)
        self.declare_parameter("safety_recheck_period_sec", 1.0)

        self.method = self.get_parameter("method").value
        self.num_poses = int(self.get_parameter("num_poses").value)
        self.ee_link = self.get_parameter("end_effector_link").value
        self.base_frame = self.get_parameter("base_frame").value
        self.camera_topic = self.get_parameter("camera_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.result_file = self.get_parameter("result_file").value
        self.image_save_dir = self.get_parameter("image_save_dir").value
        self.joint_state_timeout_sec = float(
            self.get_parameter("joint_state_timeout_sec").value
        )
        self.image_timeout_sec = float(
            self.get_parameter("image_timeout_sec").value
        )
        self.safety_recheck_period_sec = float(
            self.get_parameter("safety_recheck_period_sec").value
        )
        if (
            self.joint_state_timeout_sec <= 0.0
            or self.image_timeout_sec <= 0.0
            or self.safety_recheck_period_sec <= 0.0
        ):
            raise ValueError("Calibration freshness and recheck periods must be positive")

        self._bridge = CvBridge()
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._latest_image = None
        self._latest_image_time = None
        self._latest_image_received_monotonic = None
        self._camera_matrix = None
        self._dist_coeffs = None
        self._camera_frame_error_logged = False
        self._joint_positions: dict[str, float] = {}
        self._joint_state_received_monotonic = None
        self._robot_description = None
        self._image_lock = threading.Lock()
        self._joint_lock = threading.Lock()
        self._robot_description_lock = threading.Lock()
        self._attestation_limitation_logged = False
        self._controller_client = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self._hardware_client = self.create_client(
            ListHardwareComponents,
            "/controller_manager/list_hardware_components",
        )

        self._detector = BoardDetector(
            board_type=self.get_parameter("board_type").value,
            charuco_rows=self.get_parameter("charuco_rows").value,
            charuco_cols=self.get_parameter("charuco_cols").value,
            charuco_square_length=self.get_parameter("charuco_square_length").value,
            charuco_marker_length=self.get_parameter("charuco_marker_length").value,
            chessboard_rows=self.get_parameter("chessboard_rows").value,
            chessboard_cols=self.get_parameter("chessboard_cols").value,
            aruco_dict_name=self.get_parameter("aruco_dict").value,
            aruco_marker_length=self.get_parameter("aruco_marker_length").value,
            aruco_marker_id=self.get_parameter("aruco_marker_id").value,
        )
        self._calibrator = EyeToHandCalibrator(self, method=self.method)

        self.create_subscription(
            Image, self.camera_topic, self._image_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_states_callback,
            qos_profile_sensor_data,
        )
        robot_description_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            String,
            "/robot_description",
            self._robot_description_callback,
            robot_description_qos,
        )
        self.get_logger().info(
            f"Real eye-to-hand calibration: {self.base_frame} -> "
            f"{self.camera_frame or '<CameraInfo.header.frame_id>'}, "
            f"end effector={self.ee_link}"
        )

    def _image_callback(self, msg: Image):
        try:
            image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Image conversion failed: {exc}")
            return
        with self._image_lock:
            self._latest_image = image
            self._latest_image_time = rclpy.time.Time.from_msg(msg.header.stamp)
            self._latest_image_received_monotonic = time.monotonic()

    def _camera_info_callback(self, msg: CameraInfo):
        if self._camera_matrix is not None:
            return
        message_frame = msg.header.frame_id.strip()
        if not message_frame:
            if not self._camera_frame_error_logged:
                self.get_logger().error("CameraInfo.header.frame_id is empty")
                self._camera_frame_error_logged = True
            return
        if self.camera_frame and message_frame != self.camera_frame:
            if not self._camera_frame_error_logged:
                self.get_logger().error(
                    f"Configured camera_frame={self.camera_frame!r} does not match "
                    f"CameraInfo.header.frame_id={message_frame!r}. PnP poses use "
                    "the CameraInfo optical frame."
                )
                self._camera_frame_error_logged = True
            return
        if not self.camera_frame:
            self.camera_frame = message_frame
            self.get_logger().info(
                f"Using camera optical frame from CameraInfo: {self.camera_frame}"
            )
        self._camera_matrix = np.asarray(msg.k, dtype=float).reshape(3, 3)
        self._dist_coeffs = np.asarray(msg.d, dtype=float)
        self.get_logger().info("Camera intrinsics received")

    def _joint_states_callback(self, msg: JointState):
        positions = dict(zip(msg.name, msg.position))
        if (
            any(name not in positions for name in JOINT_NAMES)
            or any(not np.isfinite(positions[name]) for name in JOINT_NAMES)
        ):
            return
        with self._joint_lock:
            self._joint_positions.update(positions)
            self._joint_state_received_monotonic = time.monotonic()

    def _robot_description_callback(self, msg: String):
        with self._robot_description_lock:
            self._robot_description = msg.data

    def _get_current_joint_positions(self):
        with self._joint_lock:
            if (
                self._joint_state_received_monotonic is None
                or time.monotonic() - self._joint_state_received_monotonic
                > self.joint_state_timeout_sec
                or any(name not in self._joint_positions for name in JOINT_NAMES)
            ):
                return None
            return [float(self._joint_positions[name]) for name in JOINT_NAMES]

    def _image_is_fresh(self):
        with self._image_lock:
            return (
                self._latest_image_received_monotonic is not None
                and time.monotonic() - self._latest_image_received_monotonic
                <= self.image_timeout_sec
            )

    def _joint_state_is_fresh(self):
        with self._joint_lock:
            return (
                self._joint_state_received_monotonic is not None
                and time.monotonic() - self._joint_state_received_monotonic
                <= self.joint_state_timeout_sec
            )

    def _get_current_ee_pose(self, image_time=None, log_error=True):
        lookup_time = image_time if image_time is not None else rclpy.time.Time()
        try:
            transform = self._tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_link,
                lookup_time,
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except TransformException as exc:
            if log_error:
                self.get_logger().error(
                    f"TF {self.base_frame} -> {self.ee_link} unavailable: {exc}"
                )
            return None

        T = np.eye(4)
        T[:3, 3] = [
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ]
        q = transform.transform.rotation
        T[:3, :3] = self._quat_to_rot(q.x, q.y, q.z, q.w)
        return T

    @staticmethod
    def _quat_to_rot(x, y, z, w):
        norm = np.sqrt(x * x + y * y + z * z + w * w)
        if norm == 0:
            return np.eye(3)
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        return np.array([
            [1 - 2*y*y - 2*z*z, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
            [2*x*y + 2*w*z, 1 - 2*x*x - 2*z*z, 2*y*z - 2*w*x],
            [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x*x - 2*y*y],
        ])

    def _wait_for_inputs(self, timeout=30.0):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            with self._image_lock:
                camera_ready = (
                    self._latest_image is not None
                    and self._latest_image_time is not None
                    and self._latest_image_received_monotonic is not None
                    and time.monotonic() - self._latest_image_received_monotonic
                    <= self.image_timeout_sec
                    and self._camera_matrix is not None
                    and self._dist_coeffs is not None
                    and bool(self.camera_frame)
                )
                image_time = self._latest_image_time
            if camera_ready and self._get_current_joint_positions() is not None:
                if self._get_current_ee_pose(
                    image_time=image_time, log_error=False
                ) is not None:
                    return True
            time.sleep(0.2)
        return False

    def _wait_for_future(self, future, deadline, description):
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            self.get_logger().error(f"Timed out while {description}")
            return None
        try:
            return future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed while {description}: {exc}")
            return None

    def _verify_hardware_calibration_mode(self, timeout=10.0):
        deadline = time.monotonic() + timeout
        description = None
        while rclpy.ok() and time.monotonic() < deadline:
            publisher_count = self.count_publishers("/robot_description")
            if publisher_count > 1:
                self.get_logger().error(
                    "Calibration refused: /robot_description has multiple "
                    f"publishers ({publisher_count}); the controller input is ambiguous."
                )
                return False
            with self._robot_description_lock:
                description = self._robot_description
            if description and publisher_count == 1:
                break
            time.sleep(0.05)

        if not description:
            self.get_logger().error(
                "Calibration refused: no transient-local /robot_description was "
                "received. Start real_bringup.launch.py with calibration_mode:=true."
            )
            return False
        if self.count_publishers("/robot_description") != 1:
            self.get_logger().error(
                "Calibration refused: /robot_description must have exactly one "
                "live publisher."
            )
            return False

        try:
            root = ET.fromstring(description)
        except ET.ParseError as exc:
            self.get_logger().error(f"Invalid /robot_description XML: {exc}")
            return False

        modes = []
        for control in root.findall(".//ros2_control"):
            plugin = control.find("./hardware/plugin")
            if (
                plugin is None
                or (plugin.text or "").strip() != "piper_control/PiperHardware"
            ):
                continue
            mode = control.find("./hardware/param[@name='calibration_mode']")
            modes.append("" if mode is None else (mode.text or "").strip().lower())

        if modes != ["true"]:
            self.get_logger().error(
                "Calibration refused: PiperHardware was not loaded exactly once "
                "with calibration_mode=true. Restart real_bringup.launch.py "
                "with calibration_mode:=true."
            )
            return False

        if not self._attestation_limitation_logged:
            self.get_logger().warning(
                "Calibration hardware check uses the transient-local "
                "/robot_description input and a single-publisher guard. ROS 2 "
                "does not expose immutable runtime hardware parameters, so this "
                "cannot independently prove which description an already-running "
                "controller_manager consumed; do not remap or replace this topic "
                "during calibration."
            )
            self._attestation_limitation_logged = True
        return True

    def _verify_calibration_controllers(
        self, timeout=10.0, *, log_progress=True
    ):
        """Refuse sampling unless hardware and controllers are feedback-only."""
        if log_progress:
            self.get_logger().info(
                "Checking hardware mode and controller state before calibration..."
            )
        if not self._verify_hardware_calibration_mode(timeout=timeout):
            return False
        if not self._controller_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(
                "/controller_manager/list_controllers is unavailable. Start "
                "real_bringup.launch.py with calibration_mode:=true."
            )
            return False
        if not self._hardware_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(
                "/controller_manager/list_hardware_components is unavailable. "
                "Cannot verify that PiperHardware remains active."
            )
            return False

        hardware_future = self._hardware_client.call_async(
            ListHardwareComponents.Request()
        )
        hardware_response = self._wait_for_future(
            hardware_future,
            time.monotonic() + timeout,
            "checking PiperHardware state",
        )
        if hardware_response is None:
            return False

        piper_components = [
            component
            for component in hardware_response.component
            if (
                getattr(component, "plugin_name", "") or component.class_type
            )
            == "piper_control/PiperHardware"
        ]
        if len(piper_components) != 1:
            self.get_logger().error(
                "Calibration refused: expected exactly one loaded "
                "piper_control/PiperHardware component."
            )
            return False
        piper_component = piper_components[0]
        if (
            piper_component.type != "system"
            or piper_component.state.id != LifecycleState.PRIMARY_STATE_ACTIVE
        ):
            self.get_logger().error(
                "Calibration stopped: PiperHardware is not an active system "
                f"(type={piper_component.type!r}, "
                f"state={piper_component.state.label!r})."
            )
            return False

        future = self._controller_client.call_async(ListControllers.Request())
        response = self._wait_for_future(
            future,
            time.monotonic() + timeout,
            "checking active controllers",
        )
        if response is None:
            return False

        active = sorted(
            (
                controller
                for controller in response.controller
                if controller.state == "active"
            ),
            key=lambda controller: controller.name,
        )
        unexpected = [
            controller.name
            for controller in active
            if controller.name != "joint_state_broadcaster"
        ]
        broadcasters = [
            controller
            for controller in active
            if controller.name == "joint_state_broadcaster"
        ]
        if len(broadcasters) != 1:
            self.get_logger().error(
                "Exactly one active joint_state_broadcaster is required; "
                "calibration feedback cannot be trusted."
            )
            return False
        if unexpected:
            self.get_logger().error(
                "Calibration refused because motion controllers are active: "
                + ", ".join(unexpected)
                + ". Restart real_bringup.launch.py with calibration_mode:=true."
            )
            return False

        broadcaster = broadcasters[0]
        expected_type = "joint_state_broadcaster/JointStateBroadcaster"
        if broadcaster.type != expected_type:
            self.get_logger().error(
                "Calibration refused: active controller named "
                f"joint_state_broadcaster has type {broadcaster.type!r}, "
                f"expected {expected_type!r}."
            )
            return False
        if broadcaster.claimed_interfaces:
            self.get_logger().error(
                "Calibration refused: joint_state_broadcaster unexpectedly claims "
                "command interfaces: " + ", ".join(broadcaster.claimed_interfaces)
            )
            return False

        if log_progress:
            self.get_logger().info(
                "Controller safety check passed: typed feedback broadcaster only"
            )
        return True

    @staticmethod
    def _pose_record(meaning, T):
        return {
            "meaning": meaning,
            "translation_m": {
                "x": float(T[0, 3]),
                "y": float(T[1, 3]),
                "z": float(T[2, 3]),
            },
            "rotation_matrix": T[:3, :3].tolist(),
        }

    def _save_samples(self, samples):
        samples_file = os.path.splitext(self.result_file)[0] + "_samples.yaml"
        data = {
            "schema_version": 2,
            "calibration_type": "eye_to_hand",
            "description": (
                "Raw observations only. The final base-to-camera transform is "
                "stored in result_file, not in this samples file."
            ),
            "frames": {
                "base": self.base_frame,
                "end_effector": self.ee_link,
                "camera": self.camera_frame,
                "target": "calibration_target",
            },
            "result_file": self.result_file,
            "samples": samples,
        }
        os.makedirs(os.path.dirname(samples_file) or ".", exist_ok=True)
        with open(samples_file, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)
        self.get_logger().info(f"Samples saved to {samples_file}")

    def run(self):
        if sys.platform.startswith("linux") and not (
            os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
        ):
            self.get_logger().error(
                "Calibration requires an interactive OpenCV window, but neither "
                "DISPLAY nor WAYLAND_DISPLAY is set. Run from a graphical session "
                "or start the container with docker/run.sh --gui."
            )
            return False

        if not self._verify_calibration_controllers():
            return False
        self.get_logger().info(
            "Waiting for camera topics, /joint_states and robot TF..."
        )
        if not self._wait_for_inputs():
            self.get_logger().error(
                f"Input timeout; verify camera topics and TF "
                f"{self.base_frame} -> {self.ee_link}"
            )
            return False

        os.makedirs(self.image_save_dir, exist_ok=True)
        samples = []
        window = "Eye-to-Hand Calibration [Space=capture, Q=finish]"
        try:
            cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)
        except cv2.error as exc:
            self.get_logger().error(
                f"Unable to open the calibration window; check DISPLAY/X11: {exc}"
            )
            return False
        self.get_logger().info(
            "Use the robot teach button to drag the arm; press SPACE at each "
            "distinct pose while the target remains visible."
        )

        def sampling_safety_ok():
            if not self._joint_state_is_fresh():
                self.get_logger().error(
                    "Calibration stopped: /joint_states is stale "
                    f"(timeout={self.joint_state_timeout_sec:.3f}s)."
                )
                return False
            if not self._image_is_fresh():
                self.get_logger().error(
                    "Calibration stopped: camera images are stale "
                    f"(timeout={self.image_timeout_sec:.3f}s)."
                )
                return False
            return self._verify_calibration_controllers(
                timeout=2.0, log_progress=False
            )

        safety_failed = False
        next_safety_check = time.monotonic()
        while rclpy.ok() and len(samples) < self.num_poses:
            now = time.monotonic()
            if now >= next_safety_check:
                if not sampling_safety_ok():
                    safety_failed = True
                    break
                next_safety_check = now + self.safety_recheck_period_sec

            with self._image_lock:
                frame = None if self._latest_image is None else self._latest_image.copy()
                image_time = self._latest_image_time
                K = None if self._camera_matrix is None else self._camera_matrix.copy()
                D = None if self._dist_coeffs is None else self._dist_coeffs.copy()
            if frame is None or image_time is None or K is None or D is None:
                time.sleep(0.05)
                continue

            display = self._detector.draw_detection(frame, K, D)
            cv2.putText(
                display, f"Captured: {len(samples)}/{self.num_poses}",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2
            )
            cv2.imshow(window, display)
            key = cv2.waitKey(1) & 0xFF

            if key == ord(" "):
                if not sampling_safety_ok():
                    safety_failed = True
                    break
                joints = self._get_current_joint_positions()
                T_base_gripper = self._get_current_ee_pose(image_time=image_time)
                if joints is None or T_base_gripper is None:
                    continue
                T_camera_target = self._detector.detect(frame, K, D)
                if T_camera_target is None:
                    self.get_logger().warn("Target not detected; sample rejected")
                    continue

                index = len(samples)
                image_path = os.path.join(
                    self.image_save_dir, f"calib_{index:02d}.png"
                )
                if not cv2.imwrite(image_path, frame):
                    self.get_logger().error(f"Unable to write {image_path}")
                    continue
                try:
                    self._calibrator.add_sample(
                        T_base_gripper, T_camera_target
                    )
                except ValueError as exc:
                    self.get_logger().warn(f"Invalid sample rejected: {exc}")
                    continue
                samples.append({
                    "index": index,
                    "image": image_path,
                    "joints_rad": [round(x, 8) for x in joints],
                    "robot_pose": self._pose_record(
                        f"T_{self.base_frame}_{self.ee_link}", T_base_gripper
                    ),
                    "target_pose": self._pose_record(
                        f"T_{self.camera_frame}_target", T_camera_target
                    ),
                })
                self.get_logger().info(
                    f"Captured {index + 1}/{self.num_poses}"
                )
            elif key in (ord("q"), ord("Q"), 27):
                break

        cv2.destroyAllWindows()
        self._save_samples(samples)
        if safety_failed:
            return False
        if len(samples) < 3:
            self.get_logger().error(
                f"Too few valid samples ({len(samples)}); need at least 3"
            )
            return False

        try:
            result, residual = self._calibrator.calibrate()
        except (cv2.error, ValueError, RuntimeError, np.linalg.LinAlgError) as exc:
            self.get_logger().error(f"Calibration failed: {exc}")
            return False
        self._calibrator.save_result(
            self.result_file, self.base_frame, self.camera_frame
        )
        p = result[:3, 3]
        self.get_logger().info(
            f"Camera origin in {self.base_frame}: "
            f"[{p[0]:.6f}, {p[1]:.6f}, {p[2]:.6f}] m; "
            f"residual={residual:.8f}"
        )
        return True


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    outcome = {"success": False}

    def run_and_shutdown():
        try:
            outcome["success"] = node.run()
        finally:
            if rclpy.ok():
                rclpy.shutdown()

    thread = threading.Thread(target=run_and_shutdown, daemon=True)
    thread.start()
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        thread.join(timeout=2.0)
        executor.shutdown()
        node.destroy_node()
    return 0 if outcome["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
