import sys
import os
import threading
import time

import yaml
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Image, CameraInfo, JointState
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float64MultiArray
import tf2_ros
from tf2_ros import TransformException
from cv_bridge import CvBridge

from pymoveit2 import MoveIt2

from .board_detector import BoardDetector
from .calibrator import HandEyeCalibrator
from .sample_collector import SampleCollector

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


class CalibrationNode(Node):
    def __init__(self):
        super().__init__("calibration_node")

        self._cb_group = ReentrantCallbackGroup()

        # --- parameters ---
        self.declare_parameter("eye_mode", "eye_in_hand")
        self.declare_parameter("method", "park")
        self.declare_parameter("num_poses", 15)
        self.declare_parameter("move_group_name", "piper_arm")
        self.declare_parameter("end_effector_link", "link6")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_topic", "/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("camera_frame", "wrist_cam_link")
        self.declare_parameter("board_type", "charuco")
        self.declare_parameter("charuco_rows", 9)
        self.declare_parameter("charuco_cols", 14)
        self.declare_parameter("charuco_square_length", 0.02)
        self.declare_parameter("charuco_marker_length", 0.015)
        self.declare_parameter("aruco_dict", "DICT_5X5_100")
        self.declare_parameter("result_file", "calibration_result.yaml")
        self.declare_parameter("output_frame", "camera_optical_frame")
        self.declare_parameter("seed_joints", [0.0, 1.57, -1.3485, 0.0, 0.0, 0.0])
        self.declare_parameter("noise_per_joint", [0.05, 0.08, 0.08, 0.10, 0.05, 0.10])
        self.declare_parameter("real_robot", False)
        self.declare_parameter("image_save_dir", "calibration_images")

        self.eye_mode = self.get_parameter("eye_mode").value
        self.method = self.get_parameter("method").value
        self.num_poses = self.get_parameter("num_poses").value
        self.move_group_name = self.get_parameter("move_group_name").value
        self.ee_link = self.get_parameter("end_effector_link").value
        self.base_frame = self.get_parameter("base_frame").value
        self.camera_topic = self.get_parameter("camera_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.result_file = self.get_parameter("result_file").value
        self.output_frame = self.get_parameter("output_frame").value
        self._real_robot = self.get_parameter("real_robot").value
        self._image_save_dir = self.get_parameter("image_save_dir").value

        self._bridge = CvBridge()
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # camera data (ROS mode)
        self._latest_image: np.ndarray | None = None
        self._camera_matrix: np.ndarray | None = None
        self._dist_coeffs: np.ndarray | None = None
        self._image_lock = threading.Lock()

        # joint state data
        self._joint_positions: dict[str, float] = {}
        self._joint_lock = threading.Lock()

        self._detector = BoardDetector(
            board_type=self.get_parameter("board_type").value,
            charuco_rows=self.get_parameter("charuco_rows").value,
            charuco_cols=self.get_parameter("charuco_cols").value,
            charuco_square_length=self.get_parameter("charuco_square_length").value,
            charuco_marker_length=self.get_parameter("charuco_marker_length").value,
            aruco_dict_name=self.get_parameter("aruco_dict").value,
        )

        self._calibrator = HandEyeCalibrator(
            self, method=self.method, eye_mode=self.eye_mode
        )

        self._collector = SampleCollector(
            num_poses=self.num_poses,
            seed_joints=self.get_parameter("seed_joints").value,
            noise_per_joint=self.get_parameter("noise_per_joint").value,
        )

        self._moveit2: MoveIt2 | None = None

        self._viz_pub = self.create_publisher(
            MarkerArray, "calibration_markers", 10,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            f"CalibrationNode initialized (real_robot={self._real_robot})")

    # ===================================================================
    # Common helpers
    # ===================================================================

    def _get_current_ee_pose(self) -> np.ndarray:
        try:
            t = self._tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_link,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
            T = np.eye(4)
            T[:3, 3] = [
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
            ]
            q = t.transform.rotation
            T[:3, :3] = self._quat_to_rot([q.x, q.y, q.z, q.w])
            return T
        except TransformException as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return np.eye(4)

    def _get_current_joint_positions(self) -> list[float] | None:
        with self._joint_lock:
            if len(self._joint_positions) < 6:
                return None
            return [self._joint_positions.get(f"joint{i}", 0.0)
                    for i in range(1, 7)]

    @staticmethod
    def _quat_to_rot(q):
        x, y, z, w = q
        return np.array([
            [1 - 2*y*y - 2*z*z, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
            [2*x*y + 2*w*z, 1 - 2*x*x - 2*z*z, 2*y*z - 2*w*x],
            [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x*x - 2*y*y],
        ])

    # ===================================================================
    # Simulation mode (unchanged: random poses + MoveIt + ROS camera)
    # ===================================================================

    def _setup_moveit2(self):
        self._moveit2 = MoveIt2(
            node=self,
            joint_names=JOINT_NAMES,
            base_link_name=self.base_frame,
            end_effector_name=self.ee_link,
            group_name=self.move_group_name,
            use_move_group_action=True,
            callback_group=self._cb_group,
        )
        self._moveit2.max_velocity = 0.3
        self._moveit2.max_acceleration = 0.3
        self.get_logger().info(
            f"pymoveit2 initialized: group={self.move_group_name}")

    def _subscribe_camera_ros(self):
        self.create_subscription(
            Image, self.camera_topic, self._image_callback, 1,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self._camera_info_callback, 1,
            callback_group=self._cb_group,
        )
        self.get_logger().info(
            f"Subscribed to {self.camera_topic} and {self.camera_info_topic}")

    def _image_callback(self, msg: Image):
        with self._image_lock:
            self._latest_image = self._bridge.imgmsg_to_cv2(
                msg, desired_encoding="bgr8")

    def _camera_info_callback(self, msg: CameraInfo):
        if self._camera_matrix is not None:
            return
        self._camera_matrix = np.array(msg.k).reshape(3, 3)
        self._dist_coeffs = np.array(msg.d)
        self.get_logger().info("Camera info received")

    def _wait_for_camera_ros(self, timeout: float = 30.0):
        start = time.time()
        while time.time() - start < timeout:
            with self._image_lock:
                if self._latest_image is not None \
                        and self._camera_matrix is not None:
                    return True
            time.sleep(0.1)
        return False

    def _move_to_joints(self, joint_positions: np.ndarray) -> bool:
        succeeded = self._moveit2.move_to_configuration(
            joint_positions[:6].tolist())
        if not succeeded:
            self.get_logger().warn("Move to configuration failed")
        return succeeded

    def _detect_board_at_current_pose(self) -> np.ndarray | None:
        with self._image_lock:
            if self._latest_image is None:
                return None
            img = self._latest_image.copy()

        T_cam_board = self._detector.detect(
            img, self._camera_matrix, self._dist_coeffs)
        if T_cam_board is not None:
            self.get_logger().info("Board detected successfully")
        return T_cam_board

    def run_simulation(self):
        self.get_logger().info("=== Hand-Eye Calibration (Simulation) ===")
        self.get_logger().info(f"Mode: {self.eye_mode}, Method: {self.method}")
        self.get_logger().info(f"Poses: {self.num_poses}")

        self._setup_moveit2()
        self._subscribe_camera_ros()

        if not self._wait_for_camera_ros():
            self.get_logger().error("Camera timeout")
            return

        poses = self._collector.generate()
        self.get_logger().info(f"Generated {len(poses)} calibration poses")

        collected = 0
        for i, q in enumerate(poses):
            self.get_logger().info(f"--- Pose {i + 1}/{len(poses)} ---")

            if not self._move_to_joints(q):
                self.get_logger().warn(f"Pose {i}: move failed, skipping")
                continue

            time.sleep(0.5)

            T_robot = self._get_current_ee_pose()
            T_board = self._detect_board_at_current_pose()

            if T_board is None:
                self.get_logger().warn(
                    f"Pose {i}: board not detected, skipping")
                continue

            self._calibrator.add_sample(T_robot, T_board)
            collected += 1
            self.get_logger().info(
                f"Collected sample {collected}: "
                f"ee_pos=({T_robot[0,3]:.3f}, {T_robot[1,3]:.3f}, "
                f"{T_robot[2,3]:.3f})")

        self._finish_calibration(collected)

    # ===================================================================
    # Real robot mode (ROS话题订阅，兼容所有相机)
    # ===================================================================

    def run_real_robot(self):
        self.get_logger().info("=== Hand-Eye Calibration (Real Robot) ===")
        self.get_logger().info(f"Mode: {self.eye_mode}, Method: {self.method}")
        self.get_logger().info(f"Target: {self.num_poses} samples")

        # subscribe to joint states from ros2_control
        self.create_subscription(
            JointState, "/joint_states", self._joint_states_callback, 10,
            callback_group=self._cb_group,
        )

        # 统一使用ROS话题订阅相机（兼容RealSense和Orbbec）
        # 确保相机驱动节点已启动（如：ros2 launch orbbec_camera femto_bolt.launch.py）
        self.get_logger().info("Subscribing to camera topics via ROS...")
        self.get_logger().info(f"Camera topic: {self.camera_topic}")
        self.get_logger().info(f"Camera info topic: {self.camera_info_topic}")

        self._subscribe_camera_ros()

        # wait for camera and joint states
        self.get_logger().info("Waiting for camera and joint states...")
        if not self._wait_for_camera_ros(timeout=30.0):
            self.get_logger().error("Camera timeout, please ensure camera node is running")
            self.get_logger().error("  For Orbbec: ros2 launch orbbec_camera femto_bolt.launch.py")
            self.get_logger().error("  For RealSense: ros2 launch realsense2_camera rs_launch.py")
            return

        if self._get_current_joint_positions() is None:
            self.get_logger().warn("No joint states received yet, continuing anyway...")

        # prepare image save directory
        os.makedirs(self._image_save_dir, exist_ok=True)

        # --- Phase 1: interactive capture ---
        collected_images: list[str] = []
        collected_joints: list[list[float]] = []
        collected_ee_poses: list[np.ndarray] = []
        window_name = "Calibration Capture [Space=save, Q/Esc=quit]"
        cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

        self.get_logger().info(
            "Move the robot arm manually, then press SPACE to capture.")
        self.get_logger().info(
            f"Press Q or Esc to quit early (need >= 3 valid samples).")

        while len(collected_images) < self.num_poses:
            # 使用ROS话题的图像
            with self._image_lock:
                if self._latest_image is None:
                    time.sleep(0.1)
                    continue
                frame = self._latest_image.copy()

            display = frame.copy()

            # overlay status
            count = len(collected_images)
            text = f"Captured: {count}/{self.num_poses}"
            cv2.putText(display, text, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            cv2.putText(display, "SPACE=capture  Q=finish",
                        (10, display.shape[0] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

            cv2.imshow(window_name, display)
            key = cv2.waitKey(1) & 0xFF

            if key == ord(' '):
                # save raw image
                filename = os.path.join(
                    self._image_save_dir, f"calib_{count:02d}.png")
                cv2.imwrite(filename, frame)
                collected_images.append(filename)

                # record joint positions
                joints = self._get_current_joint_positions()
                if joints is None:
                    joints = [0.0] * 6
                    self.get_logger().warn(
                        f"Sample {count}: no joint data, using zeros")
                collected_joints.append(joints)

                # cache EE pose NOW (while robot is still in this pose)
                T_ee = self._get_current_ee_pose()
                collected_ee_poses.append(T_ee)

                self.get_logger().info(
                    f"Sample {count + 1}/{self.num_poses} saved: "
                    f"{filename}  joints={[f'{j:.4f}' for j in joints]}")

            elif key in (ord('q'), ord('Q'), 27):  # Q or Escape
                self.get_logger().info(
                    f"Early exit with {count} samples")
                break

        cv2.destroyAllWindows()

        if len(collected_images) < 3:
            self.get_logger().error(
                f"Too few samples ({len(collected_images)}), need at least 3")
            return

        # --- Phase 2: detect boards and calibrate ---
        self.get_logger().info(
            f"Processing {len(collected_images)} captured images...")

        # Save all captured sample data for inspection
        samples_file = os.path.splitext(self.result_file)[0] + "_samples.yaml"
        samples_data = []

        valid = 0
        for i, (img_path, joints, T_ee) in enumerate(
                zip(collected_images, collected_joints, collected_ee_poses)):
            img = cv2.imread(img_path)
            T_cam_board = self._detector.detect(
                img, self._camera_matrix, self._dist_coeffs)

            sample = {
                "index": i,
                "image": img_path,
                "joints_rad": [round(j, 6) for j in joints],
                "ee_position": {
                    "x": round(float(T_ee[0, 3]), 6),
                    "y": round(float(T_ee[1, 3]), 6),
                    "z": round(float(T_ee[2, 3]), 6),
                },
                "board_detected": T_cam_board is not None,
            }

            if T_cam_board is None:
                self.get_logger().warn(
                    f"Image {i}: board NOT detected, skipping")
                samples_data.append(sample)
                continue

            sample["board_position"] = {
                "x": round(float(T_cam_board[0, 3]), 6),
                "y": round(float(T_cam_board[1, 3]), 6),
                "z": round(float(T_cam_board[2, 3]), 6),
            }
            samples_data.append(sample)

            self._calibrator.add_sample(T_ee, T_cam_board)
            valid += 1
            self.get_logger().info(
                f"Image {i}: board detected, sample added ({valid} valid)")

        # Save samples data
        os.makedirs(os.path.dirname(samples_file) or ".", exist_ok=True)
        with open(samples_file, "w") as f:
            yaml.dump(samples_data, f, default_flow_style=False, sort_keys=False)
        self.get_logger().info(f"Samples data saved to {samples_file}")

        self._finish_calibration(valid)

    def _joint_states_callback(self, msg: JointState):
        with self._joint_lock:
            for name, pos in zip(msg.name, msg.position):
                self._joint_positions[name] = pos

    # ===================================================================
    # Shared calibration finish
    # ===================================================================

    def _finish_calibration(self, valid_count: int):
        if valid_count < 3:
            self.get_logger().error(
                f"Too few valid samples ({valid_count}), need at least 3")
            return

        self.get_logger().info(
            f"Running calibration with {valid_count} samples...")
        T_result, error = self._calibrator.calibrate()
        self.get_logger().info(f"Result:\n{T_result}")
        self.get_logger().info(f"Error: {error:.8f}")

        self._calibrator.save_result(self.result_file)

        # 根据标定模式设置TF父子帧
        # eye_in_hand: 结果是 ee_link -> camera_frame (相机安装在末端)
        # eye_to_hand: 结果是 base_link -> camera_link (相机固定在外部)
        if self.eye_mode == "eye_to_hand":
            parent_frame = self.base_frame
            child_frame = self.camera_frame
        else:
            parent_frame = self.ee_link
            child_frame = self.camera_frame

        self._calibrator.publish_tf(
            parent_frame=parent_frame,
            child_frame=child_frame,
        )

        self.get_logger().info("=== Calibration complete ===")
        self.get_logger().info(f"Result saved to {self.result_file}")
        self.get_logger().info(
            f"Static TF: {parent_frame} -> {child_frame}")

    # ===================================================================
    # Entry
    # ===================================================================

    def run(self):
        if self._real_robot:
            self.run_real_robot()
        else:
            self.run_simulation()

    def destroy(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()

    executor = MultiThreadedExecutor(2)
    executor.add_node(node)

    run_thread = threading.Thread(target=node.run, daemon=True)
    run_thread.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
