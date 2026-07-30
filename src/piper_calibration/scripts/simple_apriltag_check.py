#!/usr/bin/env python3
"""
AprilTag 检测脚本（带窗口显示）
用于确认标定板类型和检测效果

用法:
  source ./start_real.sh
  python3 src/piper_calibration/scripts/simple_apriltag_check.py

操作:
  - 将 AprilTag 放在相机视野内
  - 脚本会实时显示相机画面和检测到的标记信息
  - 按 'N' 切换字典类型
  - 按 'Q' 退出
"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer


class SimpleAprilTagCheck(Node):
    def __init__(self):
        super().__init__('simple_apriltag_check')

        # 参数
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')

        # AprilTag 字典（尝试多种）
        self.dictionaries = {
            'DICT_APRILTAG_36h11': cv2.aruco.DICT_APRILTAG_36H11,
            'DICT_APRILTAG_36h10': cv2.aruco.DICT_APRILTAG_36H10,
            'DICT_APRILTAG_25h9': cv2.aruco.DICT_APRILTAG_25H9,
            'DICT_APRILTAG_16h5': cv2.aruco.DICT_APRILTAG_16H5,
            'DICT_6X6_250': cv2.aruco.DICT_6X6_250,
            'DICT_5X5_100': cv2.aruco.DICT_5X5_100,
        }

        self.current_dict_idx = 0
        self.dict_names = list(self.dictionaries.keys())

        # CV Bridge
        self.bridge = CvBridge()

        # 相机参数
        self.camera_matrix = None
        self.dist_coeffs = None

        # 检测参数（优化以提高检测率）
        self.detector_params = (
            cv2.aruco.DetectorParameters()
            if hasattr(cv2.aruco, "DetectorParameters")
            else cv2.aruco.DetectorParameters_create()
        )
        # 角点细化
        self.detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector_params.cornerRefinementWinSize = 5
        self.detector_params.cornerRefinementMinAccuracy = 0.1
        self.detector_params.cornerRefinementMaxIterations = 30
        # 自适应阈值
        self.detector_params.adaptiveThreshWinSizeMin = 3
        self.detector_params.adaptiveThreshWinSizeMax = 23
        self.detector_params.adaptiveThreshWinSizeStep = 10
        self.detector_params.adaptiveThreshConstant = 7

        # 创建检测器（初始使用第一个字典）
        self.detector = self._create_detector(self.dict_names[self.current_dict_idx])

        # 订阅相机话题
        image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value

        self.image_sub = Subscriber(self, Image, image_topic)
        self.camera_info_sub = Subscriber(self, CameraInfo, camera_info_topic)

        self.sync = ApproximateTimeSynchronizer(
            [self.image_sub, self.camera_info_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.callback)

        self.get_logger().info("="*60)
        self.get_logger().info("AprilTag 检测脚本启动")
        self.get_logger().info(f"图像话题: {image_topic}")
        self.get_logger().info(f"相机参数话题: {camera_info_topic}")
        self.get_logger().info("将 AprilTag 放在相机视野内")
        self.get_logger().info("按 'N' 切换字典类型")
        self.get_logger().info("按 'Q' 退出")
        self.get_logger().info("="*60)

        # 创建窗口
        cv2.namedWindow('AprilTag Detector', cv2.WINDOW_AUTOSIZE)

    def _create_detector(self, dict_name):
        """创建ArucoDetector"""
        aruco_dict = cv2.aruco.getPredefinedDictionary(self.dictionaries[dict_name])
        if hasattr(cv2.aruco, "ArucoDetector"):
            return cv2.aruco.ArucoDetector(aruco_dict, self.detector_params)
        return aruco_dict

    def _detect_markers(self, gray):
        if hasattr(self.detector, "detectMarkers"):
            return self.detector.detectMarkers(gray)
        return cv2.aruco.detectMarkers(
            gray, self.detector, parameters=self.detector_params
        )

    def callback(self, image_msg, camera_info_msg):
        try:
            # 更新相机参数
            self.camera_matrix = np.array(camera_info_msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(camera_info_msg.d)

            # 转换图像
            frame = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # 当前字典
            dict_name = self.dict_names[self.current_dict_idx]

            # 检测（使用ArucoDetector）
            marker_corners, marker_ids, rejected = self._detect_markers(gray)

            # 显示信息
            result = frame.copy()
            y_offset = 30

            cv2.putText(result, f"Dictionary: {dict_name}", (10, y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            y_offset += 30

            if marker_ids is not None and len(marker_ids) > 0:
                # 绘制检测到的标记
                cv2.aruco.drawDetectedMarkers(result, marker_corners, marker_ids)

                # 显示检测到的标记数量和 ID
                cv2.putText(result, f"Detected: {len(marker_ids)} markers", (10, y_offset),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                y_offset += 30

                # 显示每个标记的 ID 和像素尺寸
                for i in range(len(marker_ids)):
                    # 计算像素尺寸（边长）
                    width = np.linalg.norm(marker_corners[i][0][0] - marker_corners[i][0][1])
                    height = np.linalg.norm(marker_corners[i][0][1] - marker_corners[i][0][2])
                    
                    mid = marker_ids[i]

                    cv2.putText(result, f"ID={mid}: {width:.0f}x{height:.0f} px",
                                (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                    y_offset += 25

                # 终端输出
                self.get_logger().info("")
                self.get_logger().info("="*60)
                self.get_logger().info(f"✅ 检测成功！")
                self.get_logger().info(f"字典类型: {dict_name}")
                self.get_logger().info(f"检测到 {len(marker_ids)} 个标记")
                for i in range(len(marker_ids)):
                    width = np.linalg.norm(marker_corners[i][0][0] - marker_corners[i][0][1])
                    height = np.linalg.norm(marker_corners[i][0][1] - marker_corners[i][0][2])
                    mid = marker_ids[i]
                    self.get_logger().info(f"  - 标记 ID: {mid}")
                    self.get_logger().info(f"    像素尺寸: {width:.0f} x {height:.0f} px")
                self.get_logger().info("="*60)
            else:
                cv2.putText(result, "No markers detected", (10, y_offset),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                y_offset += 30

            # 显示帮助信息
            cv2.putText(result, "Press 'N' to switch dictionary, 'Q' to quit",
                        (10, result.shape[0] - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

            # 显示结果
            cv2.imshow('AprilTag Detector', result)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('n') or key == ord('N'):
                self.current_dict_idx = (self.current_dict_idx + 1) % len(self.dict_names)
                # 重新创建检测器
                self.detector = self._create_detector(self.dict_names[self.current_dict_idx])
                self.get_logger().info(f"切换到字典: {self.dict_names[self.current_dict_idx]}")
            elif key == ord('q') or key == ord('Q') or key == 27:
                self.get_logger().info("退出")
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"处理图像失败: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())


def main(args=None):
    rclpy.init(args=args)
    checker = SimpleAprilTagCheck()

    try:
        rclpy.spin(checker)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        checker.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
