#!/usr/bin/env python3
"""
通过相机检测 AprilTag，显示：
  - 相机坐标系下位置 (camera)
  - 末端坐标系下位置 (ee_frame)
  - 基座坐标系下位置 (base_position)
按 Q 退出。
"""
import argparse
import time
import threading
import yaml
import cv2
import cv2.aruco as aruco
import numpy as np
import pyrealsense2 as rs

import rclpy
from rclpy.node import Node
import tf2_ros


def load_calibration(path):
    with open(path, "r") as f:
        data = yaml.safe_load(f)
    T = np.eye(4)
    T[:3, :3] = np.array(data["rotation_matrix"])
    T[0, 3] = data["translation"]["x"]
    T[1, 3] = data["translation"]["y"]
    T[2, 3] = data["translation"]["z"]
    return T, data


def get_base_to_ee(tf_buffer):
    """通过 TF 获取基座到末端的变换矩阵"""
    try:
        t = tf_buffer.lookup_transform("base_link", "link6",
                                        rclpy.time.Time(),
                                        timeout=rclpy.duration.Duration(seconds=1.0))
        T = np.eye(4)
        T[:3, 3] = [t.transform.translation.x,
                     t.transform.translation.y,
                     t.transform.translation.z]
        q = t.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        T[:3, :3] = np.array([
            [1-2*y*y-2*z*z, 2*x*y-2*w*z, 2*x*z+2*w*y],
            [2*x*y+2*w*z, 1-2*x*x-2*z*z, 2*y*z-2*w*x],
            [2*x*z-2*w*y, 2*y*z+2*w*x, 1-2*x*x-2*y*y]])
        return T
    except Exception:
        return None


class TFNode(Node):
    def __init__(self):
        super().__init__("tag_position_viewer")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--result-file", default="real_eye_in_hand_result.yaml")
    parser.add_argument("--tag-id", type=int, default=1)
    parser.add_argument("--tag-size", type=float, default=0.057)
    args = parser.parse_args()

    T_cam_ee, info = load_calibration(args.result_file)
    T_ee_cam = np.linalg.inv(T_cam_ee)
    print(f"标定结果: method={info.get('method')}, error={info.get('error'):.6f}")
    print(f"末端到相机偏移: [{T_ee_cam[0,3]:.4f}, {T_ee_cam[1,3]:.4f}, {T_ee_cam[2,3]:.4f}]")
    print()

    # ROS2 + TF
    rclpy.init()
    node = TFNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    time.sleep(2)

    # 相机
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

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    params = cv2.aruco.DetectorParameters_create()

    print(f"查找 Tag ID={args.tag_id}，按 Q 退出\n")

    try:
        while True:
            frames = pipeline.wait_for_frames()
            cf = frames.get_color_frame()
            if not cf:
                continue

            image = np.asanyarray(cf.get_data())
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.detectMarkers(gray, dictionary, parameters=params)

            vis = image.copy()

            if ids is not None and len(ids) > 0:
                aruco.drawDetectedMarkers(vis, corners, ids)
                for i, tid in enumerate(ids.flatten().tolist()):
                    half = args.tag_size / 2.0
                    obj = np.array([[-half,-half,0],[half,-half,0],
                                    [half,half,0],[-half,half,0]], dtype=np.float32)
                    img_pts = corners[i].reshape(4, 2).astype(np.float32)
                    ok, rvec, tvec = cv2.solvePnP(obj, img_pts, K, D)
                    if ok:
                        dist = np.linalg.norm(tvec)
                        cv2.drawFrameAxes(vis, K, D, rvec, tvec, 0.05)
                        cv2.putText(vis, f"ID={tid} dist={dist:.3f}m", (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                        if tid == args.tag_id:
                            R_m, _ = cv2.Rodrigues(rvec)
                            T_cam_tag = np.eye(4)
                            T_cam_tag[:3, :3] = R_m
                            T_cam_tag[:3, 3] = tvec.flatten()

                            # 相机坐标系下
                            cam_pos = T_cam_tag[:3, 3]

                            # 末端坐标系下
                            T_ee_tag = T_ee_cam @ T_cam_tag
                            ee_pos = T_ee_tag[:3, 3]

                            # 基座坐标系下
                            T_base_ee = get_base_to_ee(node.tf_buffer)
                            if T_base_ee is not None:
                                T_base_tag = T_base_ee @ T_ee_cam @ T_cam_tag
                                base_pos = T_base_tag[:3, 3]

                                # 显示在画面上
                                y0 = 60
                                cv2.putText(vis, f"camera:  [{cam_pos[0]:.3f}, {cam_pos[1]:.3f}, {cam_pos[2]:.3f}]",
                                            (10, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
                                cv2.putText(vis, f"ee_frame:[{ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f}]",
                                            (10, y0+20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
                                cv2.putText(vis, f"base:    [{base_pos[0]:.3f}, {base_pos[1]:.3f}, {base_pos[2]:.3f}]",
                                            (10, y0+40), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

                                print(f"\rcamera=[{cam_pos[0]:.4f}, {cam_pos[1]:.4f}, {cam_pos[2]:.4f}]  "
                                      f"ee=[{ee_pos[0]:.4f}, {ee_pos[1]:.4f}, {ee_pos[2]:.4f}]  "
                                      f"base=[{base_pos[0]:.4f}, {base_pos[1]:.4f}, {base_pos[2]:.4f}]",
                                      end="", flush=True)
                            else:
                                cv2.putText(vis, "TF: base_link->link6 not available", (10, 60),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            else:
                cv2.putText(vis, f"Looking for ID={args.tag_id}...", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            cv2.imshow("AprilTag Position", vis)
            if cv2.waitKey(1) & 0xFF in (ord('q'), ord('Q'), 27):
                break
            time.sleep(0.03)

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
        print("\nDone.")


if __name__ == "__main__":
    main()
