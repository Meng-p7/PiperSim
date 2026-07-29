import cv2
import cv2.aruco as aruco
import numpy as np
from typing import Optional


_ARUCO_DICT_MAP = {
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_APRILTAG_16H5": cv2.aruco.DICT_APRILTAG_16H5,
    "DICT_APRILTAG_25H9": cv2.aruco.DICT_APRILTAG_25H9,
    "DICT_APRILTAG_36H10": cv2.aruco.DICT_APRILTAG_36H10,
    "DICT_APRILTAG_36H11": cv2.aruco.DICT_APRILTAG_36H11,
}


class BoardDetector:
    def __init__(
        self,
        board_type: str = "charuco",
        charuco_rows: int = 9,
        charuco_cols: int = 14,
        charuco_square_length: float = 0.03,
        charuco_marker_length: float = 0.022,
        aruco_dict_name: str = "DICT_5X5_100",
        chessboard_rows: int = 10,
        chessboard_cols: int = 12,
        aruco_marker_length: float = 0.05,  # 单个ArUco标记尺寸（米）
        aruco_marker_id: int = 0,  # 单个ArUco标记ID
    ):
        self.board_type = board_type
        self.aruco_marker_length = aruco_marker_length
        self.aruco_marker_id = aruco_marker_id

        aruco_dict_id = _ARUCO_DICT_MAP.get(aruco_dict_name, cv2.aruco.DICT_5X5_100)
        self.aruco_dict = aruco.getPredefinedDictionary(aruco_dict_id)

        # OpenCV 5.0.0+: 使用ArucoDetector
        detector_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, detector_params)

        self.charuco_rows = charuco_rows
        self.charuco_cols = charuco_cols
        self.charuco_square_length = charuco_square_length
        self.charuco_marker_length = charuco_marker_length
        self.chessboard_rows = chessboard_rows
        self.chessboard_cols = chessboard_cols

        # OpenCV 5.0.0+: 使用CharucoBoard类创建board
        self.charuco_board = cv2.aruco.CharucoBoard(
            (self.charuco_cols, self.charuco_rows),  # size as tuple
            self.charuco_square_length,
            self.charuco_marker_length,
            self.aruco_dict,
        )
        
        # 创建CharucoDetector (OpenCV 5.0.0+)
        charuco_params = cv2.aruco.CharucoParameters()
        charuco_params.tryRefineMarkers = True
        self.charuco_detector = cv2.aruco.CharucoDetector(self.charuco_board, charuco_params)

    def detect(self, image: np.ndarray, camera_matrix: np.ndarray, dist_coeffs: np.ndarray) -> Optional[np.ndarray]:
        if self.board_type == "charuco":
            return self._detect_charuco(image, camera_matrix, dist_coeffs)
        elif self.board_type == "chessboard":
            return self._detect_chessboard(image, camera_matrix, dist_coeffs)
        elif self.board_type == "aruco":
            return self._detect_single_aruco(image, camera_matrix, dist_coeffs)
        return None

    def _detect_charuco(self, image: np.ndarray, K: np.ndarray, D: np.ndarray) -> Optional[np.ndarray]:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # OpenCV 5.0.0+: 使用CharucoDetector
        charuco_corners, charuco_ids, marker_corners, marker_ids = self.charuco_detector.detectBoard(gray, K, D)
        
        if charuco_corners is None or len(charuco_corners) < 4:
            return None

        # 使用solvePnP估计位姿
        obj_points = []
        img_points = []
        for i, corner_id in enumerate(charuco_ids):
            # 获取ChArUco board上的3D点
            point = self.charuco_board.getChessboardCorners()[corner_id]
            obj_points.append(point)
            img_points.append(charuco_corners[i])
        
        obj_points = np.array(obj_points, dtype=np.float32)
        img_points = np.array(img_points, dtype=np.float32)
        
        ret, rvec, tvec = cv2.solvePnP(obj_points, img_points, K, D)
        if not ret:
            return None

        T_cam_board = np.eye(4)
        T_cam_board[:3, :3] = cv2.Rodrigues(rvec)[0]
        T_cam_board[:3, 3] = tvec.flatten()
        return T_cam_board

    def _detect_chessboard(self, image: np.ndarray, K: np.ndarray, D: np.ndarray) -> Optional[np.ndarray]:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        pattern = (self.chessboard_cols, self.chessboard_rows)
        flags = (
            cv2.CALIB_CB_ADAPTIVE_THRESH
            + cv2.CALIB_CB_NORMALIZE_IMAGE
            + cv2.CALIB_CB_FAST_CHECK
        )
        ret, corners = cv2.findChessboardCorners(gray, pattern, flags)
        if not ret:
            return None

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
        corners_refined = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)

        objp = np.zeros((self.chessboard_rows * self.chessboard_cols, 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.chessboard_cols, 0:self.chessboard_rows].T.reshape(-1, 2)
        objp *= self.charuco_square_length

        ret, rvec, tvec = cv2.solvePnP(objp, corners_refined, K, D)
        if not ret:
            return None

        T_cam_board = np.eye(4)
        T_cam_board[:3, :3] = cv2.Rodrigues(rvec)[0]
        T_cam_board[:3, 3] = tvec.flatten()
        return T_cam_board

    def _detect_single_aruco(self, image: np.ndarray, K: np.ndarray, D: np.ndarray) -> Optional[np.ndarray]:
        """检测单个 ArUco/AprilTag 标记"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # OpenCV 5.0.0+: 使用ArucoDetector
        marker_corners, marker_ids, rejected = self.aruco_detector.detectMarkers(gray)
        
        if marker_ids is None or len(marker_ids) == 0:
            return None

        # 查找指定 ID 的标记（如果指定了）
        if self.aruco_marker_id >= 0:
            found_idx = None
            for i, mid in enumerate(marker_ids):
                if mid == self.aruco_marker_id:
                    found_idx = i
                    break
            if found_idx is None:
                return None
            
            # 只使用找到的标记
            marker_corners = [marker_corners[found_idx]]
            marker_ids = [marker_ids[found_idx]]

        # 使用solvePnP估计单个标记的位姿
        half_size = self.aruco_marker_length / 2.0
        obj_points = np.array([
            [-half_size, -half_size, 0],
            [half_size, -half_size, 0],
            [half_size, half_size, 0],
            [-half_size, half_size, 0]
        ], dtype=np.float32)
        
        img_points = marker_corners[0].reshape(4, 2).astype(np.float32)
        
        ret, rvec, tvec = cv2.solvePnP(obj_points, img_points, K, D)
        if not ret:
            return None

        T_cam_board = np.eye(4)
        T_cam_board[:3, :3] = cv2.Rodrigues(rvec)[0]
        T_cam_board[:3, 3] = tvec.flatten()
        return T_cam_board

    def draw_detection(self, image: np.ndarray, camera_matrix: np.ndarray, dist_coeffs: np.ndarray) -> np.ndarray:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        marker_corners, marker_ids, rejected = self.aruco_detector.detectMarkers(gray)
        result = image.copy()
        if marker_ids is not None and len(marker_ids) > 0:
            cv2.aruco.drawDetectedMarkers(result, marker_corners, marker_ids)
            
            if self.board_type == "charuco":
                charuco_corners, charuco_ids, _, _ = self.charuco_detector.detectBoard(gray, camera_matrix, dist_coeffs)
                if charuco_corners is not None and len(charuco_corners) > 0:
                    cv2.aruco.drawDetectedCornersCharuco(result, charuco_corners, charuco_ids)
                    
                    # 估计位姿并绘制坐标轴
                    obj_points = []
                    img_points = []
                    for i, corner_id in enumerate(charuco_ids):
                        point = self.charuco_board.getChessboardCorners()[corner_id]
                        obj_points.append(point)
                        img_points.append(charuco_corners[i])
                    
                    obj_points = np.array(obj_points, dtype=np.float32)
                    img_points = np.array(img_points, dtype=np.float32)
                    
                    ret, rvec, tvec = cv2.solvePnP(obj_points, img_points, camera_matrix, dist_coeffs)
                    if ret:
                        cv2.drawFrameAxes(result, camera_matrix, dist_coeffs, rvec, tvec, 0.03)
        return result