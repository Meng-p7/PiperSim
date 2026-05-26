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
    ):
        self.board_type = board_type

        aruco_dict_id = _ARUCO_DICT_MAP.get(aruco_dict_name, cv2.aruco.DICT_5X5_100)
        self.aruco_dict = aruco.getPredefinedDictionary(aruco_dict_id)

        self.charuco_rows = charuco_rows
        self.charuco_cols = charuco_cols
        self.charuco_square_length = charuco_square_length
        self.charuco_marker_length = charuco_marker_length
        self.chessboard_rows = chessboard_rows
        self.chessboard_cols = chessboard_cols

        self.charuco_board = aruco.CharucoBoard_create(
            self.charuco_cols,
            self.charuco_rows,
            self.charuco_square_length,
            self.charuco_marker_length,
            self.aruco_dict,
        )

    def detect(self, image: np.ndarray, camera_matrix: np.ndarray, dist_coeffs: np.ndarray) -> Optional[np.ndarray]:
        if self.board_type == "charuco":
            return self._detect_charuco(image, camera_matrix, dist_coeffs)
        elif self.board_type == "chessboard":
            return self._detect_chessboard(image, camera_matrix, dist_coeffs)
        return None

    def _detect_charuco(self, image: np.ndarray, K: np.ndarray, D: np.ndarray) -> Optional[np.ndarray]:
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

        marker_corners, marker_ids, _ = aruco.detectMarkers(gray, self.aruco_dict)
        if marker_ids is None or len(marker_ids) < 4:
            return None

        ret, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
            marker_corners, marker_ids, gray, self.charuco_board
        )
        if not ret or charuco_corners is None or len(charuco_corners) < 4:
            return None

        ret, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
            charuco_corners, charuco_ids, self.charuco_board, K, D, None, None
        )
        if not ret:
            return None

        T_cam_board = np.eye(4)
        T_cam_board[:3, :3] = cv2.Rodrigues(rvec)[0]
        T_cam_board[:3, 3] = tvec.flatten()
        return T_cam_board

    def _detect_chessboard(self, image: np.ndarray, K: np.ndarray, D: np.ndarray) -> Optional[np.ndarray]:
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
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

    def draw_detection(self, image: np.ndarray, camera_matrix: np.ndarray, dist_coeffs: np.ndarray) -> np.ndarray:
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        marker_corners, marker_ids, _ = aruco.detectMarkers(gray, self.aruco_dict)
        result = image.copy()
        if marker_ids is not None and len(marker_ids) > 0:
            aruco.drawDetectedMarkers(result, marker_corners, marker_ids)
            ret, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
                marker_corners, marker_ids, gray, self.charuco_board
            )
            if ret and charuco_corners is not None and len(charuco_corners) > 0:
                cv2.aruco.drawDetectedCornersCharuco(result, charuco_corners, charuco_ids)
                ret2, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                    charuco_corners, charuco_ids, self.charuco_board,
                    camera_matrix, dist_coeffs, None, None
                )
                if ret2:
                    cv2.drawFrameAxes(result, camera_matrix, dist_coeffs, rvec, tvec, 0.03)
        return result
