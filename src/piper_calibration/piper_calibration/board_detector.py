"""Calibration target detection with OpenCV 4.x/5.x API compatibility."""

from typing import Optional

import cv2
import numpy as np


_DICT_NAMES = (
    "DICT_4X4_100", "DICT_5X5_100", "DICT_5X5_250", "DICT_6X6_250",
    "DICT_7X7_100", "DICT_APRILTAG_16H5", "DICT_APRILTAG_25H9",
    "DICT_APRILTAG_36H10", "DICT_APRILTAG_36H11",
)
_DICT_MAP = {name: getattr(cv2.aruco, name) for name in _DICT_NAMES
             if hasattr(cv2.aruco, name)}


class BoardDetector:
    def __init__(
        self,
        board_type: str = "aruco",
        charuco_rows: int = 9,
        charuco_cols: int = 14,
        charuco_square_length: float = 0.02,
        charuco_marker_length: float = 0.015,
        aruco_dict_name: str = "DICT_APRILTAG_36H11",
        chessboard_rows: int = 10,
        chessboard_cols: int = 12,
        aruco_marker_length: float = 0.057,
        aruco_marker_id: int = 0,
    ):
        if board_type not in ("aruco", "charuco", "chessboard"):
            raise ValueError("board_type must be aruco, charuco, or chessboard")
        if aruco_dict_name not in _DICT_MAP:
            raise ValueError(f"Unsupported ArUco dictionary: {aruco_dict_name}")

        self.board_type = board_type
        self.aruco_marker_length = float(aruco_marker_length)
        self.aruco_marker_id = int(aruco_marker_id)
        self.charuco_rows = int(charuco_rows)
        self.charuco_cols = int(charuco_cols)
        self.charuco_square_length = float(charuco_square_length)
        self.charuco_marker_length = float(charuco_marker_length)
        self.chessboard_rows = int(chessboard_rows)
        self.chessboard_cols = int(chessboard_cols)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(_DICT_MAP[aruco_dict_name])

        if hasattr(cv2.aruco, "DetectorParameters"):
            self.detector_params = cv2.aruco.DetectorParameters()
        else:
            self.detector_params = cv2.aruco.DetectorParameters_create()
        self.aruco_detector = (
            cv2.aruco.ArucoDetector(self.aruco_dict, self.detector_params)
            if hasattr(cv2.aruco, "ArucoDetector") else None
        )

        if hasattr(cv2.aruco, "CharucoBoard"):
            self.charuco_board = cv2.aruco.CharucoBoard(
                (self.charuco_cols, self.charuco_rows),
                self.charuco_square_length,
                self.charuco_marker_length,
                self.aruco_dict,
            )
        else:
            self.charuco_board = cv2.aruco.CharucoBoard_create(
                self.charuco_cols,
                self.charuco_rows,
                self.charuco_square_length,
                self.charuco_marker_length,
                self.aruco_dict,
            )
        self.charuco_detector = (
            cv2.aruco.CharucoDetector(self.charuco_board)
            if hasattr(cv2.aruco, "CharucoDetector") else None
        )

    def _detect_markers(self, gray):
        if self.aruco_detector is not None:
            return self.aruco_detector.detectMarkers(gray)
        return cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.detector_params
        )

    def detect(self, image: np.ndarray, K: np.ndarray, D: np.ndarray) -> Optional[np.ndarray]:
        if self.board_type == "aruco":
            return self._detect_single_aruco(image, K, D)
        if self.board_type == "charuco":
            return self._detect_charuco(image, K, D)
        return self._detect_chessboard(image, K, D)

    def _detect_single_aruco(self, image, K, D):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detect_markers(gray)
        if ids is None:
            return None
        matches = np.flatnonzero(ids.reshape(-1).astype(int) == self.aruco_marker_id)
        if len(matches) == 0:
            return None
        half = self.aruco_marker_length / 2.0
        obj = np.array([
            [-half, -half, 0], [half, -half, 0],
            [half, half, 0], [-half, half, 0]
        ], dtype=np.float32)
        img = corners[int(matches[0])].reshape(4, 2).astype(np.float32)
        return self._solve_pnp(obj, img, K, D)

    def _detect_charuco(self, image, K, D):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if self.charuco_detector is not None:
            corners, ids, _, _ = self.charuco_detector.detectBoard(gray, K, D)
        else:
            marker_corners, marker_ids, _ = self._detect_markers(gray)
            if marker_ids is None:
                return None
            _, corners, ids = cv2.aruco.interpolateCornersCharuco(
                marker_corners, marker_ids, gray, self.charuco_board,
                cameraMatrix=K, distCoeffs=D
            )
        if corners is None or ids is None or len(corners) < 4:
            return None
        board_corners = (
            self.charuco_board.getChessboardCorners()
            if hasattr(self.charuco_board, "getChessboardCorners")
            else self.charuco_board.chessboardCorners
        )
        obj = np.asarray(board_corners, dtype=np.float32)[ids.reshape(-1).astype(int)]
        img = np.asarray(corners, dtype=np.float32).reshape(-1, 2)
        return self._solve_pnp(obj, img, K, D)

    def _detect_chessboard(self, image, K, D):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        pattern = (self.chessboard_cols, self.chessboard_rows)
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        found, corners = cv2.findChessboardCorners(gray, pattern, flags)
        if not found:
            return None
        corners = cv2.cornerSubPix(
            gray, corners, (5, 5), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
        )
        obj = np.zeros((self.chessboard_rows * self.chessboard_cols, 3), np.float32)
        obj[:, :2] = np.mgrid[0:self.chessboard_cols, 0:self.chessboard_rows].T.reshape(-1, 2)
        obj *= self.charuco_square_length
        return self._solve_pnp(obj, corners.reshape(-1, 2), K, D)

    @staticmethod
    def _solve_pnp(obj, img, K, D):
        ok, rvec, tvec = cv2.solvePnP(obj, img, K, D)
        if not ok:
            return None
        T = np.eye(4)
        T[:3, :3] = cv2.Rodrigues(rvec)[0]
        T[:3, 3] = tvec.reshape(3)
        return T

    def draw_detection(self, image, K, D):
        result = image.copy()
        corners, ids, _ = self._detect_markers(
            cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        )
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(result, corners, ids)
        T = self.detect(image, K, D)
        if T is not None:
            cv2.drawFrameAxes(
                result, K, D, cv2.Rodrigues(T[:3, :3])[0], T[:3, 3], 0.05
            )
        return result
