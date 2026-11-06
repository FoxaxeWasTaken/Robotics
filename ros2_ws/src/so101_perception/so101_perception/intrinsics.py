"""intrinsics.py - the camera's K and distortion from views of a ChArUco board (05 §3.2, ADR-014, 6.2b). Pure:
no ROS. The board is the one of tools/calibration_sheets.py: its square size is checked on the print."""

import cv2
import numpy as np
import yaml

from so101_perception.calibration import CalibrationError

SQUARES = (7, 5)  # columns, rows: fills an A4 sheet
SQUARE_M, MARKER_M = 0.038, 0.028
DICTIONARY = cv2.aruco.DICT_5X5_100  # never confused with the table's markers (DICT_4X4_50)
MIN_CORNERS = 8  # of 24 in a view: fewer say little about the lens
MIN_VIEWS = 12
MAX_RMS_PX = 0.5  # ADR-014
MIN_SHIFT_PX = 25.0  # a view this far from the kept ones adds something
MAX_MOTION_PX = 1.5  # between two frames: the board is held still, so no motion blur


def board():
    return cv2.aruco.CharucoBoard(SQUARES, SQUARE_M, MARKER_M, cv2.aruco.getPredefinedDictionary(DICTIONARY))


class BoardDetector:
    """The board's corners in an rgb8 image: {corner id: (u, v)}."""

    def __init__(self):
        self._detector = cv2.aruco.CharucoDetector(board())

    def corners(self, rgb) -> dict:
        ids_px, ids = self._detector.detectBoard(cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY))[:2]
        return {} if ids is None else dict(zip(ids.ravel().tolist(), ids_px.reshape(-1, 2), strict=True))


def mean_shift(a: dict, b: dict) -> float:
    """Mean distance in px between the corners two views share; infinite when they share none."""
    common = set(a) & set(b)
    return float(np.mean([np.linalg.norm(a[i] - b[i]) for i in common])) if common else float("inf")


class ViewPicker:
    """Keeps a view when the board is held still and it differs from those kept. The driver publishes its
    newest frame at 10 Hz, so a camera under that repeats frames: an identical view proves no stillness."""

    def __init__(self):
        self.views, self._last = [], {}

    def offer(self, corners: dict) -> bool:
        moved = mean_shift(corners, self._last)
        if moved == 0.0:
            return False
        self._last = corners
        if len(corners) < MIN_CORNERS or moved > MAX_MOTION_PX:
            return False
        if any(mean_shift(corners, kept) < MIN_SHIFT_PX for kept in self.views):
            return False
        self.views.append(corners)
        return True


def board_pose(corners, K, D):
    """(T_cam_board, mean reprojection in px) of a view ({corner id: (u, v)}), with the camera's K, D."""
    chessboard = board().getChessboardCorners()
    objects = np.float32([chessboard[i] for i in corners])
    images = np.float32([corners[i] for i in corners])
    _, rvec, tvec = cv2.solvePnP(objects, images, K, D)
    seen = cv2.projectPoints(objects, rvec, tvec, K, D)[0].reshape(-1, 2)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = cv2.Rodrigues(rvec)[0], tvec.ravel()
    return T, float(np.linalg.norm(seen - images, axis=1).mean())


def rigid_fit(points, targets):
    """(T, residuals in m): the rotation and translation that best bring points onto targets (Kabsch)."""
    a, b = np.asarray(points, float), np.asarray(targets, float)
    ca, cb = a.mean(0), b.mean(0)
    U, _, Vt = np.linalg.svd((a - ca).T @ (b - cb))
    flip = np.diag([1.0, 1.0, np.sign(np.linalg.det(Vt.T @ U.T))])  # a rotation, never a mirror
    T = np.eye(4)
    T[:3, :3] = Vt.T @ flip @ U.T
    T[:3, 3] = cb - T[:3, :3] @ ca
    return T, np.linalg.norm(a @ T[:3, :3].T + T[:3, 3] - b, axis=1)


def extrinsics_from_touches(T_cam_board, touches):
    """(T_base_cam, residual of each touch in m) from the board seen by the camera and touches {corner id: the
    TCP in base_link on that corner}: the arm's own frame, so it reaches where the camera says (#68)."""
    chessboard = board().getChessboardCorners()
    T_base_board, residuals = rigid_fit([chessboard[i] for i in touches], list(touches.values()))
    return T_base_board @ np.linalg.inv(T_cam_board), residuals


def calibrate(views, size):
    """(K, D, RMS in px) of views ({corner id: (u, v)}) of images of size (width, height); k3 stays 0: a
    webcam's few views would only fit noise. CalibrationError under MIN_VIEWS or above MAX_RMS_PX."""
    if len(views) < MIN_VIEWS:
        raise CalibrationError(f"{len(views)} views, {MIN_VIEWS} needed")
    chessboard = board().getChessboardCorners()
    objects = [np.float32([chessboard[i] for i in view]) for view in views]
    images = [np.float32([view[i] for i in view]) for view in views]
    rms, K, D, *_ = cv2.calibrateCamera(objects, images, size, None, None, flags=cv2.CALIB_FIX_K3)
    if rms > MAX_RMS_PX:
        raise CalibrationError(f"RMS {rms:.3f} px, {MAX_RMS_PX} at most")
    return K, D.ravel(), rms


def save_camera_info(path, K, D, size, rms):
    """The ROS calibration file the driver's camera_info_file reads (driver_node.camera_info_from_yaml)."""

    def matrix(m):
        return {"rows": m.shape[0], "cols": m.shape[1], "data": np.ravel(m).tolist()}

    calibration = {
        "image_width": size[0],
        "image_height": size[1],
        "camera_name": "external_cam",
        "camera_matrix": matrix(K),
        "distortion_model": "plumb_bob",
        "distortion_coefficients": matrix(np.reshape(D, (1, -1))),
        "rectification_matrix": matrix(np.eye(3)),
        "projection_matrix": matrix(np.column_stack([K, np.zeros(3)])),
    }
    header = f"# calibrate_intrinsics (05 §3.2, ADR-014): RMS {rms:.3f} px\n"
    path.write_text(header + yaml.safe_dump(calibration, sort_keys=False))
