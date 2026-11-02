"""calibration.py - the camera pose from ArUco markers at known positions in base_link (docs/project/05 §5,
ADR-013). Pure: no ROS.

The layout is a markers file (so101_bringup/config/markers_sim.yaml in sim): dictionary, side_m, the
centers in base_link, and max_reprojection_px, the quality gate of that setup (1.5 px in sim, 2 px on the real
jig).
"""

import cv2
import numpy as np

from so101_common.markers import marker_corners

MIN_MARKERS = 4  # one marker gives an ambiguous pose (MES-16: 1 m and 86° off)


class CalibrationError(ValueError):
    """The markers seen do not give a trustworthy pose."""


def detect_markers(rgb, dictionary) -> dict:
    """{id: its 4 corners (px), in cv2.aruco's order} of the markers of that dictionary in an rgb8 image."""
    aruco = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    params = cv2.aruco.DetectorParameters()
    # sub-pixel corners: without, 1.17 px of mean reprojection on the sim scene, above its 1 px gate (#45)
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    corners, ids, _ = cv2.aruco.ArucoDetector(aruco, params).detectMarkers(gray)
    return {} if ids is None else {int(i): c[0] for i, c in zip(ids.ravel(), corners, strict=True)}


def solve(frames, layout, K, dist=None):
    """(T_base_cam, mean reprojection error in px, the marker ids used) from the detections of one or more
    frames of a still camera; CalibrationError under MIN_MARKERS markers of the layout or above its gate."""
    seen = [(i, corners) for frame in frames for i, corners in frame.items() if i in layout["markers"]]
    ids = sorted({i for i, _ in seen})
    if len(ids) < MIN_MARKERS:
        raise CalibrationError(f"{len(ids)} markers of the layout seen {ids}, at least {MIN_MARKERS} needed")
    world = np.vstack([marker_corners(layout["markers"][i], layout["side_m"]) for i, _ in seen])
    image = np.vstack([corners for _, corners in seen]).astype(float)
    _, rvec, tvec = cv2.solvePnP(world, image, K, dist, flags=cv2.SOLVEPNP_SQPNP)
    rvec, tvec = cv2.solvePnPRefineLM(world, image, K, dist, rvec, tvec)
    error = float(
        np.linalg.norm(cv2.projectPoints(world, rvec, tvec, K, dist)[0][:, 0] - image, axis=1).mean()
    )
    if error >= layout["max_reprojection_px"]:
        raise CalibrationError(
            f"mean reprojection error {error:.2f} px, above {layout['max_reprojection_px']} px"
        )
    R_cam_base = cv2.Rodrigues(rvec)[0]
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R_cam_base.T, -R_cam_base.T @ tvec.ravel()
    return T, error, ids
