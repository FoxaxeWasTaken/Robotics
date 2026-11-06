"""Camera geometry: pixel <-> base_link on a horizontal plane, and the extrinsics file (docs/project/05
§3-§4, ADR-012). Pure: no ROS.

Camera frame: OpenCV optical, x right, y down, z forward (REP-103). R_wc, t_wc map it to base_link.
"""

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml

from so101_common.robot import BASE_FRAME, CAMERA_FRAME

FLIP = np.diag([1.0, -1.0, -1.0])  # MuJoCo camera (y up, looks along -z) -> OpenCV optical (05 §3.1)
CUBE_CORNERS = 0.5 * np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])
YAWS = np.radians(np.arange(0.0, 90.0, 5.0))
HEADER = (
    "# Pose of the external camera in base_link (docs/project/05 §3-§5), written by\n"
    "# so101_perception.geometry.save_extrinsics. T_base_cam maps a point of the camera's OpenCV optical\n"
    "# frame (x right, y down, z forward) to base_link.\n"
)


@dataclass(frozen=True)
class Extrinsics:
    T_base_cam: np.ndarray  # 4x4, camera (OpenCV optical) -> base_link
    method: str
    date: str
    reprojection_error_px: float | None = None


def from_mujoco(T_world_cam: np.ndarray) -> np.ndarray:
    """A MuJoCo camera pose -> the same camera in the OpenCV optical convention. MuJoCo world = base_link."""
    T = np.array(T_world_cam, dtype=float)
    T[:3, :3] = T[:3, :3] @ FLIP
    return T


def undistorted(image, K, D):
    """The image a pinhole of matrix K would see: all that follows assumes one (ADR-014). The sim has no D."""
    return cv2.undistort(image, K, D) if np.any(D) else image


def pixel_to_world(u, v, K, R_wc, t_wc, z0) -> np.ndarray | None:
    """The point of the plane z = z0 seen at pixel (u, v), or None when the ray does not reach it."""
    d = R_wc @ np.linalg.solve(K, [u, v, 1.0])  # ray direction in base_link
    if d[2] * (z0 - t_wc[2]) <= 0.0:  # parallel to the plane, or the plane behind the camera
        return None
    return t_wc + (z0 - t_wc[2]) / d[2] * d


def cube_yaw(contour, K, R_wc, t_wc, z_top=0.05) -> float:
    """The cube yaw about z in [-pi/4, pi/4) (90° symmetry): its outline (px) on the top-face plane, then the
    smallest rectangle around it (05 §4, ADR-012, MES-14)."""
    points = (pixel_to_world(u, v, K, R_wc, t_wc, z_top) for u, v in contour)
    xy = np.array([p[:2] for p in points if p is not None], np.float32)
    return (np.radians(cv2.minAreaRect(xy)[2]) + np.pi / 4) % (np.pi / 2) - np.pi / 4


def cube_area_ratio(contour, p, K, R_wc, t_wc, side=0.05) -> float:
    """The blob's pixels (its whole outline, as detection's CHAIN_APPROX_NONE gives it) against the smallest
    silhouette of a whole cube of `side` m centered at p, its back-projected point, over the yaws (90°
    symmetry): at least about 1 for a whole cube, less for a partly hidden one, whose center is then pulled
    towards its visible part (#167, 05 §2.3)."""
    silhouettes = []
    for yaw in YAWS:
        c, s = np.cos(yaw), np.sin(yaw)
        corners = (
            np.asarray(p, dtype=float) + side * CUBE_CORNERS @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]).T
        )
        x = (corners - t_wc) @ R_wc @ K.T  # in the optical frame, then pixels
        silhouettes.append(cv2.contourArea(cv2.convexHull(np.float32(x[:, :2] / x[:, 2:]))))
    contour = np.asarray(contour, np.float32)
    pixels = cv2.contourArea(contour) + len(contour) / 2 + 1  # Pick: the outline runs through pixel centers
    return pixels / min(silhouettes)


def world_to_pixel(p, K, R_wc, t_wc) -> np.ndarray:
    """Pixel (u, v) of a base_link point in front of the camera."""
    x = K @ (R_wc.T @ (np.asarray(p, dtype=float) - t_wc))
    return x[:2] / x[2]


def quaternion(R) -> np.ndarray:
    """(x, y, z, w) of a rotation matrix, through its rotation vector (cv2 handles angles near 180°)."""
    r = cv2.Rodrigues(np.asarray(R, dtype=float))[0].ravel()
    angle = np.linalg.norm(r)
    return np.r_[r * (np.sin(angle / 2) / angle if angle > 1e-12 else 0.5), np.cos(angle / 2)]


def save_extrinsics(path, extrinsics: Extrinsics):
    data = {
        "T_base_cam": np.asarray(extrinsics.T_base_cam, dtype=float).tolist(),
        "parent_frame": BASE_FRAME,
        "child_frame": CAMERA_FRAME,
        "method": extrinsics.method,
        "date": extrinsics.date,
        "reprojection_error_px": extrinsics.reprojection_error_px,
    }
    Path(path).write_text(HEADER + yaml.safe_dump(data, sort_keys=False, default_flow_style=None))


def load_extrinsics(path) -> Extrinsics:
    """The file of save_extrinsics; ValueError when it is not a rigid camera pose in base_link."""
    data = yaml.safe_load(Path(path).read_text())
    T = np.array(data["T_base_cam"], dtype=float)
    if (data["parent_frame"], data["child_frame"]) != (BASE_FRAME, CAMERA_FRAME):
        raise ValueError(f"{path}: frames {data['parent_frame']} -> {data['child_frame']}, not the camera")
    rigid = (
        T.shape == (4, 4)
        and np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-6)
        and np.linalg.det(T[:3, :3]) > 0.0
    )
    if not rigid:
        raise ValueError(f"{path}: T_base_cam is not a rigid transform")
    return Extrinsics(T, data["method"], str(data["date"]), data["reprojection_error_px"])
