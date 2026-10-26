"""Extrinsic calibration from ArUco markers (3.4b, #45; 05 §5, ADR-013), on synthetic projections."""

import cv2
import numpy as np
import pytest

from so101_common.markers import marker_corners
from so101_perception.calibration import CalibrationError, solve

K = np.array([[500.0, 0, 319.5], [0, 500.0, 239.5], [0, 0, 1]])
LAYOUT = {
    "side_m": 0.075,
    "max_reprojection_px": 1.0,
    "markers": {
        0: [0.43, -0.01],
        1: [-0.35, 0.04],
        2: [0.18, -0.40],
        3: [-0.01, 0.19],
        4: [0.37, -0.26],
        5: [0.31, -0.51],
    },
}
# a camera 35° down on the work area, as sim_scene.yaml
T_TRUE = np.eye(4)
T_TRUE[:3, :3] = np.array([[-0.7660, 0.3687, -0.5266], [0.6428, 0.4394, -0.6275], [0.0, -0.8192, -0.5736]])
T_TRUE[:3, :3] = np.linalg.svd(T_TRUE[:3, :3])[0] @ np.linalg.svd(T_TRUE[:3, :3])[2]  # exactly orthonormal
T_TRUE[:3, 3] = [0.458, 0.339, 0.363]


def seen(ids, noise_px=0.0, seed=0):
    """The corners of those markers in the image of T_TRUE, as detect_markers gives them."""
    rng = np.random.default_rng(seed)
    R, t = T_TRUE[:3, :3], T_TRUE[:3, 3]
    frame = {}
    for i in ids:
        camera = (marker_corners(LAYOUT["markers"][i], LAYOUT["side_m"]) - t) @ R
        uv = (camera @ K.T)[:, :2] / camera[:, 2:]
        frame[i] = uv + rng.normal(0.0, noise_px, uv.shape)
    return frame


def test_a_known_pose_is_recovered_from_its_exact_projection():
    T, error, ids = solve([seen(LAYOUT["markers"])], LAYOUT, K)
    assert np.allclose(T, T_TRUE, atol=1e-6)
    assert error < 1e-6 and ids == sorted(LAYOUT["markers"])


def test_half_a_pixel_of_noise_on_6_spread_markers_stays_under_5_mm():
    frames = [seen(LAYOUT["markers"], noise_px=0.5, seed=k) for k in range(3)]
    T, error, _ = solve(frames, LAYOUT, K)
    assert np.linalg.norm(T[:3, 3] - T_TRUE[:3, 3]) < 0.005
    angle = np.degrees(np.linalg.norm(cv2.Rodrigues(T[:3, :3].T @ T_TRUE[:3, :3])[0]))
    assert angle < 0.3 and error < 1.0


@pytest.mark.parametrize("ids", [[0], [0, 2, 5]])
def test_fewer_than_4_markers_are_refused(ids):
    """A single marker gives an ambiguous pose (MES-16: 1 m off): refused, not "best effort"."""
    with pytest.raises(CalibrationError, match="markers"):
        solve([seen(ids)], LAYOUT, K)


def test_markers_not_in_the_layout_are_ignored():
    frame = seen([0, 1, 2]) | {42: seen([3])[3]}
    with pytest.raises(CalibrationError, match="markers"):
        solve([frame], LAYOUT, K)


def test_a_high_reprojection_error_is_refused():
    """Corners 3 px off on average (a moved marker, a wrong layout): above the 1 px of the sim."""
    with pytest.raises(CalibrationError, match="reprojection"):
        solve([seen(LAYOUT["markers"], noise_px=4.0)], LAYOUT, K)
