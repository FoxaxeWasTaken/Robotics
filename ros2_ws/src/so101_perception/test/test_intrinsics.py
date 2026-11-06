"""The intrinsic calibration (6.2b, #67): the board's detection, the choice of views, K and D from known
views."""

import cv2
import numpy as np
import pytest
import yaml

from so101_perception.calibration import CalibrationError
from so101_perception.geometry import undistorted
from so101_perception.intrinsics import (
    MIN_VIEWS,
    BoardDetector,
    ViewPicker,
    board,
    calibrate,
    save_camera_info,
)

SIZE = (640, 480)
K = np.array([[520.0, 0, 322.0], [0, 521.0, 238.0], [0, 0, 1]])
D = np.array([-0.31, 0.12, 0.001, -0.0015, 0.0])


def views(n, noise=0.0, seed=0):
    """n views of the board's corners through K and D, tilted and spread over the image."""
    rng = np.random.default_rng(seed)
    corners = board().getChessboardCorners()
    middle = corners.mean(0)
    out = []
    for _ in range(n):
        rvec = rng.uniform(-0.6, 0.6, 3)
        tvec = np.array([rng.uniform(-0.05, 0.05), rng.uniform(-0.04, 0.04), rng.uniform(0.3, 0.5)])
        px = cv2.projectPoints(corners - middle, rvec, tvec, K, D)[0].reshape(-1, 2)
        px += rng.normal(0, noise, px.shape) if noise else 0.0
        out.append(dict(enumerate(px.astype(np.float32))))
    return out


def test_k_and_d_come_back_from_known_views():
    k, d, rms = calibrate(views(20, noise=0.1), SIZE)
    assert rms < 0.2
    assert np.abs(k - K).max() < 6.0  # px
    assert np.abs(d[:2] - D[:2]).max() < 0.02
    assert d[4] == 0.0  # k3 stays out


def test_too_few_views_or_a_high_rms_is_refused():
    with pytest.raises(CalibrationError, match="views"):
        calibrate(views(MIN_VIEWS - 1), SIZE)
    with pytest.raises(CalibrationError, match="RMS"):
        calibrate(views(20, noise=3.0), SIZE)


def test_the_board_is_found_in_an_image():
    side = 40  # px per 38 mm square
    w, h = board().getChessboardSize()
    image = board().generateImage((w * side, h * side), marginSize=0, borderBits=1)
    canvas = np.full((SIZE[1], SIZE[0]), 255, np.uint8)
    canvas[40 : 40 + image.shape[0], 30 : 30 + image.shape[1]] = image
    corners = BoardDetector().corners(cv2.cvtColor(canvas, cv2.COLOR_GRAY2RGB))
    assert len(corners) == (w - 1) * (h - 1)


def corners_at(x):
    return {i: np.float32([x + 10.0 * i, 100.0]) for i in range(10)}


def test_a_view_counts_once_still_and_new():
    picker = ViewPicker()
    assert not picker.offer(corners_at(0.0))  # no frame before it: unknown motion
    assert not picker.offer(corners_at(0.0))  # the driver's repeated frame says nothing
    assert picker.offer(corners_at(0.5))  # still
    assert not picker.offer(corners_at(1.0))  # still, but no new view
    assert not picker.offer(corners_at(60.0))  # moving
    assert picker.offer(corners_at(60.5))
    assert len(picker.views) == 2


def test_too_few_corners_do_not_count():
    picker = ViewPicker()
    few = {i: np.float32([i * 10.0, 0.0]) for i in range(3)}
    picker.offer(few)
    assert not picker.offer({i: p + 0.5 for i, p in few.items()})


def test_the_file_is_the_ros_calibration_format(tmp_path):
    path = tmp_path / "intrinsics.yaml"
    save_camera_info(path, K, D, SIZE, 0.3)
    calib = yaml.safe_load(path.read_text())
    assert (calib["image_width"], calib["image_height"]) == SIZE
    assert calib["camera_matrix"]["data"] == K.ravel().tolist()
    assert calib["distortion_coefficients"]["data"] == D.tolist()
    assert len(calib["rectification_matrix"]["data"]) == 9 and len(calib["projection_matrix"]["data"]) == 12


def test_undistorted_straightens_the_lens_and_leaves_the_sim_alone():
    image = np.zeros((*SIZE[::-1], 3), np.uint8)
    assert undistorted(image, K, np.zeros(5)) is image
    assert undistorted(image, K, np.array([])) is image
    ideal = np.array([[[500.0, 400.0]]])
    normalized = cv2.undistortPoints(ideal, K, None)
    seen = cv2.projectPoints(np.column_stack([normalized[0], [1.0]]), np.zeros(3), np.zeros(3), K, D)[0][0, 0]
    cv2.circle(image, (round(seen[0]), round(seen[1])), 3, (255, 255, 255), -1)
    ys, xs = np.nonzero(undistorted(image, K, D)[:, :, 0] > 128)
    assert abs(xs.mean() - 500.0) < 2.0 and abs(ys.mean() - 400.0) < 2.0
