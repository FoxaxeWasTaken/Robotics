"""The calibrate_intrinsics entry point (6.2b, #67) against a fake camera in this process: synthetic images of
the ChArUco board, each held still a moment, then moved."""

import itertools

import cv2
import numpy as np
import yaml
from fake_camera import H, K, W, board_view, fake_camera

from so101_perception.calibrate_intrinsics import main
from so101_perception.intrinsics import MIN_VIEWS


def pose(rvec, x, y, nudge=0.0):
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(np.array(rvec))[0]
    T[:3, 3] = [x + nudge, y, 0.5]
    return T


def held_still(rvec, x, y):
    """The same view twice, 0.3 px apart: a hand holding the board, never two identical frames."""
    return [board_view(pose(rvec, x, y)), board_view(pose(rvec, x, y, nudge=0.0006))]


def views():
    """MIN_VIEWS views all over the image, each tilted, each frame shown a few times: the same spot tilted
    is no new view (MIN_SHIFT_PX)."""
    spots = itertools.product((-0.20, -0.15, -0.10, -0.05), (-0.14, -0.09, -0.04))
    tilts = itertools.cycle([(0.35, 0.0, 0.0), (-0.35, 0.0, 0.0), (0.0, 0.35, 0.0), (0.0, -0.35, 0.0)])
    frames = []
    for (x, y), rvec in zip(spots, tilts, strict=False):  # tilts cycles forever
        frames += held_still(rvec, x, y) * 3
    return frames


def run(tmp_path, timeout):
    out = tmp_path / "intrinsics.yaml"
    return main(
        ["calibrate_intrinsics", "--out", str(out), "--views", str(MIN_VIEWS), "--timeout", timeout]
    ), out


def test_the_tool_writes_k_and_d(tmp_path):
    frames = itertools.cycle(views())
    with fake_camera(lambda: next(frames)):
        code, out = run(tmp_path, "60")
    assert code == 0
    calibration = yaml.safe_load(out.read_text())
    assert (calibration["image_width"], calibration["image_height"]) == (W, H)
    fitted = np.reshape(calibration["camera_matrix"]["data"], (3, 3))
    assert np.abs(fitted - K).max() < 10.0  # px
    assert np.abs(calibration["distortion_coefficients"]["data"]).max() < 0.1


def test_one_view_held_still_is_refused_and_nothing_written(tmp_path):
    frames = itertools.cycle(held_still((0.3, 0.0, 0.0), -0.13, -0.09))
    with fake_camera(lambda: next(frames)):
        code, out = run(tmp_path, "3")
    assert code == 1
    assert not out.exists()


def test_no_camera_exits_1_after_the_timeout(tmp_path):
    code, out = run(tmp_path, "1")
    assert code == 1
    assert not out.exists()
