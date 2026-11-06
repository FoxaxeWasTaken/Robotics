"""The calibrate_touch entry point (6.3a, #68) against a fake camera in this process: synthetic images of the
ChArUco board from a known pose, and the touches the arm would read on it."""

import cv2
import numpy as np
from fake_camera import H, W, board_view, fake_camera

from so101_perception.calibrate_touch import main
from so101_perception.geometry import load_extrinsics
from so101_perception.intrinsics import board

T_CAM_BOARD = np.eye(4)  # the board fills the image, a little tilted
T_CAM_BOARD[:3, :3] = cv2.Rodrigues(np.array([0.25, -0.2, 0.1]))[0]
T_CAM_BOARD[:3, 3] = [-0.14, -0.09, 0.5]
T_BASE_BOARD = np.eye(4)  # flat on the table, its back on it: the camera looks down
T_BASE_BOARD[:3, :3] = np.diag([1.0, -1.0, -1.0])
T_BASE_BOARD[:3, 3] = [0.15, 0.1, 0.002]
T_TRUE = T_BASE_BOARD @ np.linalg.inv(T_CAM_BOARD)
CORNERS = (0, 5, 18, 23)


def touches():
    chessboard = board().getChessboardCorners()
    return {i: T_BASE_BOARD[:3, :3] @ chessboard[i] + T_BASE_BOARD[:3, 3] for i in CORNERS}


def run(tmp_path, read, timeout="20"):
    out = tmp_path / "extrinsics.yaml"
    args = ["calibrate_touch", "--out", str(out), "--timeout", timeout]
    for i, xyz in read.items():
        args += ["--touch", str(i), *(f"{v:.5f}" for v in xyz)]  # as tools/so101-arm touch prints them
    return main(args), out


def test_the_tool_writes_the_camera_pose(tmp_path):
    image = board_view(T_CAM_BOARD)
    with fake_camera(lambda: image):
        code, out = run(tmp_path, touches())
    assert code == 0
    extrinsics = load_extrinsics(out)
    assert np.linalg.norm(extrinsics.T_base_cam[:3, 3] - T_TRUE[:3, 3]) < 0.005
    angle = np.degrees(np.linalg.norm(cv2.Rodrigues(extrinsics.T_base_cam[:3, :3].T @ T_TRUE[:3, :3])[0]))
    assert angle < 0.5
    assert extrinsics.reprojection_error_px < 1.0
    assert "touched at [0, 5, 18, 23]" in extrinsics.method


def test_a_bad_touch_exits_1_and_writes_nothing(tmp_path):
    image = board_view(T_CAM_BOARD)
    read = touches()
    read[23] = read[23] + [0.03, 0.0, 0.0]
    with fake_camera(lambda: image):
        code, out = run(tmp_path, read)
    assert code == 1
    assert not out.exists()


def test_no_board_in_view_exits_1_and_writes_nothing(tmp_path):
    table = np.full((H, W, 3), 255, np.uint8)
    with fake_camera(lambda: table):
        code, out = run(tmp_path, touches())
    assert code == 1
    assert not out.exists()


def test_no_camera_exits_1_after_the_timeout(tmp_path):
    code, out = run(tmp_path, touches(), timeout="1")
    assert code == 1
    assert not out.exists()
