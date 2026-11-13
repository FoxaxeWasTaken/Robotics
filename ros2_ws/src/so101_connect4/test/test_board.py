"""board.py (#81, phase2/P2-02 §3): the grid's pose from its markers, the 42 cells read in an rgb8 image, the
board's validation, the drop target of a column, the stable-read filter. On synthetic images drawn through a
known camera, and on reference images rendered from the game's scene (test/data, made by
docs/project/experiences/connect4/reading/render_reference.py). No ROS, no sim."""

import json
from pathlib import Path

import cv2
import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory

from so101_connect4 import board, layout
from so101_connect4.board import EMPTY, HUMAN, ROBOT
from so101_perception.calibration import detect_markers
from so101_perception.geometry import load_extrinsics, quaternion

LAYOUT = layout.load()
GRID = LAYOUT["grid"]
DATA = Path(__file__).parent / "data"
REFERENCE = json.loads((DATA / "reference.json").read_text())
EXTRINSICS = Path(get_package_share_directory("so101_perception")) / "config" / "camera_extrinsics.yaml"
K = np.array([[600.0, 0.0, 319.5], [0.0, 600.0, 239.5], [0.0, 0.0, 1.0]])
SIZE = (480, 640)
RED, YELLOW, BACKGROUND = (204, 13, 26), (242, 199, 13), (60, 60, 70)  # rgb


def camera_facing_the_grid(distance=0.45, side=0.05):
    """T_cam_grid of a camera in front of the grid (+y), a little to one side: x right, y down, z forward."""
    eye = np.array([3 * GRID["column_pitch"] + side, distance, 2.5 * GRID["row_pitch"]])
    forward = np.array([3 * GRID["column_pitch"], 0.0, 2.5 * GRID["row_pitch"]]) - eye
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, [0.0, 0.0, 1.0])
    right /= np.linalg.norm(right)
    T_grid_cam = np.eye(4)
    T_grid_cam[:3, :3] = np.column_stack([right, np.cross(forward, right), forward])
    T_grid_cam[:3, 3] = eye
    return np.linalg.inv(T_grid_cam)


T_CAM_GRID = camera_facing_the_grid()


def project(points):
    camera = np.atleast_2d(points) @ T_CAM_GRID[:3, :3].T + T_CAM_GRID[:3, 3]
    pixels = camera @ K.T
    return pixels[:, :2] / pixels[:, 2:]


def synthetic(cells):
    """An rgb8 image of that board: each token a disc of its color, as wide as a window."""
    image = np.full((*SIZE, 3), BACKGROUND, np.uint8)
    centres, radius = board.cell_pixels(LAYOUT, T_CAM_GRID, K)
    window = radius / board.DISC * GRID["window_diameter"] / 2 / GRID["column_pitch"]
    for (u, v), cell in zip(centres, cells, strict=True):
        if cell:
            cv2.circle(image, (round(u), round(v)), round(window), RED if cell == HUMAN else YELLOW, -1)
    return image


def random_board(rng, moves):
    cells, heights, player = [EMPTY] * 42, [0] * 7, HUMAN
    for _ in range(moves):
        column = int(rng.choice([c for c in range(7) if heights[c] < 6]))
        cells[heights[column] * 7 + column] = player
        heights[column] += 1
        player = HUMAN + ROBOT - player
    return tuple(cells)


def detected(ids):
    return {i: project(layout.marker_corners(LAYOUT, i)) for i in ids}


@pytest.mark.parametrize("ids", [(10, 11, 12, 13), (10, 11, 13), (11, 12, 13)])
def test_the_grids_pose_comes_from_four_markers_or_three(ids):
    T, error, used = board.grid_in_camera(detected(ids), LAYOUT, K)
    assert used == list(ids) and error < 0.01
    assert np.allclose(T[:3, 3], T_CAM_GRID[:3, 3], atol=1e-4)  # 0.1 mm
    assert np.allclose(T[:3, :3], T_CAM_GRID[:3, :3], atol=1e-4)


def test_two_markers_or_markers_of_another_set_give_no_pose():
    assert board.grid_in_camera(detected((10, 13)), LAYOUT, K) is None
    others = {i - 10: corners for i, corners in detected((10, 11, 12, 13)).items()}  # the calibration's ids
    assert board.grid_in_camera(others, LAYOUT, K) is None


def test_the_cells_are_row_major_from_the_bottom_column_0_on_the_viewers_right():
    centres, radius = board.cell_pixels(LAYOUT, T_CAM_GRID, K)
    assert centres.shape == (42, 2)
    assert centres[0][0] > centres[6][0]  # seen from the front, x of the grid goes to the left
    assert centres[0][1] > centres[35][1]  # row 0 is the lowest in the image
    pitch_px = np.linalg.norm(centres[0] - centres[6]) / 6  # the mean over the row: one radius for the grid
    assert radius == pytest.approx(board.DISC * pitch_px, rel=0.05)
    assert radius < pitch_px * GRID["window_diameter"] / 2 / GRID["column_pitch"]  # inside a window


def test_every_cell_of_100_random_boards_is_read():
    rng = np.random.default_rng(81)
    for _ in range(100):
        cells = random_board(rng, int(rng.integers(0, 43)))
        assert board.read(synthetic(cells), LAYOUT, T_CAM_GRID, K) == cells


def test_an_image_is_rgb8_a_bgr_one_reads_the_humans_tokens_as_nothing():
    cells = random_board(np.random.default_rng(1), 20)
    bgr = cv2.cvtColor(synthetic(cells), cv2.COLOR_RGB2BGR)
    read = board.read(bgr, LAYOUT, T_CAM_GRID, K)
    assert HUMAN not in read and read != cells


def test_a_grid_partly_out_of_the_image_is_not_read():
    shifted = T_CAM_GRID.copy()
    shifted[0, 3] += 0.15  # the grid 15 cm to the right of the camera's axis
    assert board.read(synthetic([EMPTY] * 42), LAYOUT, shifted, K) is None


def test_a_board_a_game_can_show_is_valid():
    rng = np.random.default_rng(3)
    for moves in (0, 1, 2, 17, 41, 42):
        assert board.validate(random_board(rng, moves)) == (True, "")


def test_a_token_over_an_empty_cell_is_refused_with_its_column():
    cells = [EMPTY] * 42
    cells[1 * 7 + 4] = HUMAN  # row 1 of column 4, nothing under it
    assert board.validate(tuple(cells)) == (False, "gravity:4")


def test_two_tokens_more_on_one_side_are_refused_with_the_counts():
    cells = [EMPTY] * 42
    cells[0], cells[1], cells[2] = HUMAN, HUMAN, ROBOT
    assert board.validate(tuple(cells)) == (True, "")
    cells[3] = HUMAN
    assert board.validate(tuple(cells)) == (False, "counts:3-1")


def test_the_drop_target_is_over_the_guide_above_the_column_1_mm_towards_the_fixed_jaw():
    T_base_grid = layout.grid_pose(LAYOUT)
    for column in (0, 3, 6):
        target = board.column_target(LAYOUT, T_base_grid, column)
        x, y = (np.linalg.inv(T_base_grid) @ [*target, 1.0])[:2]
        assert x == pytest.approx(column * GRID["column_pitch"]) and y == pytest.approx(0.001)
        # table 10 cm under base_link, entries at 25 cm, guide 30 mm, 8 mm clear, the token's radius
        assert target[2] == pytest.approx(-0.10 + 0.25 + 0.030 + 0.008 + 0.017)


def test_a_pose_message_gives_back_the_transform_it_was_made_from():
    """/connect4/grid_pose, as board_reader writes it: a position and a quaternion x, y, z, w."""
    T = np.eye(4)
    T[:3, :3], _ = cv2.Rodrigues(np.array([0.3, -1.1, 0.6]))
    T[:3, 3] = 0.2, -0.17, 0.11
    np.testing.assert_allclose(board.transform(T[:3, 3], quaternion(T[:3, :3])), T, atol=1e-12)


def test_a_read_is_stable_after_5_identical_ones_stamped_as_the_first():
    stable = board.Stable(reads=5)
    read = (random_board(np.random.default_rng(5), 9), True, "")
    assert [stable.update(stamp, read) for stamp in (10, 11, 12, 13)] == [None] * 4
    assert stable.update(14, read) == (10, read)
    assert stable.update(15, read) == (10, read)  # the same streak goes on


def test_an_image_already_seen_does_not_count():
    stable = board.Stable(reads=5)
    read = ((EMPTY,) * 42, True, "")
    for stamp in (10, 10, 10, 11, 11, 12, 13):
        assert stable.update(stamp, read) is None
    assert stable.update(14, read) == (10, read)


def test_one_different_read_starts_the_streak_again():
    stable = board.Stable(reads=5)
    empty, glitch = ((EMPTY,) * 42, True, ""), ((HUMAN,) + (EMPTY,) * 41, True, "")
    for stamp in (1, 2, 3, 4):
        stable.update(stamp, empty)
    assert stable.update(5, glitch) is None
    assert [stable.update(stamp, empty) for stamp in (6, 7, 8, 9)] == [None] * 4
    assert stable.update(10, empty) == (6, empty)


@pytest.mark.parametrize("name", sorted(REFERENCE["boards"]))
def test_every_cell_of_a_reference_image_of_the_scene_is_read(name):
    rgb = cv2.cvtColor(cv2.imread(str(DATA / f"{name}.png")), cv2.COLOR_BGR2RGB)
    camera = np.array(REFERENCE["K"])
    found = detect_markers(rgb, LAYOUT["markers"]["dictionary"])
    T_cam_grid, error, ids = board.grid_in_camera(found, LAYOUT, camera)
    assert ids == LAYOUT["markers"]["ids"] and error < 1.0
    cells = board.read(rgb, LAYOUT, T_cam_grid, camera)
    assert cells == tuple(REFERENCE["boards"][name])
    assert board.validate(cells) == (True, "")
    # the grid where the layout puts it, through our calibration of the phase-1 camera (05 §5)
    T_base_grid = load_extrinsics(EXTRINSICS).T_base_cam @ T_cam_grid
    assert np.linalg.norm(T_base_grid[:3, 3] - layout.grid_pose(LAYOUT)[:3, 3]) < 0.005


def test_the_debug_image_fills_the_cells_read_and_keeps_the_size():
    cells = random_board(np.random.default_rng(2), 12)
    image = synthetic([EMPTY] * 42)
    out = board.draw(image, LAYOUT, T_CAM_GRID, K, cells, "board VALID")
    assert out.shape == image.shape and out is not image
    centres, _ = board.cell_pixels(LAYOUT, T_CAM_GRID, K)
    for (u, v), cell in zip(centres, cells, strict=True):
        pixel = tuple(int(c) for c in out[round(v), round(u)])
        assert pixel == {EMPTY: BACKGROUND, HUMAN: (255, 0, 0), ROBOT: (255, 255, 0)}[cell]
