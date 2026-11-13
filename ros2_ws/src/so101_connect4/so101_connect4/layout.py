"""layout.py - the geometry of config/connect4_grid.yaml (phase2/P2-02 §6, §7), shared by the sim scene that
builds the grid (so101_bringup.sim_scene) and by what reads it or aims at it. Pure, no ROS.

The grid frame (P2-02 §6): origin at the centre of cell (column 0, row 0) in the slot mid-plane, x outward
along the columns (column 0 nearest the arm), z up, y = z cross x: the front face, the camera's side.
"""

from pathlib import Path

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory

TOKEN_COLORS = ("red", "yellow")  # the store-bought game's: the two the reader has a range for (board.py)


def load(path=None) -> dict:
    """The installed layout, or the one of `path`."""
    path = path or Path(get_package_share_directory("so101_connect4")) / "config" / "connect4_grid.yaml"
    layout = yaml.safe_load(Path(path).read_text())
    colors(layout)  # a robot_color the reader has no range for is refused here, before anything starts
    return layout


def colors(layout) -> dict:
    """{1: the color of the person's tokens, 2: of the robot's} (Connect4Board's cells): token.robot_color,
    and the person plays the other. The one place that pairs a side with a color (#255)."""
    robot = layout["token"]["robot_color"]
    if robot not in TOKEN_COLORS:
        raise ValueError(f"connect4_grid.yaml: token.robot_color is {robot!r}, it is red or yellow")
    return {1: next(color for color in TOKEN_COLORS if color != robot), 2: robot}


def slot_bottom(layout) -> float:
    """Height of the slots' floor above the table (m): a full column ends `headroom` under the entry."""
    grid = layout["grid"]
    return grid["slot_entry_height"] - grid["rows"] * grid["row_pitch"] - grid["headroom"]


def grid_pose(layout) -> np.ndarray:
    """T_base_grid (4x4) where the layout puts the grid: the scene builds it there, the reader measures it."""
    grid, place = layout["grid"], layout["placement"]
    a = place["azimuth"]
    T = np.eye(4)
    T[:3, 0], T[:3, 1] = [np.cos(a), np.sin(a), 0.0], [-np.sin(a), np.cos(a), 0.0]
    T[:2, 3] = place["pan_axis"]
    T[2, 3] = -place["riser"] + slot_bottom(layout) + grid["row_pitch"] / 2
    T[:3, 3] += place["first_column_r"] * T[:3, 0] - place["plane_offset"] * T[:3, 1]
    return T


def cell(layout, column, row) -> np.ndarray:
    """Centre of a cell in the grid frame; row `rows` is the slot entry's level, half a row too high."""
    return np.array([column * layout["grid"]["column_pitch"], 0.0, row * layout["grid"]["row_pitch"]])


def marker_centers(layout) -> dict:
    """{id: centre in the grid frame} of the 4 markers: on the front face of the side posts, one at the top
    and one at the bottom of each."""
    grid, markers = layout["grid"], layout["markers"]
    y = grid["slot_gap"] / 2 + grid["plate"] + 0.0004  # just off the face: no z-fighting
    near = -(grid["column_pitch"] + grid["frame_margin"]) / 2
    far = (grid["columns"] - 0.5) * grid["column_pitch"] + grid["frame_margin"] / 2
    edge = markers["plate"] / 2 + 0.002
    top = grid["rows"] * grid["row_pitch"] + grid["headroom"] - grid["row_pitch"] / 2 - edge
    bottom = -grid["row_pitch"] / 2 - slot_bottom(layout) + edge
    xz = [(near, top), (far, top), (near, bottom), (far, bottom)]
    return {i: np.array([x, y, z]) for i, (x, z) in zip(markers["ids"], xz, strict=True)}


def marker_corners(layout, i) -> np.ndarray:
    """A marker's 4 corners in the grid frame, in the order cv2.aruco detects them: seen from the front, the
    marker image's right is -x and its up is +z."""
    center, h = marker_centers(layout)[i], layout["markers"]["size"] / 2
    right, up = np.array([-h, 0.0, 0.0]), np.array([0.0, 0.0, h])
    return np.array([center - right + up, center + right + up, center + right - up, center - right - up])
