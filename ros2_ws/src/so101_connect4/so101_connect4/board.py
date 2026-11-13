"""board.py - reading the Connect Four grid in an image (docs/project/phase2/P2-02 §3). Pure: numpy and cv2,
the grid's geometry from layout.py. No ROS.

The grid's pose comes from its markers (grid_in_camera); the 42 cell centres are projected on the tokens'
face, and a cell is a token when enough of a disc around its centre lies in that color's HSV range. Images
are rgb8.
Cells are row-major from the bottom row, cells[row * 7 + column], column 0 nearest the arm: 0 empty, 1 the
human's (red), 2 the robot's (yellow), as so101_interfaces/msg/Connect4Board.
"""

from itertools import pairwise

import cv2
import numpy as np

from so101_connect4 import layout as geometry
from so101_connect4.game import EMPTY, HUMAN, ROBOT  # the cells' values, one definition

# OpenCV HSV ranges, hue 0 to 180. Red is the cube's of so101_perception's perception.yaml, unchanged; with
# this yellow, 2100 cells of 2100 [MES-65].
RED = (((160, 150, 30), (180, 255, 255)), ((0, 150, 30), (8, 255, 255)))
YELLOW = (((18, 120, 30), (40, 255, 255)),)
THRESHOLD = 0.35  # of the disc in the color's range: a token [MES-65]
DISC = 0.3  # the disc's radius, of the column pitch: inside a window, whatever the token's place in its cell
MIN_MARKERS = 3  # of the grid's 4: one may hide behind a hand


def grid_in_camera(found, layout, K, dist=None):
    """(T_cam_grid, mean reprojection error in px, the marker ids used) from detected marker corners
    ({id: 4 corners in px}, cv2.aruco's order); None with fewer than MIN_MARKERS of the grid's markers."""
    ids = [i for i in layout["markers"]["ids"] if i in found]
    if len(ids) < MIN_MARKERS:
        return None
    points = np.vstack([geometry.marker_corners(layout, i) for i in ids])
    pixels = np.vstack([found[i] for i in ids]).astype(float)
    _, rvec, tvec = cv2.solvePnP(points, pixels, K, dist, flags=cv2.SOLVEPNP_SQPNP)
    rvec, tvec = cv2.solvePnPRefineLM(points, pixels, K, dist, rvec, tvec)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = cv2.Rodrigues(rvec)[0], tvec.ravel()
    projected = cv2.projectPoints(points, rvec, tvec, K, dist)[0][:, 0]
    return T, float(np.linalg.norm(projected - pixels, axis=1).mean()), ids


def cell_pixels(layout, T_cam_grid, K):
    """(the 42 cell centres in px, on the tokens' face; the sampling disc's radius in px)."""
    grid = layout["grid"]
    cells = [(column, row) for row in range(grid["rows"]) for column in range(grid["columns"])]
    points = np.array([geometry.cell(layout, column, row) for column, row in cells])
    points[:, 1] = layout["token"]["thickness"] / 2  # the tokens' face, not the slot's mid-plane
    camera = points @ T_cam_grid[:3, :3].T + T_cam_grid[:3, 3]
    pixels = camera @ K.T
    radius = DISC * grid["column_pitch"] * K[0, 0] / camera[:, 2].mean()
    return pixels[:, :2] / pixels[:, 2:], float(radius)


def _in_ranges(hsv, ranges):
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for low, high in ranges:
        mask |= cv2.inRange(hsv, low, high)
    return mask > 0


def read(rgb, layout, T_cam_grid, K, red=RED, yellow=YELLOW, threshold=THRESHOLD):
    """The 42 cells of an rgb8 image, or None when a cell lies outside it."""
    centres, radius = cell_pixels(layout, T_cam_grid, K)
    height, width = rgb.shape[:2]
    inside = (centres >= radius) & (centres < np.array([width, height]) - radius)
    if not inside.all():
        return None
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    colors = {HUMAN: _in_ranges(hsv, red), ROBOT: _in_ranges(hsv, yellow)}
    cells = []
    for u, v in centres:
        disc = np.zeros((height, width), np.uint8)
        cv2.circle(disc, (round(u), round(v)), round(radius), 255, -1)
        disc = disc > 0
        fractions = {value: mask[disc].mean() for value, mask in colors.items()}
        best = max(fractions, key=fractions.get)
        cells.append(best if fractions[best] >= threshold else EMPTY)
    return tuple(cells)


def validate(cells, columns=7) -> tuple:
    """(True, "") for a board a game can show; else (False, why): `gravity:<column>` for a token over an
    empty cell, `counts:<human>-<robot>` when one side has 2 tokens more. Whose turn it is: turns.py."""
    for column in range(columns):
        stack = cells[column::columns]
        if any(above and not below for below, above in pairwise(stack)):
            return False, f"gravity:{column}"
    human, robot = cells.count(HUMAN), cells.count(ROBOT)
    if abs(human - robot) > 1:
        return False, f"counts:{human}-{robot}"
    return True, ""


def column_target(layout, T_base_grid, column) -> np.ndarray:
    """Where /brain/drop puts the held token's centre to play `column`, in base_link: above the depth guide by
    the drop's clearance, aimed off the slot plane towards the fixed jaw (P2-02 §2.2, §2.3)."""
    grid, drop = layout["grid"], layout["drop"]
    entry = (grid["rows"] - 0.5) * grid["row_pitch"] + grid["headroom"]
    height = entry + grid["depth_guide_height"] + drop["clearance"] + layout["token"]["diameter"] / 2
    return (T_base_grid @ [column * grid["column_pitch"], drop["aim"], height, 1.0])[:3]


def transform(position, quaternion) -> np.ndarray:
    """The 4x4 transform of a pose message: its position, its quaternion x, y, z, w (/connect4/grid_pose)."""
    x, y, z, w = quaternion
    T = np.eye(4)
    T[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    T[:3, 3] = position
    return T


class Stable:
    """Lets a read through once `reads` identical ones came in a row, each on a new image: a token in flight,
    a hand over the grid or one bad frame never reach the game. Its stamp is the first image's of the streak:
    when the board was first seen so, often before the move that the game waits for has replied."""

    def __init__(self, reads=5):
        self._reads, self._last, self._count, self._first, self._stamp = reads, None, 0, None, None

    def update(self, stamp, read):
        """(the streak's first stamp, the read) once stable, else None. `stamp` is comparable (ns); an image
        already seen changes nothing."""
        if stamp == self._stamp:
            return None
        self._stamp = stamp
        if read != self._last:
            self._last, self._count, self._first = read, 0, stamp
        self._count += 1
        return (self._first, read) if self._count >= self._reads else None


def draw(rgb, layout, T_cam_grid, K, cells, text="", four=()):
    """The debug image: each cell's disc, filled in its color when read as a token, the cells of `four`
    (game.four) circled, and a status line."""
    out = rgb.copy()
    centres, radius = cell_pixels(layout, T_cam_grid, K)
    fill = {HUMAN: (255, 0, 0), ROBOT: (255, 255, 0)}
    for (u, v), cell in zip(centres, cells or [EMPTY] * len(centres), strict=True):
        centre = (round(u), round(v))
        if cell:
            cv2.circle(out, centre, round(radius), fill[cell], -1, cv2.LINE_AA)
        cv2.circle(out, centre, round(radius), (255, 255, 255), 1, cv2.LINE_AA)
    for u, v in (centres[i] for i in four):
        cv2.circle(out, (round(u), round(v)), round(radius) + 3, (0, 255, 0), 3, cv2.LINE_AA)
    return label(out, text) if text else out


def label(rgb, text, row=0):
    """A copy of the image with a status line at its top, or under it (`row` 1)."""
    out = rgb.copy()
    for color, thickness in (((0, 0, 0), 3), ((0, 255, 0), 1)):
        at = (6, 18 + 20 * row)
        cv2.putText(out, text, at, cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, thickness, cv2.LINE_AA)
    return out
