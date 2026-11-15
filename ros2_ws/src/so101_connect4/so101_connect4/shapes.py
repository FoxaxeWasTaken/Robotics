"""shapes.py - what the game's RViz view draws beside the arm (#280): the grid, the tokens read in its cells,
the dispenser's rails and the token waiting at each station. Pure, no ROS: board_reader turns each shape into
a marker of /connect4/markers, in the frame it measures. Display only: nothing of it reaches the game.
"""

from typing import NamedTuple

import numpy as np

from so101_connect4 import layout as geometry
from so101_connect4.game import EMPTY, ROBOT

RGBA = {"red": (0.85, 0.1, 0.1, 1.0), "yellow": (0.95, 0.8, 0.1, 1.0)}
GRID_RGBA = (0.1, 0.3, 0.8, 0.35)  # translucent: the tokens stand inside it
RAIL_RGBA = (0.6, 0.6, 0.6, 1.0)
ON_EDGE = np.array(
    [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]]
)  # a disc's axis, its z, on the frame's y


class Shape(NamedTuple):
    ns: str  # with id, the shape's name from one call to the next: a token that is gone is the same, hidden
    id: int
    frame: str  # "grid" or "dispenser": the frames of layout.py
    kind: str  # "box", or "disc": a cylinder along its own z
    center: np.ndarray  # in `frame`, m
    R: np.ndarray  # the shape's axes in `frame`, 3x3
    size: np.ndarray  # along the shape's own axes, m
    color: tuple  # r, g, b, a
    shown: bool


def shapes(layout, cells, waiting) -> list:
    """Every shape of the view. `cells` as Connect4Board's (None: not read), `waiting` as /connect4/stations
    (None: the dispenser was not read)."""
    token, dispenser = layout["token"], layout["dispenser"]
    disc = np.array([token["diameter"], token["diameter"], token["thickness"]])
    colors = {side: RGBA[color] for side, color in geometry.colors(layout).items()}
    found = [
        Shape("grid", i, "grid", "box", (low + high) / 2, np.eye(3), high - low, GRID_RGBA, True)
        for i, (low, high) in enumerate(geometry.grid_boxes(layout))
    ]
    columns = layout["grid"]["columns"]
    for i, cell in enumerate(cells or [EMPTY] * (columns * layout["grid"]["rows"])):
        center = geometry.cell(layout, i % columns, i // columns)
        found.append(
            Shape(
                "tokens",
                i,
                "grid",
                "disc",
                center,
                ON_EDGE,
                disc,
                colors.get(cell, colors[ROBOT]),
                cell != EMPTY,
            )
        )
    along, across = geometry.rail_axes(layout)
    tilted = np.column_stack([along, [0.0, 1.0, 0.0], across])
    length = dispenser["tokens_per_rail"] * token["diameter"]
    floor = np.array([length, dispenser["slot_gap"] + 2 * dispenser["wall"], dispenser["floor"]])
    under = (length - token["diameter"]) / 2 * along - (token["diameter"] + floor[2]) / 2 * across
    for rail in range(dispenser["rails"]):
        station = geometry.station(layout, rail)
        there = bool(waiting[rail]) if waiting else False
        found.append(
            Shape("dispenser", rail, "dispenser", "box", station + under, tilted, floor, RAIL_RGBA, True)
        )
        found.append(
            Shape("stations", rail, "dispenser", "disc", station, ON_EDGE, disc, colors[ROBOT], there)
        )
    return found
