"""placement.py - would the brain play where the camera sees the grid and the dispenser? (#260,
docs/project/phase2/P2-02 §2.5). Pure, no ROS.

Where they stand is measured, never given: a grid set down too near, too far or not edge-on is found before
a game starts, not by its first move. The brain knows the arm, so the brain judges: one check_only request
of /brain/drop per column and of /brain/pick per station, at the targets of board.py. The game node asks
them one at a time (each holds the brain's executor for the time of its IK), only while nothing moves, and
again once the grid or the dispenser has moved.
"""

import numpy as np

from so101_connect4 import board

# Further than this from where it was judged, a pose is judged again: above the camera's noise from one
# image to the next (0.7 mm [MES-77]), under the 5 mm the brain keeps between the arm and the grid [MES-78]
MOVED_M = 0.005
MOVED_RAD = np.radians(2.0)
CHECKING = "checking where the grid stands"
BUSY = "busy"  # the brain's refusal of a check while it moves the arm
NAMES = {"drop": "column", "pick": "station"}


def _moved(frames, judged) -> bool:
    for T, was in zip(frames, judged, strict=True):
        if T is None:
            continue
        turn = np.arccos(np.clip((np.trace(T[:3, :3].T @ was[:3, :3]) - 1.0) / 2.0, -1.0, 1.0))
        if np.linalg.norm(T[:3, 3] - was[:3, 3]) > MOVED_M or turn > MOVED_RAD:
            return True
    return False


class Placement:
    def __init__(self, layout, dispenser):
        """`dispenser`: the robot takes its tokens there, so its stations are checked too."""
        self._layout, self._dispenser = layout, dispenser
        self._judged = None  # the frames (grid, dispenser) of the last verdict
        self._verdict = ""
        self._asked = None  # the frames being checked
        self._todo, self._waiting, self._refusals = [], None, []

    @property
    def reason(self) -> str:
        """ "": a game can be played there. Else why not: CHECKING, or the brain's first refusal, as
        `column 7: unsafe_path:column`, the columns and the stations numbered as the console's keys."""
        return CHECKING if self._judged is None or self._asked is not None else self._verdict

    def step(self, grid, dispenser, reply, idle):
        """One tick. `grid`, `dispenser`: their frames in base_link as the camera sees them, or None;
        `reply`: (success, message) of the check asked last, once it came; `idle`: neither the arm nor the
        engine works. Returns the check to ask now, (drop or pick, the column or the rail, its target), or
        None."""
        frames = (grid, dispenser if self._dispenser else None)
        if grid is None or (self._dispenser and dispenser is None):
            return None
        if self._asked is not None and _moved(frames, self._asked):  # moved meanwhile: from the start
            self._asked = self._waiting = reply = None
        if self._waiting is not None and reply is not None:
            self._take(reply)
        if self._asked is None and not self._start(frames, idle):
            return None
        if self._waiting is not None or (self._todo and not idle):
            return None
        if not self._todo:  # every check answered: the verdict
            self._judged, self._asked = self._asked, None
            self._verdict = next(iter(self._refusals), "")  # the first refusal
            return None
        return self._next()

    def _next(self):
        """The next check to ask."""
        kind, index = self._waiting = self._todo.pop(0)
        if kind == "drop":
            return kind, index, board.column_target(self._layout, self._asked[0], index)
        return kind, index, board.station_target(self._layout, self._asked[1], index)

    def _take(self, reply):
        """The brain's reply to the check asked last."""
        kind, index = self._waiting
        if reply == (False, BUSY):  # the brain moves the arm: no verdict, the same check again
            self._todo.insert(0, self._waiting)
        elif not reply[0]:
            self._refusals.append(f"{NAMES[kind]} {index + 1}: {reply[1]}")
        self._waiting = None

    def _start(self, frames, idle) -> bool:
        """A new round of checks on `frames`, when one is due: never judged, or moved since."""
        if not idle or (self._judged is not None and not _moved(frames, self._judged)):
            return False
        self._asked, self._refusals = frames, []
        self._todo = [("drop", column) for column in range(self._layout["grid"]["columns"])]
        if self._dispenser:
            self._todo += [("pick", rail) for rail in range(self._layout["dispenser"]["rails"])]
        return True
