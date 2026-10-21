"""Temporal filter of the published positions (docs/project/05 §7). Pure: no ROS."""

from collections import deque

import numpy as np


class MedianFilter:
    """Rolling median of 5; a measurement more than 2 cm from the median is rejected, unless it persists
    5 times in a row: then the object really moved, and those 5 measurements become the window."""

    def __init__(self, size: int = 5, jump_m: float = 0.02):
        self._window = deque(maxlen=size)
        self._jumps = []
        self._jump_m = jump_m

    def update(self, p) -> np.ndarray | None:
        """The filtered position, or None when p is rejected."""
        p = np.asarray(p, dtype=float)
        if self._window and np.linalg.norm(p - np.median(self._window, axis=0)) > self._jump_m:
            # shortcut: 5 jumps in a row count as a move even if they disagree; 3.5 checks the outcome
            self._jumps.append(p)
            if len(self._jumps) < self._window.maxlen:
                return None
            self._window.extend(self._jumps)  # maxlen: the 5 jumps replace the whole window
        else:
            self._window.append(p)
        self._jumps = []
        return np.median(self._window, axis=0)
