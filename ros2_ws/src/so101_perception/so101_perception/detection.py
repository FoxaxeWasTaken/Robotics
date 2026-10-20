"""HSV detection of the red cube and the brown drop box (docs/project/05 §2, ADR-011). Pure: no ROS.

The input is rgb8, the driver's encoding: COLOR_RGB2HSV, never BGR. OpenCV HSV: H in [0, 180], S, V in
[0, 255].
"""

from dataclasses import dataclass

import cv2
import numpy as np

HsvRange = tuple[tuple[int, int, int], tuple[int, int, int]]  # low, high
OPENING = np.ones((3, 3), np.uint8)


@dataclass(frozen=True)
class Thresholds:
    """05 §2.2 and §2.3. ROS parameters at 3.3b."""

    # Red wraps around H = 0/180: two ranges.
    cube: tuple[HsvRange, ...] = (((160, 100, 60), (180, 255, 255)), ((0, 100, 60), (8, 255, 255)))
    box: tuple[HsvRange, ...] = (((5, 90, 40), (25, 255, 220)),)
    min_area_px: float = 15.0  # at 320x240, the cube covers 300-600 px
    min_solidity: float = 0.6  # area / convex hull area: rejects reflections and fragments


DEFAULT = Thresholds()


@dataclass(frozen=True)
class Blob:
    u: float  # centroid (moments), px
    v: float
    truncated: bool  # the bounding box touches the image border: biased centroid (05 §6)


@dataclass(frozen=True)
class Detection:
    cube: Blob | None
    box: Blob | None
    cube_mask: np.ndarray  # uint8, 255 on the color, after the opening: for the debug image
    box_mask: np.ndarray


def detect(rgb: np.ndarray, thresholds: Thresholds = DEFAULT) -> Detection:
    """The largest valid blob of each color, or None."""
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    cube_mask, box_mask = color_mask(hsv, thresholds.cube), color_mask(hsv, thresholds.box)
    return Detection(largest(cube_mask, thresholds), largest(box_mask, thresholds), cube_mask, box_mask)


def color_mask(hsv: np.ndarray, ranges: tuple[HsvRange, ...]) -> np.ndarray:
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for low, high in ranges:
        mask |= cv2.inRange(hsv, low, high)
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, OPENING)


def largest(mask: np.ndarray, thresholds: Thresholds) -> Blob | None:
    """The largest external contour with the minimum area and solidity."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    valid = [
        c
        for c in contours
        if cv2.contourArea(c) >= thresholds.min_area_px
        and cv2.contourArea(c) > thresholds.min_solidity * cv2.contourArea(cv2.convexHull(c))
    ]
    if not valid:
        return None
    contour = max(valid, key=cv2.contourArea)
    m = cv2.moments(contour)
    x, y, w, h = cv2.boundingRect(contour)
    height, width = mask.shape
    truncated = x == 0 or y == 0 or x + w == width or y + h == height
    return Blob(m["m10"] / m["m00"], m["m01"] / m["m00"], truncated)
