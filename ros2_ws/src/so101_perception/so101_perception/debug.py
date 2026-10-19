"""Debug image of /perception/debug_image (docs/project/05 §9). Pure: no ROS.

Today the status only; the masks and centroids come with the detection (3.2b).
"""

import cv2
import numpy as np

NO_DETECTION = "NO DETECTION"
RED = (255, 0, 0)  # rgb8


def annotate(rgb: np.ndarray, status: str) -> np.ndarray:
    """Copy of the rgb8 image with the status in its top-left corner, red on a black outline."""
    out = rgb.copy()
    for color, thickness in (((0, 0, 0), 3), (RED, 1)):
        cv2.putText(out, status, (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, thickness, cv2.LINE_AA)
    return out
