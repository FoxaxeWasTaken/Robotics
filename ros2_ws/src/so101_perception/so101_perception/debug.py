"""Debug image of /perception/debug_image (docs/project/05 §9). Pure: no ROS.

The masks (cube green, box blue), the centroids and the cube status, on a copy of the rgb8 image.
"""

import cv2
import numpy as np

OK, TRUNCATED, LOST = "OK", "TRUNCATED", "LOST"
RED, GREEN, BLUE = (255, 0, 0), (0, 255, 0), (0, 0, 255)  # rgb8


def status(cube) -> str:
    """The cube's status: LOST (no valid blob), TRUNCATED (cut by the border, not published) or OK."""
    return LOST if cube is None else TRUNCATED if cube.truncated else OK


def annotate(rgb: np.ndarray, text: str, detection=None) -> np.ndarray:
    """Copy of the image with the detection's masks and centroids, and the text in its top-left corner."""
    out = rgb.copy()
    if detection is not None:
        for mask, blob, color in (
            (detection.cube_mask, detection.cube, GREEN),
            (detection.box_mask, detection.box, BLUE),
        ):
            out[mask > 0] = color
            if blob is not None:
                cv2.drawMarker(out, (round(blob.u), round(blob.v)), RED, cv2.MARKER_CROSS, 9, 1)
    for color, thickness in (((0, 0, 0), 3), (RED, 1)):
        cv2.putText(out, text, (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, thickness, cv2.LINE_AA)
    return out
