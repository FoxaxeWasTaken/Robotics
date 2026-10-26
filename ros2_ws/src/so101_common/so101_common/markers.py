"""markers.py - the corner convention of the calibration markers (05 §5), shared by the sim scene that draws
them (so101_bringup.sim_scene) and the calibration that reads them (so101_perception.calibration)."""

import numpy as np


def marker_corners(center, side):
    """A marker's corners in base_link (on the table), in the order cv2.aruco detects them: seen from above,
    the marker image's up is +x and its right is -y."""
    x, y, h = center[0], center[1], side / 2
    return np.array([[x + h, y + h, 0.0], [x + h, y - h, 0.0], [x - h, y - h, 0.0], [x - h, y + h, 0.0]])
