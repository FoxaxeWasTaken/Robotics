"""The corner convention of the calibration markers (05 §5), shared by the sim scene and the calibration."""

import numpy as np

from so101_common.markers import marker_corners


def test_corners_come_in_the_order_cv2_aruco_detects_them():
    """Seen from above, the marker image's up is +x and its right -y: top-left, top-right, bottom-right,
    bottom-left of the image."""
    corners = marker_corners([0.3, -0.1], 0.08)
    expected = [[0.34, -0.06, 0.0], [0.34, -0.14, 0.0], [0.26, -0.14, 0.0], [0.26, -0.06, 0.0]]
    np.testing.assert_allclose(corners, expected)


def test_the_corners_span_the_side_and_lie_on_the_table():
    corners = marker_corners([0.0, 0.0], 0.075)
    assert np.allclose(np.linalg.norm(np.diff(corners, axis=0, append=corners[:1]), axis=1), 0.075)
    assert not corners[:, 2].any()
