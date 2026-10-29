"""Debug image of /perception/debug_image (docs/project/05 §9): the status drawn on a copy of the image."""

import cv2
import numpy as np

from so101_perception.debug import LOST, OK, TRUNCATED, annotate, status
from so101_perception.detection import Blob, detect


def image():
    rng = np.random.default_rng(0)
    return rng.integers(0, 256, size=(240, 320, 3), dtype=np.uint8)


def test_the_status_is_drawn_in_the_top_left_corner_only():
    rgb = image()
    out = annotate(rgb, LOST)
    assert out.shape == rgb.shape
    assert out.dtype == np.uint8
    changed = np.any(out != rgb, axis=2)
    rows, cols = np.nonzero(changed)
    assert changed.sum() > 200, "no visible text"
    assert rows.max() < 30, "the text leaves the top-left corner"
    assert cols.max() < 160, "the text leaves the top-left corner"


def test_the_input_image_is_left_untouched():
    rgb = image()
    before = rgb.copy()
    annotate(rgb, LOST)
    np.testing.assert_array_equal(rgb, before)


def test_the_status_of_the_cube():
    assert status(None) == LOST
    assert status(Blob(1.0, 2.0, True)) == TRUNCATED
    assert status(Blob(1.0, 2.0, False)) == OK


def test_the_masks_are_painted_cube_green_box_blue():
    """05 §9: on a gray scene with a red square and a brown rectangle, away from the status text."""
    hsv = np.full((240, 320, 3), (0, 0, 128), np.uint8)
    hsv[100:120, 100:120] = (175, 230, 200)
    hsv[150:170, 200:240] = (12, 170, 90)
    rgb = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)
    found = detect(rgb)
    out = annotate(rgb, OK, found)
    cube, box = found.cube_mask > 0, found.box_mask > 0
    assert np.mean(np.all(out[cube] == (0, 255, 0), axis=1)) > 0.8
    assert np.mean(np.all(out[box] == (0, 0, 255), axis=1)) > 0.8
    assert np.any(out[110] != out[100]), "no centroid marker"
