"""Debug image of /perception/debug_image (docs/project/05 §9): the status drawn on a copy of the image."""

import numpy as np

from so101_perception.debug import NO_DETECTION, annotate


def image():
    rng = np.random.default_rng(0)
    return rng.integers(0, 256, size=(240, 320, 3), dtype=np.uint8)


def test_the_status_is_drawn_in_the_top_left_corner_only():
    rgb = image()
    out = annotate(rgb, NO_DETECTION)
    assert out.shape == rgb.shape and out.dtype == np.uint8
    changed = np.any(out != rgb, axis=2)
    rows, cols = np.nonzero(changed)
    assert changed.sum() > 200, "no visible text"
    assert rows.max() < 30 and cols.max() < 160, "the text leaves the top-left corner"


def test_the_input_image_is_left_untouched():
    rgb = image()
    before = rgb.copy()
    annotate(rgb, NO_DETECTION)
    np.testing.assert_array_equal(rgb, before)
