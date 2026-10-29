"""Unit tests of the HSV detection (docs/project/05 §2, §8; ADR-011): the reference images of test/data,
then synthetic scenes for the guards. Pure: no ROS."""

import time

import cv2
import numpy as np
import pytest
from test_reference_images import DATA, LABELS, project

from so101_perception.detection import Thresholds, detect, hsv_ranges

WIDTH, HEIGHT = LABELS["image_size"]
CUBE_RED = (175, 230, 200)  # HSV, OpenCV ranges: H in [0, 180], S and V in [0, 255]


def rgb(image):
    """The reference image as the driver publishes it: rgb8."""
    return cv2.cvtColor(cv2.imread(str(DATA / image["file"])), cv2.COLOR_BGR2RGB)


@pytest.fixture(scope="module")
def detections():
    return [(image, detect(rgb(image))) for image in LABELS["images"]]


def scene(*rects, hsv=CUBE_RED):
    """Gray floor (S = 0) with colored rectangles (x, y, w, h), as rgb8."""
    image = np.full((HEIGHT, WIDTH, 3), (0, 0, 128), np.uint8)
    for x, y, w, h in rects:
        image[y : y + h, x : x + w] = hsv
    return cv2.cvtColor(image, cv2.COLOR_HSV2RGB)


def test_every_whole_cube_is_found_where_the_labels_put_it(detections):
    """100 % of the 32 whole cubes, centroid within 2 px of the projected center (0.7 px at worst)."""
    for image, found in detections:
        if image["cube_view"] == "full":
            cube, name = found.cube, image["file"]
            assert cube is not None, name
            assert not cube.truncated, name
            assert np.linalg.norm(np.array([cube.u, cube.v]) - project(image["cube_center"])) < 2.0, name
            assert found.cube_mask[round(cube.v), round(cube.u)], name  # the mask for the debug image


def test_a_cube_cut_by_the_border_is_found_and_flagged_truncated(detections):
    for image, found in detections:
        if image["cube_view"] == "truncated":
            assert found.cube is not None, image["file"]
            assert found.cube.truncated, image["file"]


def test_no_cube_out_of_view_not_even_one_pixel_of_the_mask(detections):
    """0 false positive: the floor, the arm and the brown box leave no red pixel (05 §8)."""
    for image, found in detections:
        if image["cube_view"] == "out_of_view":
            assert found.cube is None, image["file"]
            assert not found.cube_mask.any(), image["file"]


def test_the_box_is_found_in_every_image(detections):
    """Centroid within 4 px of the projected center at mid-wall height (2.8 px with the arm hiding 15 %)."""
    for image, found in detections:
        box, name = found.box, image["file"]
        assert box is not None, name
        assert not box.truncated, name
        center = project(np.add(image["box_floor_center"], [0.0, 0.0, 0.025]))
        assert np.linalg.norm(np.array([box.u, box.v]) - center) < 4.0, name
        assert found.box_mask[round(box.v), round(box.u)], name


@pytest.mark.parametrize("hue", [165, 175, 2, 6])
def test_red_is_found_on_both_sides_of_hue_0(hue):
    """Red wraps around H = 0/180: two ranges, [160, 180] and [0, 8]."""
    assert detect(scene((100, 100, 12, 12), hsv=(hue, 230, 200))).cube is not None


def test_the_box_brown_is_not_a_cube():
    found = detect(scene((100, 100, 30, 20), hsv=(12, 170, 90)))
    assert found.cube is None
    assert (found.box.u, found.box.v) == pytest.approx((114.5, 109.5))


def test_neither_a_bright_nor_a_grayish_brown_is_the_box():
    """(14, 100, 42): the grey floor under the dim, warm light (MES-49), which the box range took (#175)."""
    for hsv in ((12, 170, 240), (12, 60, 90), (30, 170, 90), (14, 100, 42)):
        assert detect(scene((100, 100, 30, 20), hsv=hsv)).box is None, hsv


def test_the_box_under_the_dim_light_is_the_box():
    assert detect(scene((100, 100, 30, 20), hsv=(10, 200, 29))).box is not None  # MES-49: V median 29


def test_neither_purple_nor_a_dark_or_pale_red_is_a_cube():
    for hsv in ((150, 230, 200), (175, 230, 20), (175, 120, 200)):
        assert detect(scene((100, 100, 12, 12), hsv=hsv)).cube is None, hsv


@pytest.mark.parametrize(("size", "found"), [((4, 6), True), ((4, 5), False)])
def test_a_blob_under_15_px_is_ignored(size, found):
    """Contour area: (w - 1)(h - 1), so 15 px at 4x6 and 12 px at 4x5."""
    assert (detect(scene((100, 100, *size))).cube is not None) == found


def test_a_concave_blob_is_skipped_for_a_smaller_solid_one():
    """An L (solidity ~0.4) is larger than the square, which is kept (solidity > 0.6)."""
    cube = detect(scene((20, 20, 40, 5), (20, 25, 5, 35), (200, 150, 10, 10))).cube
    assert (cube.u, cube.v) == pytest.approx((204.5, 154.5))


@pytest.mark.parametrize(
    "rects", [((20, 20, 14, 14), (200, 150, 10, 10)), ((20, 20, 10, 10), (200, 150, 14, 14))]
)
def test_the_largest_valid_blob_is_the_cube(rects):
    large = max(rects, key=lambda r: r[2])
    cube = detect(scene(*rects)).cube
    assert (cube.u, cube.v) == pytest.approx((large[0] + 6.5, large[1] + 6.5))


@pytest.mark.parametrize(
    ("x", "y", "truncated"),
    [(0, 100, True), (310, 100, True), (100, 0, True), (100, 230, True), (1, 1, False), (309, 229, False)],
)
def test_a_blob_touching_any_border_is_truncated(x, y, truncated):
    assert detect(scene((x, y, 10, 10))).cube.truncated == truncated


def test_a_frame_takes_under_2_ms_at_320x240():
    images = [rgb(image) for image in LABELS["images"]]
    times = []
    for image in images * 3:
        start = time.perf_counter()
        detect(image)
        times.append(time.perf_counter() - start)
    print(f"detect: median {np.median(times) * 1e3:.2f} ms, p95 {np.percentile(times, 95) * 1e3:.2f} ms")
    assert np.median(times) < 0.002


def test_the_parameter_form_gives_the_default_ranges():
    """The ROS parameters (05 §2.2): one hue interval, wrapping around 180 when h_min > h_max."""
    assert hsv_ranges(160, 8, 150, 30) == Thresholds().cube
    assert hsv_ranges(5, 25, 140, 20, 220) == Thresholds().box
