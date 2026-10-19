"""Reference images of test/data (step 3.2a, docs/project/05 §8): within the size budget of ADR-029, and
labels that agree with the pixels. The generator and its determinism test are in so101_bringup."""

import json
from pathlib import Path

import cv2
import numpy as np

DATA = Path(__file__).parent / "data"
LABELS = json.loads((DATA / "labels.json").read_text())
WIDTH, HEIGHT = LABELS["image_size"]


def red_mask(bgr):
    """The red cube; the floor, the arm and the brown box have no such pixel."""
    b, g, r = (bgr[..., k].astype(int) for k in range(3))
    return (r > 120) & (r > 2 * g) & (r > 2 * b)


def project(point):
    """base_link point -> pixel, with the stored K and camera pose (OpenCV optical frame)."""
    R = np.array(LABELS["camera_in_base_link"]["rotation"])
    t = np.array(LABELS["camera_in_base_link"]["translation"])
    x, y, z = (np.asarray(point) - t) @ R
    u, v, _ = np.array(LABELS["K"]) @ (x / z, y / z, 1.0)
    return np.array([u, v])


def test_the_set_fits_the_size_budget():
    pngs = sorted(DATA.glob("*.png"))
    assert sorted(image["file"] for image in LABELS["images"]) == [p.name for p in pngs]
    assert len(pngs) >= 40
    assert max(p.stat().st_size for p in pngs) < 200_000
    assert sum(p.stat().st_size for p in pngs) < 5_000_000
    for p in pngs:
        assert cv2.imread(str(p)).shape == (HEIGHT, WIDTH, 3), p.name


def test_the_set_has_whole_truncated_and_missing_cubes():
    views = [image["cube_view"] for image in LABELS["images"]]
    assert views.count("full") >= 30
    assert views.count("truncated") >= 5
    assert views.count("out_of_view") >= 3


def test_the_labels_agree_with_the_pixels():
    """Catches a wrong frame, convention or reference point (cm), not mm: the blob centroid sits within
    1.9 px of the projected center. The mm come from the sim state, pinned by the so101_bringup test."""
    for image in LABELS["images"]:
        red = red_mask(cv2.imread(str(DATA / image["file"])))
        on_border = red[0].any() or red[-1].any() or red[:, 0].any() or red[:, -1].any()
        name, view = image["file"], image["cube_view"]
        if view == "out_of_view":
            assert not red.any(), name
            continue
        assert red.sum() > 50, name
        assert on_border == (view == "truncated"), name
        if view == "full":
            rows, cols = np.nonzero(red)
            centroid = np.array([cols.mean(), rows.mean()])
            assert np.linalg.norm(centroid - project(image["cube_center"])) < 3.0, name
