"""Reference images of so101_perception/test/data (step 3.2a): the generator is deterministic, and the
committed set is what it gives for the committed seed (so the data cannot drift from its generator)."""

import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from so101_bringup.tools.make_perception_dataset import cube_view, make_dataset

DATA = Path(__file__).resolve().parents[2] / "so101_perception" / "test" / "data"


def assert_close(a, b, path="labels"):
    """Same keys, strings and flags; numbers within 10 µm or µrad (another CPU may round the physics)."""
    if isinstance(a, dict):
        assert a.keys() == b.keys(), path
        for k in a:
            assert_close(a[k], b[k], f"{path}.{k}")
    elif isinstance(a, list):
        assert len(a) == len(b), path
        for i, (x, y) in enumerate(zip(a, b, strict=True)):
            assert_close(x, y, f"{path}[{i}]")
    elif isinstance(a, float):
        assert a == pytest.approx(b, abs=1e-5), path
    else:
        assert a == b, path


@pytest.mark.parametrize("side", [1.0, -1.0])
@pytest.mark.parametrize("axis", [0, 1])
@pytest.mark.parametrize(("margin_px", "view"), [(0.1, "full"), (-0.1, "truncated")])
def test_the_image_ends_half_a_pixel_beyond_the_border_pixel_centers(margin_px, view, axis, side):
    """OpenCV pixel centers on integers: the image spans [-0.5, W - 0.5] x [-0.5, H - 0.5]. A camera 1 m
    above the cube, looking down: its top corners (0.975 m away) margin_px inside each border in turn."""
    f, size = 246.0, (320, 240)
    K = np.array([[f, 0.0, (size[0] - 1) / 2], [0.0, f, (size[1] - 1) / 2], [0.0, 0.0, 1.0]])
    position = np.zeros(3)
    position[axis] = side * ((size[axis] / 2 - margin_px) * 0.975 / f - 0.025)
    R, t = np.diag([1.0, -1.0, -1.0]), np.array([0.0, 0.0, 1.0])
    assert cube_view(K, R, t, position, np.eye(3), *size) == view


def test_the_same_seed_gives_identical_labels(tmp_path):
    for run, seed in (("a", 3), ("b", 3), ("c", 4)):
        make_dataset(tmp_path / run, seed=seed, full=2, truncated=1, out_of_view=1)
    labels = {run: (tmp_path / run / "labels.json").read_text() for run in "abc"}
    assert labels["a"] == labels["b"]
    assert json.loads(labels["a"])["images"] != json.loads(labels["c"])["images"]


def test_the_committed_set_is_what_the_generator_gives(tmp_path):
    committed = json.loads((DATA / "labels.json").read_text())
    make_dataset(tmp_path, seed=committed["seed"], **committed["counts"])
    assert_close(json.loads((tmp_path / "labels.json").read_text()), committed)
    for image in committed["images"]:
        kept = cv2.imread(str(DATA / image["file"])).astype(int)
        rendered = cv2.imread(str(tmp_path / image["file"])).astype(int)
        # Another Mesa build may shade a few pixels differently: mean gray-level difference.
        assert np.abs(kept - rendered).mean() < 1.0, image["file"]
