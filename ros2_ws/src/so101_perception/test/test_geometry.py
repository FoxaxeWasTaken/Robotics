"""Unit tests of the camera geometry (docs/project/05 §3-§4, §8; ADR-012): conventions, round trips, the
errors on the reference images with the exact camera pose of the labels, the extrinsics file. Pure: no ROS."""

from pathlib import Path

import numpy as np
import pytest
import yaml
from test_detection import rgb
from test_reference_images import LABELS

from so101_perception.detection import detect
from so101_perception.geometry import (
    Extrinsics,
    from_mujoco,
    load_extrinsics,
    pixel_to_world,
    save_extrinsics,
    world_to_pixel,
)

K = np.array(LABELS["K"])
R_LABELS = np.array(LABELS["camera_in_base_link"]["rotation"])  # exact pose, OpenCV optical frame
T_LABELS = np.array(LABELS["camera_in_base_link"]["translation"])
CONFIG = Path(__file__).parents[1] / "config" / "camera_extrinsics.yaml"
CUBE_Z = 0.025  # center of a cube resting on the table (C-22)


def test_a_round_trip_world_pixel_world_is_exact():
    """3D -> pixel -> 3D on the plane of the point, over the workspace (05 §8: < 1e-9 m)."""
    T = load_extrinsics(CONFIG).T_base_cam
    rng = np.random.default_rng(0)
    for _ in range(200):
        r, a = rng.uniform(0.12, 0.35), rng.uniform(-np.pi / 2, np.pi / 3)
        p = np.array([r * np.cos(a), r * np.sin(a), rng.uniform(0.0, 0.1)])
        u, v = world_to_pixel(p, K, T[:3, :3], T[:3, 3])
        assert np.linalg.norm(pixel_to_world(u, v, K, T[:3, :3], T[:3, 3], p[2]) - p) < 1e-9


def test_flip_turns_a_mujoco_camera_into_the_opencv_frame():
    """A MuJoCo camera 1 m above the origin, unrotated: it looks along -z, its y (up in the image) along
    world +y. In the OpenCV frame, the origin is at the principal point, +x to the right, +y upwards."""
    T = from_mujoco(np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 1.0], [0, 0, 0, 1]]))
    R, t = T[:3, :3], T[:3, 3]
    np.testing.assert_allclose(world_to_pixel([0.0, 0.0, 0.0], K, R, t), K[:2, 2])
    assert world_to_pixel([0.1, 0.0, 0.0], K, R, t)[0] > K[0, 2]
    assert world_to_pixel([0.0, 0.1, 0.0], K, R, t)[1] < K[1, 2]
    np.testing.assert_allclose(pixel_to_world(*K[:2, 2], K, R, t, 0.0), [0.0, 0.0, 0.0], atol=1e-12)


def test_a_pixel_above_the_horizon_is_not_on_the_table():
    assert pixel_to_world(160.0, -500.0, K, R_LABELS, T_LABELS, CUBE_Z) is None


def test_a_ray_parallel_to_the_table_is_not_on_it():
    T = from_mujoco(np.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0.3], [0, 0, 0, 1]]))  # looks along +y
    assert pixel_to_world(*K[:2, 2], K, T[:3, :3], T[:3, 3], CUBE_Z) is None


def test_the_cube_center_is_within_5_mm_on_average():
    """Whole cubes, detected centroid on the plane z = 2.5 cm (expected 2.7 mm, MES-13)."""
    errors = []
    for image in LABELS["images"]:
        if image["cube_view"] == "full":
            cube = detect(rgb(image)).cube
            p = pixel_to_world(cube.u, cube.v, K, R_LABELS, T_LABELS, CUBE_Z)
            errors.append(np.linalg.norm(p[:2] - image["cube_center"][:2]))
    print(f"cube: mean {np.mean(errors) * 1e3:.2f} mm, max {np.max(errors) * 1e3:.2f} mm over {len(errors)}")
    assert np.mean(errors) < 0.005


def test_the_box_floor_center_is_within_1_cm_on_average():
    """Box centroid on the plane z = 2.5 cm, mid-height of the walls, below it the floor center (MES-15)."""
    errors = []
    for image in LABELS["images"]:
        box = detect(rgb(image)).box
        p = pixel_to_world(box.u, box.v, K, R_LABELS, T_LABELS, CUBE_Z)
        errors.append(np.linalg.norm(p[:2] - image["box_floor_center"][:2]))
    print(f"box: mean {np.mean(errors) * 1e3:.2f} mm, max {np.max(errors) * 1e3:.2f} mm over {len(errors)}")
    assert np.mean(errors) < 0.01


def test_the_extrinsics_file_round_trips(tmp_path):
    saved = Extrinsics(from_mujoco(np.eye(4)), "test", "2026-10-20", 0.42)
    save_extrinsics(tmp_path / "e.yaml", saved)
    loaded = load_extrinsics(tmp_path / "e.yaml")
    np.testing.assert_array_equal(loaded.T_base_cam, saved.T_base_cam)
    assert (loaded.method, loaded.date, loaded.reprojection_error_px) == ("test", "2026-10-20", 0.42)


@pytest.mark.parametrize(
    "change",
    [
        {"T_base_cam": np.diag([1.0, 1.0, -1.0, 1.0]).tolist()},  # a mirror, not a rotation
        {"T_base_cam": np.diag([2.0, 2.0, 2.0, 1.0]).tolist()},  # scaled
        {"T_base_cam": np.eye(4)[:3].tolist()},  # 3x4
        {"child_frame": "external_cam"},
    ],
)
def test_a_file_that_is_not_the_camera_pose_is_refused(tmp_path, change):
    save_extrinsics(tmp_path / "e.yaml", Extrinsics(np.eye(4), "test", "2026-10-20"))
    data = yaml.safe_load((tmp_path / "e.yaml").read_text()) | change
    (tmp_path / "e.yaml").write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match=r"e\.yaml"):
        load_extrinsics(tmp_path / "e.yaml")


def test_the_extrinsics_file_is_the_camera_of_the_reference_images():
    """Within 5 mm and 0.3°, the target of the own calibration (3.4), which will replace the file."""
    T = load_extrinsics(CONFIG).T_base_cam
    assert np.linalg.norm(T[:3, 3] - T_LABELS) < 0.005
    cos = (np.trace(T[:3, :3].T @ R_LABELS) - 1.0) / 2.0
    assert np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))) < 0.3
