"""Unit tests of the camera geometry (docs/project/05 §3-§4, §8; ADR-012): conventions, round trips, the
errors on the reference images with the exact camera pose of the labels, the extrinsics file. Pure: no ROS."""

from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml
from test_detection import rgb
from test_reference_images import LABELS

from so101_perception.detection import detect
from so101_perception.geometry import (
    Extrinsics,
    cube_area_ratio,
    cube_yaw,
    from_mujoco,
    load_extrinsics,
    pixel_to_world,
    quaternion,
    save_extrinsics,
    world_to_pixel,
)

K = np.array(LABELS["K"])
R_LABELS = np.array(LABELS["camera_in_base_link"]["rotation"])  # exact pose, OpenCV optical frame
T_LABELS = np.array(LABELS["camera_in_base_link"]["translation"])
CONFIG = Path(__file__).parents[1] / "config" / "camera_extrinsics.yaml"
REFERENCE = Path(__file__).parent / "data" / "camera_extrinsics.yaml"  # the camera of the reference images
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


def test_the_cube_yaw_is_within_3_deg_median_and_10_deg_p90():
    """Whole cubes, contour on the top-face plane, minAreaRect (#43; expected 1.3° / 8.3°, MES-14). The
    cube is 90°-symmetric: the yaw is in [-45°, 45°), the error modulo 90°."""
    errors = []
    for image in LABELS["images"]:
        if image["cube_view"] == "full":
            yaw = cube_yaw(detect(rgb(image)).cube.contour, K, R_LABELS, T_LABELS)
            assert -np.pi / 4 <= yaw < np.pi / 4, image["file"]
            errors.append(abs((yaw - image["cube_yaw"] + np.pi / 4) % (np.pi / 2) - np.pi / 4))
    errors = np.degrees(errors)
    print(f"yaw: median {np.median(errors):.2f}°, p90 {np.percentile(errors, 90):.2f}° over {len(errors)}")
    assert np.median(errors) < 3.0
    assert np.percentile(errors, 90) < 10.0


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


def test_the_reference_images_extrinsics_file_is_their_camera():
    """The provided camera pose rendered the reference images; the sim camera moved since (#47), so they carry
    their own file. Within 5 mm and 0.3°, the target of the own calibration (3.4)."""
    T = load_extrinsics(REFERENCE).T_base_cam
    assert np.linalg.norm(T[:3, 3] - T_LABELS) < 0.005
    cos = (np.trace(T[:3, :3].T @ R_LABELS) - 1.0) / 2.0
    assert np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))) < 0.3


def rotation(x, y, z, w):
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


@pytest.mark.parametrize("angle", [0.0, 1.0, 2.83, np.pi - 1e-9, np.pi])  # the camera's: about 162° (2.83)
def test_the_quaternion_of_the_camera_tf_gives_back_its_rotation(angle):
    """#46: the static TF base_link -> camera carries the extrinsics' rotation as a unit quaternion."""
    R = cv2.Rodrigues(np.array([1.0, -2.0, 0.5]) / np.linalg.norm([1.0, -2.0, 0.5]) * angle)[0]
    q = quaternion(R)
    assert np.isclose(np.linalg.norm(q), 1.0)
    assert np.allclose(rotation(*q), R, atol=1e-9)


def blob(width, height):
    """The outline of a filled width x height px rectangle centered on the principal point (0, 0), as
    detection gives it."""
    mask = np.zeros((200, 200), np.uint8)
    mask[100 - height // 2 : 100 - height // 2 + height, 100 - width // 2 : 100 - width // 2 + width] = 255
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    return contours[0][:, 0, :] - 100


DOWN = np.diag([1.0, -1.0, -1.0])  # optical frame looking straight down (z towards -z of base_link)
K500 = np.diag([500.0, 500.0, 1.0])  # fx = 500, principal point at (0, 0)


def test_a_whole_cube_seen_from_straight_above_has_a_ratio_of_1():
    """The camera 1.025 m above the cube center: its top face, 1 m away, spans 25 px at any yaw."""
    t = np.array([0.0, 0.0, 1.05])
    assert cube_area_ratio(blob(25, 25), [0.0, 0.0, 0.025], K500, DOWN, t) == pytest.approx(1.0, rel=1e-3)


def test_the_ratio_scales_with_the_depth_and_the_visible_part():
    t = np.array([0.0, 0.0, 2.05])  # twice as far: 12.5 px
    assert cube_area_ratio(blob(12, 12), [0.0, 0.0, 0.025], K500, DOWN, t) == pytest.approx(
        144 / 156.25, rel=1e-3
    )
    t = np.array([0.0, 0.0, 1.05])
    assert cube_area_ratio(blob(25, 12), [0.0, 0.0, 0.025], K500, DOWN, t) == pytest.approx(
        0.48, rel=1e-3
    )  # half


def test_a_whole_cube_of_the_reference_images_has_a_ratio_of_about_1():
    """Its own outline against the smallest silhouette over the yaws: at least about 1 (05 §6)."""
    for image, found in ((i, detect(rgb(i))) for i in LABELS["images"] if i["cube_view"] == "full"):
        if image["cube_visible"] < 1.0:
            continue
        p = pixel_to_world(found.cube.u, found.cube.v, K, R_LABELS, T_LABELS, 0.025)
        assert 0.9 < cube_area_ratio(found.cube.contour, p, K, R_LABELS, T_LABELS) < 1.4, image["file"]


def test_a_partly_hidden_cube_of_the_reference_images_is_below_the_threshold():
    """#167: the cubes the arm or the box's walls hide (20-80 % visible) fall under perception.yaml's
    min_cube_area_ratio, so the node publishes none of them."""
    params = yaml.safe_load((Path(__file__).parents[1] / "config" / "perception.yaml").read_text())
    threshold = params["so101_perception"]["ros__parameters"]["min_cube_area_ratio"]
    hidden = [i for i in LABELS["images"] if i["cube_view"] == "occluded"]
    assert len(hidden) >= 6
    for image in hidden:
        cube = detect(rgb(image)).cube
        assert cube is not None and not cube.truncated, image["file"]
        p = pixel_to_world(cube.u, cube.v, K, R_LABELS, T_LABELS, 0.025)
        assert cube_area_ratio(cube.contour, p, K, R_LABELS, T_LABELS) < threshold, image["file"]
