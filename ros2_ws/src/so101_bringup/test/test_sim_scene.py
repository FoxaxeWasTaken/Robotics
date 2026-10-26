"""Our sim scenes (#44, #47, ADR-036): a copy of the provided scene edited with mujoco.MjSpec, never sim/.
- The camera sits at the pose of sim_scene.yaml: no cube or box of /sim/randomize is cut by the image edge
  (21 % of the cubes were with the provided pose, MES-12), and perception's provisional extrinsics are that
  pose.
- The calibration scene's ArUco markers are detected at 640x480 with the arm in PARK, each corner where the
  layout's convention puts it (so101_common.markers); the calibration (#45) finds the camera's pose."""

import hashlib
from pathlib import Path

import cv2
import mujoco
import numpy as np
import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from so101_sim import ARM_JOINTS, DEFAULT_SCENE, SO101Sim

from so101_bringup import sim_scene
from so101_common.markers import marker_corners
from so101_driver.camera import opencv_intrinsics
from so101_driver.randomize import sample_box_xy
from so101_perception import calibration

PARK = [1.9, -1.7, 1.5, 0.7, 0.0]  # the brain's PARK (brain.yaml): calibration is done with the arm there
CONFIG = Path(get_package_share_directory("so101_bringup")) / "config"
LAYOUT = yaml.safe_load((CONFIG / "markers_sim.yaml").read_text())
CAMERA = yaml.safe_load((CONFIG / "sim_scene.yaml").read_text())["camera"]
FLIP = np.diag([1.0, -1.0, -1.0])  # MuJoCo camera frame (y up, looks along -z) -> OpenCV optical frame


def tree_hash(folder):
    digest = hashlib.sha256()
    for path in sorted(p for p in Path(folder).rglob("*") if p.is_file()):
        digest.update(str(path.relative_to(folder)).encode() + path.read_bytes())
    return digest.hexdigest()


@pytest.fixture(scope="module")
def calibration_view(tmp_path_factory):
    before = tree_hash(Path(DEFAULT_SCENE).parent)
    scene = sim_scene.generate(tmp_path_factory.mktemp("scene"), camera=CAMERA, markers=LAYOUT)
    sim = SO101Sim(scene_path=scene, camera_width=640, camera_height=480)
    sim.connect()
    for name, q in zip(ARM_JOINTS, PARK, strict=True):
        sim.data.qpos[sim.model.joint(name).qposadr[0]] = q
    mujoco.mj_forward(sim.model, sim.data)
    renderer = mujoco.Renderer(sim.model, height=480, width=640)
    renderer.update_scene(sim.data, camera=sim.camera_name)
    found = calibration.detect_markers(renderer.render(), LAYOUT["dictionary"])
    return sim, scene, before, found


def test_at_least_5_markers_are_detected_at_640x480_with_the_arm_in_park(calibration_view):
    *_, found = calibration_view
    assert set(found) <= set(LAYOUT["markers"])
    assert len(found) >= 5, sorted(found)


def test_each_detected_corner_is_where_the_layout_puts_it(calibration_view):
    sim, *_, found = calibration_view
    T = sim.get_camera_extrinsics()  # test only: the camera pose, to check the layout's convention
    R, t = T[:3, :3] @ FLIP, T[:3, 3]
    K = opencv_intrinsics(sim.get_camera_intrinsics())
    for i, detected in found.items():
        camera = (marker_corners(LAYOUT["markers"][i], LAYOUT["side_m"]) - t) @ R
        uv = (camera @ K.T)[:, :2] / camera[:, 2:]
        # corner detection on a small, oblique rendered marker: up to ~3 px; a wrong order: a side (>= 28 px)
        assert np.abs(uv - detected).max() < 5.0, (i, uv.round(1).tolist(), detected.round(1).tolist())


def test_the_calibration_finds_the_rendered_camera_within_5_mm_and_0_3_degrees(calibration_view):
    """3.4b (#45): the calibration tool's solve on the rendered view, against the camera MuJoCo renders."""
    sim, *_, found = calibration_view
    T, error, ids = calibration.solve([found], LAYOUT, opencv_intrinsics(sim.get_camera_intrinsics()))
    truth = sim.get_camera_extrinsics()  # test only
    position_mm = 1000 * np.linalg.norm(T[:3, 3] - truth[:3, 3])
    angle_deg = np.degrees(np.linalg.norm(cv2.Rodrigues(T[:3, :3].T @ truth[:3, :3] @ FLIP)[0]))
    print(f"calibration: {len(ids)} markers, {error:.2f} px, {position_mm:.2f} mm, {angle_deg:.3f} deg")
    assert position_mm < 5.0 and angle_deg < 0.3


def test_the_scene_is_a_copy_and_sim_is_left_untouched(calibration_view):
    _, scene, before, _ = calibration_view
    assert Path(DEFAULT_SCENE).parent not in Path(scene).parents
    assert tree_hash(Path(DEFAULT_SCENE).parent) == before


@pytest.fixture(scope="module")
def moved(tmp_path_factory):
    sim = SO101Sim(scene_path=sim_scene.generate(tmp_path_factory.mktemp("moved"), camera=CAMERA))
    sim.connect()
    return sim


def test_the_camera_is_at_the_pose_of_sim_scene_yaml(moved):
    T = moved.get_camera_extrinsics()  # test only
    x, y = np.array(CAMERA["xyaxes"][:3]), np.array(CAMERA["xyaxes"][3:])
    assert np.allclose(T[:3, 3], CAMERA["pos"], atol=1e-9)
    assert np.allclose(T[:3, 0], x / np.linalg.norm(x), atol=1e-4)  # MuJoCo orthonormalizes xyaxes
    assert np.allclose(T[:3, 1], y / np.linalg.norm(y), atol=1e-4)


def test_perceptions_provisional_extrinsics_are_that_pose(moved):
    """Until the own calibration (3.4c) replaces the file, perception must use the camera the sim renders."""
    # read from the sources: the control image does not build so101_perception (CI perception job)
    path = Path(__file__).resolve().parents[2] / "so101_perception" / "config" / "camera_extrinsics.yaml"
    T, truth = np.array(yaml.safe_load(path.read_text())["T_base_cam"]), moved.get_camera_extrinsics()
    assert np.allclose(T[:3, 3], truth[:3, 3], atol=1e-6)
    assert np.allclose(T[:3, :3], truth[:3, :3] @ FLIP, atol=1e-6)


def test_no_cube_or_box_of_randomize_is_cut_by_the_image_edge(moved):
    """MES-12 after the move: 200 scenes drawn as /sim/randomize does, the arm in PARK (the target lock)."""
    m, d = moved.model, moved.data
    segmentation = mujoco.Renderer(m, height=moved.camera_height, width=moved.camera_width)
    segmentation.enable_segmentation_rendering()
    cube, box = m.body("pickup_object").id, m.body("drop_box").id
    moved._rng, box_rng = np.random.default_rng(0), np.random.default_rng(1)
    cut = []
    for k in range(200):
        m.body_pos[box, :2] = (2.0, 0.0)  # as /sim/randomize: the box away while the cube settles
        moved.randomize_object()
        m.body_pos[box, :2] = sample_box_xy(box_rng, d.body("pickup_object").xpos)
        for name, q in zip(ARM_JOINTS, PARK, strict=True):
            d.qpos[m.joint(name).qposadr[0]] = q
        mujoco.mj_forward(m, d)
        segmentation.update_scene(d, camera=moved.camera_name)
        geom = segmentation.render()[..., 0]
        body = np.where(geom >= 0, m.geom_bodyid[np.clip(geom, 0, None)], -1)
        border = np.r_[body[0], body[-1], body[:, 0], body[:, -1]]
        if cube in border or box in border or not (body == cube).any():
            cut.append(k)
    assert cut == []
