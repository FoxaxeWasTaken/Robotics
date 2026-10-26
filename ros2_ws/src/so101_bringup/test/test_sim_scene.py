"""Our sim scenes (#44, ADR-036): a copy of the provided scene edited with mujoco.MjSpec, never sim/.
The calibration scene's ArUco markers are detected at 640x480 with the arm in PARK, each corner where the
layout's convention puts it (sim_scene.marker_corners), which the calibration tool (3.4b) relies on."""

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
from so101_driver.camera import opencv_intrinsics

PARK = [1.9, -1.7, 1.5, 0.7, 0.0]  # the brain's PARK (brain.yaml): calibration is done with the arm there
CONFIG = Path(get_package_share_directory("so101_bringup")) / "config"
LAYOUT = yaml.safe_load((CONFIG / "markers_sim.yaml").read_text())


def tree_hash(folder):
    digest = hashlib.sha256()
    for path in sorted(p for p in Path(folder).rglob("*") if p.is_file()):
        digest.update(str(path.relative_to(folder)).encode() + path.read_bytes())
    return digest.hexdigest()


@pytest.fixture(scope="module")
def calibration_view(tmp_path_factory):
    before = tree_hash(Path(DEFAULT_SCENE).parent)
    scene = sim_scene.generate(tmp_path_factory.mktemp("scene"), markers=LAYOUT)
    sim = SO101Sim(scene_path=scene, camera_width=640, camera_height=480)
    sim.connect()
    for name, q in zip(ARM_JOINTS, PARK, strict=True):
        sim.data.qpos[sim.model.joint(name).qposadr[0]] = q
    mujoco.mj_forward(sim.model, sim.data)
    renderer = mujoco.Renderer(sim.model, height=480, width=640)
    renderer.update_scene(sim.data, camera=sim.camera_name)
    gray = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2GRAY)
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, LAYOUT["dictionary"]))
    corners, ids, _ = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters()).detectMarkers(gray)
    found = {} if ids is None else {int(i): c[0] for i, c in zip(ids.ravel(), corners, strict=True)}
    return sim, scene, before, found


def test_at_least_5_markers_are_detected_at_640x480_with_the_arm_in_park(calibration_view):
    *_, found = calibration_view
    assert set(found) <= set(LAYOUT["markers"])
    assert len(found) >= 5, sorted(found)


def test_each_detected_corner_is_where_the_layout_puts_it(calibration_view):
    sim, *_, found = calibration_view
    T = sim.get_camera_extrinsics()  # test only: the camera pose, to check the layout's convention
    R, t = T[:3, :3] @ np.diag([1.0, -1.0, -1.0]), T[:3, 3]  # MuJoCo camera frame -> OpenCV optical frame
    K = opencv_intrinsics(sim.get_camera_intrinsics())
    for i, detected in found.items():
        camera = (sim_scene.marker_corners(LAYOUT["markers"][i], LAYOUT["side_m"]) - t) @ R
        uv = (camera @ K.T)[:, :2] / camera[:, 2:]
        # corner detection on a rendered texture: a few px; a wrong corner order: a marker side (>= 28 px)
        assert np.abs(uv - detected).max() < 3.0, (i, uv.round(1).tolist(), detected.round(1).tolist())


def test_the_scene_is_a_copy_and_sim_is_left_untouched(calibration_view):
    _, scene, before, _ = calibration_view
    assert Path(DEFAULT_SCENE).parent not in Path(scene).parents
    assert tree_hash(Path(DEFAULT_SCENE).parent) == before
