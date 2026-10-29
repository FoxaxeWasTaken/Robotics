"""Perception's thresholds under both sim lights (#175): perception.yaml's ranges find the cube and the box,
at their segmented centroid, in scenes drawn as /sim/randomize does, under the default light and under the
robustness demo's dim, warm one (lighting:=dim, MES-49). Ground truth from MuJoCo's segmentation (C-06:
tests only)."""

from pathlib import Path

import mujoco
import numpy as np
import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from so101_sim import ARM_JOINTS, SO101Sim

from so101_bringup import sim_scene
from so101_driver.randomize import sample_box_xy
from so101_perception.detection import Thresholds, detect, hsv_ranges

PARK = [
    1.9,
    -1.7,
    1.5,
    0.7,
    0.0,
]  # brain.yaml's fsm.park_pose: the arm out of the view, as at the target lock
SCENES = 20
SCENE = yaml.safe_load(
    (Path(get_package_share_directory("so101_bringup")) / "config" / "sim_scene.yaml").read_text()
)
PERCEPTION = Path(get_package_share_directory("so101_perception")) / "config" / "perception.yaml"
P = yaml.safe_load(PERCEPTION.read_text())["so101_perception"]["ros__parameters"]
THRESHOLDS = Thresholds(  # as perception_node builds them from its parameters
    cube=hsv_ranges(*(P[f"hsv_cube_{k}"] for k in ("h_min", "h_max", "s_min", "v_min"))),
    box=hsv_ranges(*(P[f"hsv_box_{k}"] for k in ("h_min", "h_max", "s_min", "v_min", "v_max"))),
    min_area_px=P["min_area_px"],
    min_solidity=P["min_solidity"],
)


def views(tmp_path_factory, lighting):
    """For each scene: the detection, the segmented centroids of the cube and the box, and the box mask's
    pixels off the box."""
    path = sim_scene.generate(tmp_path_factory.mktemp("lit"), camera=SCENE["camera"], lighting=lighting)
    sim = SO101Sim(scene_path=path)
    sim.connect()
    m, d = sim.model, sim.data
    image = mujoco.Renderer(m, height=sim.camera_height, width=sim.camera_width)
    segmentation = mujoco.Renderer(m, height=sim.camera_height, width=sim.camera_width)
    segmentation.enable_segmentation_rendering()
    cube, box = m.body("pickup_object").id, m.body("drop_box").id
    sim._rng, box_rng = np.random.default_rng(0), np.random.default_rng(1)
    out = []
    for _ in range(SCENES):
        m.body_pos[box, :2] = (2.0, 0.0)  # as /sim/randomize: the box away while the cube settles
        sim.randomize_object()
        m.body_pos[box, :2] = sample_box_xy(box_rng, d.body("pickup_object").xpos)
        for name, q in zip(ARM_JOINTS, PARK, strict=True):
            d.qpos[m.joint(name).qposadr[0]] = q
        mujoco.mj_forward(m, d)
        image.update_scene(d, camera=sim.camera_name)
        segmentation.update_scene(d, camera=sim.camera_name)
        geom = segmentation.render()[..., 0]
        body = np.where(geom >= 0, m.geom_bodyid[np.clip(geom, 0, None)], -1)
        centroid = {
            name: np.argwhere(body == i).mean(axis=0)[::-1] for name, i in (("cube", cube), ("box", box))
        }
        detection = detect(image.render(), THRESHOLDS)
        out.append((detection, centroid, np.count_nonzero(detection.box_mask & (body != box))))
    sim.disconnect()
    return out


@pytest.fixture(scope="module", params=["default", "dim"])
def lit(request, tmp_path_factory):
    return request.param, views(
        tmp_path_factory, None if request.param == "default" else SCENE["lighting"]["dim"]
    )


def test_the_box_is_found_at_its_centroid(lit):
    name, scenes = lit
    found = [
        d.box is not None and np.hypot(d.box.u - c["box"][0], d.box.v - c["box"][1]) < 3.0
        for d, c, _ in scenes
    ]
    assert all(found), (
        f"{name}: box missed or off by 3 px or more in scenes {np.flatnonzero(~np.array(found))}"
    )


def test_the_cube_is_found_at_its_centroid(lit):
    name, scenes = lit
    found = [
        d.cube is not None and np.hypot(d.cube.u - c["cube"][0], d.cube.v - c["cube"][1]) < 2.0
        for d, c, _ in scenes
    ]
    assert all(found), (
        f"{name}: cube missed or off by 2 px or more in scenes {np.flatnonzero(~np.array(found))}"
    )


def test_the_box_mask_stays_on_the_box(lit):
    """The dim floor turned box-colored (MES-49): 29,864 px of the box mask on the floor."""
    name, scenes = lit
    off = max(outside for _, _, outside in scenes)
    assert off < 50, f"{name}: up to {off} px of the box mask off the box (the box covers about 1,350 px)"
