"""MES-53: how much of a cube must perception see to publish it (#167)? The cube blob's area against the area
of a 5 cm cube at the depth of its back-projected point (so101_perception.geometry.cube_area_ratio), for whole
cubes and for cubes partly hidden by the arm and by the box's walls, with the error of the center perception
would publish.

Our scene (the moved camera, so101_bringup.sim_scene), rendered without ROS; perception's detection with the
ranges of perception.yaml, its pixel -> plane ray, K as the driver's camera_info. Ground truth (the cube's pose,
the segmentation, the sim camera pose) scores the cases only (C-06). Cases, drawn as /sim/randomize does:
- whole: the arm in PARK, every cube in full view (as MES-46's 200/200);
- arm: the TCP put by IK 6-14 cm from the cube towards the camera, 2-12 cm above it (MES-46's /go_to_target
  passes), kept when the arm hides 5-95 % of the cube;
- walls: the cube lying in the box, or the box just in front of it, towards the camera.
The visible share: the cube's pixels against the same scene with the arm lifted out and the box moved away.

In the control container, the workspace built and sourced:
    python3 exp_cube_occlusion.py [scenes per case]   (default 300)
"""

import sys
import tempfile
from pathlib import Path

import cv2
import mujoco
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from so101_sim import ARM_JOINTS, SO101Sim

from so101_brain.kinematics import Kinematics
from so101_bringup import sim_scene
from so101_driver.camera import opencv_intrinsics
from so101_driver.randomize import sample_box_xy
from so101_perception.detection import Thresholds, detect, hsv_ranges
from so101_perception.geometry import cube_area_ratio, pixel_to_world

SCENES = int(sys.argv[1]) if len(sys.argv) > 1 else 300
PARK = np.array([1.9, -1.7, 1.5, 0.7, 0.0])  # brain.yaml's fsm.park_pose
BENT = np.array([0.0, -0.5, 1.0, 0.8, 0.0])  # IK seed: q = 0 is singular (AGENTS)
PLANE_Z = 0.025  # perception_node: the cube center's height
AWAY = (2.0, 0.0)  # as /sim/randomize: the box away while the cube settles
FLIP = np.diag([1.0, -1.0, -1.0])
share = Path(get_package_share_directory("so101_bringup"))
SCENE = yaml.safe_load((share / "config" / "sim_scene.yaml").read_text())
P = yaml.safe_load(
    (Path(get_package_share_directory("so101_perception")) / "config" / "perception.yaml").read_text()
)["so101_perception"]["ros__parameters"]
THRESHOLDS = Thresholds(
    cube=hsv_ranges(*(P[f"hsv_cube_{k}"] for k in ("h_min", "h_max", "s_min", "v_min"))),
    box=hsv_ranges(*(P[f"hsv_box_{k}"] for k in ("h_min", "h_max", "s_min", "v_min", "v_max"))),
    min_area_px=P["min_area_px"],
    min_solidity=P["min_solidity"],
)
kinematics = Kinematics(str(Path(get_package_share_directory("so101_description")) / "urdf" / "so101.urdf"))

sim = SO101Sim(scene_path=sim_scene.generate(tempfile.mkdtemp(), camera=SCENE["camera"]), seed=0)
sim.connect()
m, d = sim.model, sim.data
K = opencv_intrinsics(sim.get_camera_intrinsics())
T = sim.get_camera_extrinsics()
R, t = T[:3, :3] @ FLIP, T[:3, 3]
camera = mujoco.Renderer(m, height=sim.camera_height, width=sim.camera_width)
segmentation = mujoco.Renderer(m, height=sim.camera_height, width=sim.camera_width)
segmentation.enable_segmentation_rendering()
cube_id, box_id, shoulder = m.body("pickup_object").id, m.body("drop_box").id, m.body("shoulder").id
box_geoms = [g for g in range(m.ngeom) if m.geom_bodyid[g] == box_id]
box_half = max(abs(m.geom_pos[g][0]) + m.geom_size[g][0] for g in box_geoms)  # outer half width, m
box_rng = np.random.default_rng(1)
rng = np.random.default_rng(2)


def set_arm(q):
    for name, value in zip(ARM_JOINTS, q, strict=True):
        d.qpos[m.joint(name).qposadr[0]] = value
    mujoco.mj_forward(m, d)


def cube_pixels():
    segmentation.update_scene(d, camera=sim.camera_name)
    geom = segmentation.render()[..., 0]
    return np.count_nonzero(np.where(geom >= 0, m.geom_bodyid[np.clip(geom, 0, None)], -1) == cube_id)


def visible_share():
    """The cube's pixels now, against the arm lifted 10 m and the box moved away."""
    seen = cube_pixels()
    parked, box_xy = m.body_pos[shoulder].copy(), m.body_pos[box_id, :2].copy()
    m.body_pos[shoulder] = parked + (0.0, 0.0, 10.0)
    m.body_pos[box_id, :2] = AWAY
    mujoco.mj_forward(m, d)
    alone = cube_pixels()
    m.body_pos[shoulder], m.body_pos[box_id, :2] = parked, box_xy
    mujoco.mj_forward(m, d)
    return seen / alone if alone else 0.0


def measure():
    """(ratio, visible share, error of the center perception would publish, mm) or None when not published
    as today (no blob, or a blob cut by the image edge)."""
    camera.update_scene(d, camera=sim.camera_name)
    blob = detect(camera.render(), THRESHOLDS).cube
    if blob is None or blob.truncated:
        return None
    p = pixel_to_world(blob.u, blob.v, K, R, t, PLANE_Z)
    if p is None:
        return None
    error = 1e3 * np.linalg.norm(p[:2] - d.body("pickup_object").xpos[:2])
    return cube_area_ratio(blob.contour, p, K, R, t), visible_share(), error


def new_scene():
    m.body_pos[box_id, :2] = AWAY
    sim.randomize_object()
    m.body_pos[box_id, :2] = sample_box_xy(box_rng, d.body("pickup_object").xpos)
    set_arm(PARK)


def towards_camera(cube_xy):
    v = t[:2] - cube_xy
    return v / np.linalg.norm(v)


cases = {"whole": [], "arm": [], "walls": []}
while len(cases["whole"]) < SCENES:
    new_scene()
    if (r := measure()) is not None:
        cases["whole"].append(r)
tries = 0
while len(cases["arm"]) < SCENES and tries < 50 * SCENES:
    tries += 1
    new_scene()
    cube = d.body("pickup_object").xpos.copy()
    target = cube.copy()
    target[:2] += rng.uniform(0.06, 0.14) * towards_camera(cube[:2])
    target[2] += rng.uniform(0.02, 0.12)
    q = kinematics.ik_pos(target, BENT)
    if q is None:
        continue
    set_arm(q)
    if (r := measure()) is not None and 0.05 <= r[1] <= 0.95:
        cases["arm"].append(r)
for k in range(SCENES):
    new_scene()
    cube = d.body("pickup_object").xpos.copy()
    gap = 0.0 if k % 2 == 0 else box_half + 0.025 + rng.uniform(0.002, 0.02)  # in the box / just behind it
    m.body_pos[box_id, :2] = cube[:2] + gap * towards_camera(cube[:2])
    mujoco.mj_forward(m, d)
    if (r := measure()) is not None:
        cases["walls"].append(r)

print(f"camera {SCENE['camera']}; box outer half width {box_half * 1e3:.1f} mm; arm: {tries} tries")
for name, rows in cases.items():
    a = np.array(rows).reshape(-1, 3)
    ratio, seen, error = a.T
    print(
        f"{name:5s} {len(a):3d} published today: ratio p0/5/50 {np.percentile(ratio, [0, 5, 50]).round(3).tolist()}, "
        f"visible p0/50 {np.percentile(seen, [0, 50]).round(3).tolist()}, error p50/95/max "
        f"{np.percentile(error, [50, 95, 100]).round(1).tolist()} mm"
    )
whole = np.array(cases["whole"])[:, 0]
hidden = np.concatenate([np.array(cases[c]).reshape(-1, 3) for c in ("arm", "walls")])
for threshold in (0.7, 0.75, 0.8, 0.85, 0.9, 0.95, 1.0):
    kept = hidden[hidden[:, 0] >= threshold]
    print(
        f"threshold {threshold:.2f}: whole cubes dropped {np.mean(whole < threshold):.1%}; hidden ones still "
        f"published {len(kept)}/{len(hidden)}, their error max {kept[:, 2].max() if len(kept) else 0:.1f} mm, "
        f"above 5 mm {np.sum(kept[:, 2] > 5)}"
    )
