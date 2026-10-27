"""Why does the box vanish under the robustness demo's dim light (#103, case 5)? Its pixels' HSV, and the cube's,
under the default and the dim lighting of so101_bringup/config/sim_scene.yaml, against the perception
thresholds (so101_perception/config/perception.yaml). Feeds MES-49.

Our scene (the moved camera, so101_bringup.sim_scene), the provided cube pose of seed 0, the arm in PARK; the
objects' pixels from MuJoCo's segmentation; HSV on OpenCV's scale (H 0-179, S and V 0-255), RGB input (rgb8).

In the control container, the workspace built and sourced:
    python3 exp_lighting_hsv.py
"""

import tempfile
from pathlib import Path

import cv2
import mujoco
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from so101_sim import ARM_JOINTS, SO101Sim

from so101_bringup import sim_scene

SCENE = yaml.safe_load((Path(get_package_share_directory("so101_bringup")) / "config" / "sim_scene.yaml").read_text())
PERCEPTION = Path(get_package_share_directory("so101_perception")) / "config" / "perception.yaml"
HSV = yaml.safe_load(PERCEPTION.read_text())["so101_perception"]["ros__parameters"]
PARK = [1.9, -1.7, 1.5, 0.7, 0.0]  # brain.yaml's fsm.park_pose: the arm out of the view
W, H = 320, 240


def render(model, data, segmentation):
    renderer = mujoco.Renderer(model, H, W)  # one per pass: reusing it across models breaks the segmentation
    if segmentation:
        renderer.enable_segmentation_rendering()
    renderer.update_scene(data, camera="external_cam")
    image = renderer.render()
    renderer.close()
    return image


print(f"thresholds: box V >= {HSV['hsv_box_v_min']}, S >= {HSV['hsv_box_s_min']}; cube V >= {HSV['hsv_cube_v_min']}")
for name, lighting in (("default", None), ("dim", SCENE["lighting"]["dim"])):
    sim = SO101Sim(scene_path=sim_scene.generate(tempfile.mkdtemp(), camera=SCENE["camera"], lighting=lighting), seed=0)
    sim.connect()
    for joint, q in zip(ARM_JOINTS, PARK, strict=True):
        sim.data.qpos[sim.model.joint(joint).qposadr[0]] = q
    mujoco.mj_forward(sim.model, sim.data)
    hsv = cv2.cvtColor(render(sim.model, sim.data, False), cv2.COLOR_RGB2HSV)
    geom = render(sim.model, sim.data, True)[..., 0]
    body = np.where(geom >= 0, sim.model.geom_bodyid[np.clip(geom, 0, None)], -1)
    for obj, v_min in (("drop_box", HSV["hsv_box_v_min"]), ("pickup_object", HSV["hsv_cube_v_min"])):
        px = hsv[body == sim.model.body(obj).id]
        h, s, v = (np.percentile(px[:, k], [5, 50, 95]).round().astype(int).tolist() for k in range(3))
        above = np.mean(px[:, 2] >= v_min)
        print(f"{name:8s} {obj:14s} {len(px):5d} px  H p5/50/95 {h}  S {s}  V {v}  V >= {v_min}: {above:.0%}")
