"""Why does the box vanish under the robustness demo's dim light (#103, case 5)? Under the default and the dim
lighting of so101_bringup/config/sim_scene.yaml, the box's, the cube's and the floor's pixels in OpenCV HSV,
and the share of each inside perception's box and cube ranges (so101_perception/config/perception.yaml).
Feeds MES-49.

Our scene (the moved camera, so101_bringup.sim_scene), the provided cube pose of seed 0, the arm in PARK; the
pixels of each object from MuJoCo's segmentation; HSV on OpenCV's scale (H 0-179, S and V 0-255), RGB input
(rgb8). The ranges are perception's (05 §2.2): a hue interval wraps around 180 when h_min > h_max.

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
P = yaml.safe_load(PERCEPTION.read_text())["so101_perception"]["ros__parameters"]
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


def in_range(hsv, name):
    """Perception's mask for `name` (box or cube): hue interval (wrapping if h_min > h_max), S and V bounds."""
    h, s, v = hsv[..., 0].astype(int), hsv[..., 1].astype(int), hsv[..., 2].astype(int)
    lo, hi = P[f"hsv_{name}_h_min"], P[f"hsv_{name}_h_max"]
    hue = (h >= lo) & (h <= hi) if lo <= hi else (h >= lo) | (h <= hi)
    return hue & (s >= P[f"hsv_{name}_s_min"]) & (v >= P[f"hsv_{name}_v_min"]) & (v <= P.get(f"hsv_{name}_v_max", 255))


print(f"box range: H {P['hsv_box_h_min']}-{P['hsv_box_h_max']}, S >= {P['hsv_box_s_min']}, "
      f"V {P['hsv_box_v_min']}-{P['hsv_box_v_max']}; cube: H {P['hsv_cube_h_min']}-{P['hsv_cube_h_max']} "
      f"(wraps), S >= {P['hsv_cube_s_min']}, V >= {P['hsv_cube_v_min']}")
for name, lighting in (("default", None), ("dim", SCENE["lighting"]["dim"])):
    sim = SO101Sim(scene_path=sim_scene.generate(tempfile.mkdtemp(), camera=SCENE["camera"], lighting=lighting), seed=0)
    sim.connect()
    for joint, q in zip(ARM_JOINTS, PARK, strict=True):
        sim.data.qpos[sim.model.joint(joint).qposadr[0]] = q
    mujoco.mj_forward(sim.model, sim.data)
    hsv = cv2.cvtColor(render(sim.model, sim.data, False), cv2.COLOR_RGB2HSV)
    geom = render(sim.model, sim.data, True)[..., 0]
    body = np.where(geom >= 0, sim.model.geom_bodyid[np.clip(geom, 0, None)], -1)
    box_mask, cube_mask = in_range(hsv, "box"), in_range(hsv, "cube")
    regions = {
        "drop_box": body == sim.model.body("drop_box").id,
        "cube": body == sim.model.body("pickup_object").id,
        "floor": geom == sim.model.geom("floor").id,
    }
    print(f"== {name}: box mask {box_mask.sum()} px, of which on the box {np.sum(box_mask & regions['drop_box'])}, "
          f"on the floor {np.sum(box_mask & regions['floor'])}")
    for region, where in regions.items():
        px = hsv[where]
        h, s, v = (np.percentile(px[:, k], [5, 50, 95]).round().astype(int).tolist() for k in range(3))
        print(f"   {region:9s} {where.sum():5d} px  H p5/50/95 {h}  S {s}  V {v}  "
              f"in the box range {np.mean(box_mask[where]):.0%}, in the cube range {np.mean(cube_mask[where]):.0%}")
