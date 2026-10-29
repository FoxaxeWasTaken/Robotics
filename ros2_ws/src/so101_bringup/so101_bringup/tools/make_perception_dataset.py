"""make_perception_dataset - reference images of so101_perception/test/data (step 3.2a, docs/project/05 §8).

    ros2 run so101_bringup make_perception_dataset --out /ros2_ws/src/so101_perception/test/data

Sim renders with known answers, so detection and pixel -> 3D are tested without the sim. Scenes are drawn
as /sim/randomize draws them (the provided randomize_object(), then the driver's box sampler), with the arm
at PARK, where the brain locks its target (06 §6), until each kind of cube view has its count. The labels
come from the sim state: test data only (C-06), never the pipeline.
"""

import argparse
import json
from pathlib import Path

import cv2
import mujoco
import numpy as np
from so101_sim import SO101Sim

from so101_brain.fsm import Params
from so101_common.robot import ARM_JOINT_NAMES
from so101_driver.camera import opencv_intrinsics
from so101_driver.randomize import sample_box_xy

PARK = Params().park_pose  # rad, ARM_JOINT_NAMES order: the brain's PARK (06 §6), as in brain.yaml
PARKED_BOX_XY = (2.0, 0.0)  # m: as /sim/randomize, the box waits there while the cube settles
CORNERS = 0.025 * np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])  # 5 cm cube
FLIP = np.diag([1.0, -1.0, -1.0])  # MuJoCo camera (y up, looks along -z) -> OpenCV optical (05 §3.1)
# Cubes out of view: the provided sampler on a wider ring all around the base, drawn until out of view.
OUT_OF_VIEW_AREA = {"radius_range": (0.18, 0.45), "angle_range_deg": (-180.0, 180.0)}


def cube_view(K, R, t, position, rotation, width, height):
    """'full', 'truncated' or 'out_of_view': the cube's 8 corners projected against the image bounds, which
    span [-0.5, W - 0.5] with OpenCV pixel centers on integers."""
    corners = (position + CORNERS @ rotation.T - t) @ R  # in the optical frame
    u, v = (corners @ K.T)[:, :2].T / corners[:, 2]
    inside = (corners[:, 2] > 0) & (u >= -0.5) & (u <= width - 0.5) & (v >= -0.5) & (v <= height - 0.5)
    if inside.all():
        return "full"
    return "truncated" if inside.any() else "out_of_view"


def visible(segmentation, sim):
    """Fractions of the cube's and of the box's pixels that the parked arm leaves visible, against the same
    scene with the arm lifted 10 m out of the picture (MuJoCo segmentation); 0 when not in the image."""
    m, d = sim.model, sim.data
    shoulder = m.body("shoulder").id
    parked = m.body_pos[shoulder].copy()
    counts = []
    for lift in (0.0, 10.0):
        m.body_pos[shoulder] = parked + np.array([0.0, 0.0, lift])
        mujoco.mj_forward(m, d)
        segmentation.update_scene(d, camera=sim.camera_name)
        geom = segmentation.render()[..., 0]
        body = np.where(geom >= 0, m.geom_bodyid[geom], -1)
        counts.append([np.sum(body == m.body(name).id) for name in ("pickup_object", "drop_box")])
    m.body_pos[shoulder] = parked
    mujoco.mj_forward(m, d)
    return [round(float(p / a), 3) if a else 0.0 for p, a in zip(*counts, strict=True)]


def rounded(values, digits=6):
    return [round(float(v), digits) for v in values]


def make_dataset(out, seed=0, full=32, truncated=8, out_of_view=4):
    """Renders the scenes into out/ (NNN.png + labels.json) and returns the labels."""
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    sim_seed, box_seed = np.random.SeedSequence(seed).spawn(2)  # two streams from one seed, as the driver
    sim = SO101Sim(seed=sim_seed)
    sim.connect()
    m, d = sim.model, sim.data
    box_rng = np.random.default_rng(box_seed)
    for name, q in zip(ARM_JOINT_NAMES, PARK, strict=True):
        d.qpos[m.joint(name).qposadr[0]] = q
    sim.send_action({f"{name}.pos": q for name, q in zip(ARM_JOINT_NAMES, PARK, strict=True)})  # held there
    K = opencv_intrinsics(sim.get_camera_intrinsics())  # as the driver's camera_info [MES-41]
    T = sim.get_camera_extrinsics()  # MuJoCo world = base_link: the base sits at the scene origin
    R, t = T[:3, :3] @ FLIP, T[:3, 3]
    segmentation = mujoco.Renderer(m, height=sim.camera_height, width=sim.camera_width)
    segmentation.enable_segmentation_rendering()
    box = m.body("drop_box").id

    wanted = {"full": full, "truncated": truncated, "out_of_view": out_of_view}
    images = []
    while any(wanted.values()):
        m.body_pos[box, :2] = PARKED_BOX_XY  # else the cube may settle on a wall
        sim.randomize_object(**({} if wanted["full"] or wanted["truncated"] else OUT_OF_VIEW_AREA))
        cube = d.body("pickup_object")
        m.body_pos[box, :2] = sample_box_xy(box_rng, cube.xpos)
        mujoco.mj_forward(m, d)
        rotation = cube.xmat.reshape(3, 3)
        view = cube_view(K, R, t, cube.xpos, rotation, sim.camera_width, sim.camera_height)
        if not wanted[view]:
            continue
        wanted[view] -= 1
        name = f"{len(images):03d}.png"
        bgr = cv2.cvtColor(sim.render_camera(), cv2.COLOR_RGB2BGR)
        cv2.imwrite(str(out / name), bgr, [cv2.IMWRITE_PNG_COMPRESSION, 9])
        cube_visible, box_visible = visible(segmentation, sim)
        images.append(
            {
                "file": name,
                "cube_view": view,
                "cube_center": rounded(cube.xpos),
                "cube_yaw": round(float(np.arctan2(rotation[1, 0], rotation[0, 0]) % (np.pi / 2)), 6),
                "cube_visible": cube_visible,
                "box_floor_center": rounded(d.body("drop_box").xpos),
                "box_visible": box_visible,
            }
        )
    segmentation.close()
    sim.disconnect()

    labels = {
        "generator": "so101_bringup/so101_bringup/tools/make_perception_dataset.py",
        "seed": seed,
        "counts": {"full": full, "truncated": truncated, "out_of_view": out_of_view},
        "image_size": [sim.camera_width, sim.camera_height],
        "K": [rounded(row) for row in K],
        "camera_in_base_link": {
            "frame": "OpenCV optical: x right, y down, z forward",
            "rotation": [rounded(row) for row in R],
            "translation": rounded(t),
        },
        "arm_joints": dict(zip(ARM_JOINT_NAMES, PARK, strict=True)),
        "images": images,
    }
    (out / "labels.json").write_text(json.dumps(labels, indent=1) + "\n")
    return labels


def main():
    parser = argparse.ArgumentParser(description="Reference images of so101_perception/test/data (05 §8).")
    parser.add_argument("--out", type=Path, required=True, help="output folder")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--full", type=int, default=32, help="images with the whole cube in view")
    parser.add_argument("--truncated", type=int, default=8, help="images with the cube cut by the border")
    parser.add_argument("--out-of-view", type=int, default=4, help="images without the cube")
    args = parser.parse_args()
    labels = make_dataset(args.out, args.seed, args.full, args.truncated, args.out_of_view)
    print(f"{len(labels['images'])} images and labels.json in {args.out}")


if __name__ == "__main__":
    main()
