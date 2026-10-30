"""make_perception_dataset - reference images of so101_perception/test/data (step 3.2a, docs/project/05 §8).

    ros2 run so101_bringup make_perception_dataset --out /ros2_ws/src/so101_perception/test/data

Sim renders with known answers, so detection and pixel -> 3D are tested without the sim. Scenes are drawn
as /sim/randomize draws them (the provided randomize_object(), then the driver's box sampler), with the arm
at PARK, where the brain locks its target (06 §6), until each kind of cube view has its count. Then the
cubes partly hidden (#167), "occluded", that perception must not publish: in turn by the arm (its TCP put by
IK between the camera and the cube, as a /go_to_target passing in front of it) and by the box's walls (the
cube in the box, or just behind its front wall), kept when 20-80 % of the cube shows. The labels come from
the sim state: test data only (C-06), never the pipeline.
"""

import argparse
import json
from pathlib import Path

import cv2
import mujoco
import numpy as np
from ament_index_python.packages import get_package_share_directory
from so101_sim import SO101Sim

from so101_brain.fsm import Params
from so101_brain.kinematics import Kinematics
from so101_common.robot import ARM_JOINT_NAMES
from so101_driver.camera import opencv_intrinsics
from so101_driver.randomize import sample_box_xy

PARK = Params().park_pose  # rad, ARM_JOINT_NAMES order: the brain's PARK (06 §6), as in brain.yaml
PARKED_BOX_XY = (2.0, 0.0)  # m: as /sim/randomize, the box waits there while the cube settles
CORNERS = 0.025 * np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])  # 5 cm cube
FLIP = np.diag([1.0, -1.0, -1.0])  # MuJoCo camera (y up, looks along -z) -> OpenCV optical (05 §3.1)
BENT = np.array([0.0, -0.5, 1.0, 0.8, 0.0])  # IK seed: q = 0 is singular (04 §8)
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


def visible(segmentation, sim, box_too=False):
    """Fractions of the cube's and of the box's pixels that the arm (and with box_too, the box) leaves
    visible: against the same scene with the arm lifted 10 m out of the picture (and, for the cube with
    box_too, the box parked away); 0 when not in the image (MuJoCo segmentation)."""
    m, d = sim.model, sim.data
    shoulder, box = m.body("shoulder").id, m.body("drop_box").id
    parked, box_xy = m.body_pos[shoulder].copy(), m.body_pos[box, :2].copy()
    counts = []
    for lift, away in ((0.0, False), (10.0, False), (10.0, True))[: 3 if box_too else 2]:
        m.body_pos[shoulder] = parked + np.array([0.0, 0.0, lift])
        m.body_pos[box, :2] = PARKED_BOX_XY if away else box_xy
        mujoco.mj_forward(m, d)
        segmentation.update_scene(d, camera=sim.camera_name)
        geom = segmentation.render()[..., 0]
        body = np.where(geom >= 0, m.geom_bodyid[geom], -1)
        counts.append([np.sum(body == m.body(name).id) for name in ("pickup_object", "drop_box")])
    m.body_pos[shoulder], m.body_pos[box, :2] = parked, box_xy
    mujoco.mj_forward(m, d)
    (cube, box_px), cube_alone, box_alone = counts[0], counts[-1][0], counts[1][1]
    return [round(float(p / a), 3) if a else 0.0 for p, a in ((cube, cube_alone), (box_px, box_alone))]


def rounded(values, digits=6):
    return [round(float(v), digits) for v in values]


def set_arm(sim, q):
    for name, value in zip(ARM_JOINT_NAMES, q, strict=True):
        sim.data.qpos[sim.model.joint(name).qposadr[0]] = value
    mujoco.mj_forward(sim.model, sim.data)


def place_occluder(sim, cube, camera_xy, n, rng, kinematics):
    """Hides part of the cube, in turn (n even, odd) behind the arm, its TCP put by IK between the camera and
    the cube, and behind the box's walls, the cube in the box or just behind its front wall: the occluder's
    name, None when the IK finds no pose."""
    m, d = sim.model, sim.data
    to_camera = camera_xy - cube.xpos[:2]  # on the table
    towards = to_camera / np.linalg.norm(to_camera)
    if n % 2 == 0:
        target = cube.xpos + np.array([*(rng.uniform(0.06, 0.14) * towards), rng.uniform(0.02, 0.12)])
        q = kinematics.ik_pos(target, BENT)
        if q is None:
            return None
        set_arm(sim, q)
        return "arm"
    box = m.body("drop_box").id
    half = max(abs(m.geom_pos[g][0]) + m.geom_size[g][0] for g in range(m.ngeom) if m.geom_bodyid[g] == box)
    gap = rng.choice([0.0, half + 0.025 + rng.uniform(0.002, 0.02)])
    m.body_pos[box, :2] = cube.xpos[:2] + gap * towards
    mujoco.mj_forward(m, d)
    return "walls"


def make_dataset(out, seed=0, full=32, truncated=8, out_of_view=4, occluded=8):
    """Renders the scenes into out/ (NNN.png + labels.json) and returns the labels."""
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    # Two streams from one seed, as the driver; the third places the occluders (its own: the first two, so
    # the images before it, do not change with it).
    sim_seed, box_seed, occluder_seed = np.random.SeedSequence(seed).spawn(3)
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
    kinematics = Kinematics(
        str(Path(get_package_share_directory("so101_description")) / "urdf" / "so101.urdf")
    )
    rng = np.random.default_rng(occluder_seed)
    hidden = 0
    while hidden < occluded:
        m.body_pos[box, :2] = PARKED_BOX_XY
        sim.randomize_object()
        cube = d.body("pickup_object")
        m.body_pos[box, :2] = sample_box_xy(box_rng, cube.xpos)
        set_arm(sim, PARK)
        rotation = cube.xmat.reshape(3, 3)
        if cube_view(K, R, t, cube.xpos, rotation, sim.camera_width, sim.camera_height) != "full":
            continue
        occluder = place_occluder(sim, cube, t[:2], hidden, rng, kinematics)
        if occluder is None:
            continue
        cube_visible, box_visible = visible(segmentation, sim, box_too=True)
        if not 0.2 <= cube_visible <= 0.8:
            continue
        hidden += 1
        name = f"{len(images):03d}.png"
        bgr = cv2.cvtColor(sim.render_camera(), cv2.COLOR_RGB2BGR)
        cv2.imwrite(str(out / name), bgr, [cv2.IMWRITE_PNG_COMPRESSION, 9])
        images.append(
            {
                "file": name,
                "cube_view": "occluded",
                "occluder": occluder,
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
        "counts": {"full": full, "truncated": truncated, "out_of_view": out_of_view, "occluded": occluded},
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
    parser.add_argument("--occluded", type=int, default=8, help="images with the cube partly hidden")
    args = parser.parse_args()
    labels = make_dataset(args.out, args.seed, args.full, args.truncated, args.out_of_view, args.occluded)
    print(f"{len(labels['images'])} images and labels.json in {args.out}")


if __name__ == "__main__":
    main()
