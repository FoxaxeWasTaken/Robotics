"""A small game scene for the driver's own tests (#80): the provided scene plus a `grid` body at the layout's
pose and a few tokens named as so101_bringup.sim_scene names them. No slot: what falls is tested on the real
scene, in so101_bringup/test."""

import shutil
from pathlib import Path

import mujoco
import numpy as np
from ament_index_python.packages import get_package_share_directory
from so101_sim import DEFAULT_SCENE

from so101_connect4 import layout as connect4_layout

LAYOUT = connect4_layout.load()
PER_COLOR = 8


def layout_path() -> str:
    return str(Path(get_package_share_directory("so101_connect4")) / "config" / "connect4_grid.yaml")


def parking(color, i):
    return [-0.5 - 0.05 * i, 0.2 if color == "red" else 0.3, LAYOUT["token"]["thickness"] / 2]


def scene(folder) -> Path:
    folder = Path(folder) / "so101"
    shutil.copytree(Path(DEFAULT_SCENE).parent, folder)
    spec = mujoco.MjSpec.from_file(str(folder / Path(DEFAULT_SCENE).name))
    token, contact = LAYOUT["token"], LAYOUT["contact"]
    spec.option.impratio = contact["impratio"]
    spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    T = connect4_layout.grid_pose(LAYOUT)
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, T[:3, :3].ravel())
    spec.worldbody.add_body(name="grid", pos=T[:3, 3], quat=quat)
    for color in ("red", "yellow"):
        for i in range(PER_COLOR):
            name = f"token_{color}_{i}"
            body = spec.worldbody.add_body(name=name, pos=parking(color, i))
            joint = body.add_joint(name=f"{name}_free", type=mujoco.mjtJoint.mjJNT_FREE)
            joint.damping = [contact["damping"], 0, 0]
            body.add_geom(
                name=name,
                type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                size=[token["diameter"] / 2, token["thickness"] / 2, 0],
                mass=token["mass"],
                friction=[token["friction"], 0.005, 0.0001],
                solref=[contact["solref"], 1],
                solmix=contact["solmix"],
            )
    path = folder / "scene_fake_connect4.xml"
    path.write_text(spec.to_xml())
    return path
