"""reach.py: where could a token dispenser's pick station stand? A spike for the design study.
1. Where the drop's pose (release_pose: jaws across the arm plane, approach tilted 70 deg) reaches, by azimuth and
   height, with 25 mm of straight lift above it.
2. What the open jaws occupy around a token at that pose: how low and how far they go, so what walls can stay.
3. Where the scene camera sees a marker there, and how obliquely.
"""

import sys
import tempfile
from pathlib import Path

import mujoco
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from so101_sim import ARM_JOINTS, SO101Sim

from so101_brain import drop
from so101_brain.drop import release_pose
from so101_brain.fsm import Params
from so101_brain.kinematics import PAN_AXIS_X, Kinematics
from so101_bringup import sim_scene
from so101_common.robot import pct_to_rad
from so101_connect4 import layout

share = Path(get_package_share_directory("so101_bringup")) / "config"
CAMERA = yaml.safe_load((share / "sim_scene.yaml").read_text())["camera"]
LAYOUT = layout.load()
kin = Kinematics(str(Path(get_package_share_directory("so101_description")) / "urdf" / "so101.urdf"))
p = Params()
scene = sim_scene.generate(Path(tempfile.mkdtemp()), camera=CAMERA, connect4=LAYOUT)
sim = SO101Sim(scene_path=scene, camera_width=640, camera_height=480)
sim.connect()
model, data = sim.model, sim.data


def at(azimuth_deg, r, z):
    a = np.radians(azimuth_deg)
    return np.array([PAN_AXIS_X + r * np.cos(a), r * np.sin(a), z])


def set_arm(q, gripper_pct):
    for name, value in zip(ARM_JOINTS, q, strict=False):
        data.qpos[model.joint(name).qposadr[0]] = value
    data.qpos[model.joint("gripper").qposadr[0]] = pct_to_rad(gripper_pct)
    mujoco.mj_forward(model, data)


print("== 1. reach of the pick pose (token centre), r in cm where the pose and its 25 mm lift both exist")
heights = (-0.05, 0.0, 0.05, 0.10, 0.15, 0.20)
print("azimuth  " + "  ".join(f"z={z:+.2f}" for z in heights))
for azimuth in (40,):
    cells = []
    for z in heights:
        ok = []
        for r in np.arange(0.10, 0.42, 0.01):
            q = release_pose(kin, at(azimuth, r, z))
            up = None if q is None else release_pose(kin, at(azimuth, r, z + drop.PRE_DROP), q)
            if up is not None:
                ok.append(r)
        cells.append(f"{100 * ok[0]:.0f}-{100 * ok[-1]:.0f}" if ok else "  -  ")
    print(f"{azimuth:+4d}     " + "   ".join(f"{c:>6}" for c in cells))

print("== 2. the open jaws around the token at the pick pose (frame: r outwards, t across, z up; mm from the token centre)")
bodies = {model.body(n).id: n for n in ("gripper", "moving_jaw_so101_v1")}
for azimuth, r, z in ((40, 0.28, 0.05), (40, 0.25, 0.10)):
    q = release_pose(kin, at(azimuth, r, z))
    set_arm(q, p.open_pct)
    a = np.radians(azimuth)
    frame = np.array([[np.cos(a), np.sin(a), 0], [-np.sin(a), np.cos(a), 0], [0, 0, 1]])
    points = {}
    for g in range(model.ngeom):
        body = model.geom_bodyid[g]
        if body in bodies and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
            mesh = model.geom_dataid[g]
            v = model.mesh_vert[model.mesh_vertadr[mesh] : model.mesh_vertadr[mesh] + model.mesh_vertnum[mesh]]
            world = data.geom_xpos[g] + v @ data.geom_xmat[g].reshape(3, 3).T
            points.setdefault(bodies[body], []).append((world - at(azimuth, r, z)) @ frame.T * 1e3)
    print(f"azimuth {azimuth}, r {r}, z {z}: joints {np.round(q, 2)}, open {p.open_pct} %")
    for name, chunks in points.items():
        pts = np.vstack(chunks)
        print(f"  {name}: lowest z {pts[:, 2].min():+.0f}")
        for low, high in ((-30, -17), (-17, -7), (-7, 0), (0, 17), (17, 40)):
            band = pts[(pts[:, 2] >= low) & (pts[:, 2] < high)]
            if len(band):
                print(
                    f"    z {low:+4d}..{high:+4d}: r {band[:, 0].min():+5.0f}..{band[:, 0].max():+5.0f}"
                    f"   t {band[:, 1].min():+5.0f}..{band[:, 1].max():+5.0f}"
                )

print("== 3. the scene camera: pixel (640x480) of a station, and the view's angle to a marker's normal")
cam = model.camera("scene_camera").id if "scene_camera" in [model.camera(i).name for i in range(model.ncam)] else 0
print("camera:", model.camera(cam).name, "fovy", model.cam_fovy[cam])
mujoco.mj_forward(model, data)
pos, R = data.cam_xpos[cam], data.cam_xmat[cam].reshape(3, 3)  # columns: right, up, back
f = 240 / np.tan(np.radians(model.cam_fovy[cam]) / 2)
for azimuth in (20, 40, 60, 90, -70, -90):
    for r in (0.27, 0.40):
        point = at(azimuth, r, 0.05)
        c = R.T @ (point - pos)
        u, v = 320 + f * c[0] / -c[2], 240 - f * c[1] / -c[2]
        view = (pos - point) / np.linalg.norm(pos - point)
        a = np.radians(azimuth)
        side = np.array([-np.sin(a), np.cos(a), 0])  # the normal of a plate in the radial plane
        print(
            f"azimuth {azimuth:+4d} r {r}: pixel ({u:4.0f}, {v:4.0f}) at {np.linalg.norm(pos - point):.2f} m;"
            f" a flat marker seen at {np.degrees(np.arccos(view[2])):.0f} deg from its normal,"
            f" one on the rail's side at {np.degrees(np.arccos(abs(view @ side))):.0f} deg"
        )
sim.disconnect()
sys.exit(0)
