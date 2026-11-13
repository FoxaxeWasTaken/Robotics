"""wake_probe.py: what wakes a sleeping queue when its front token is gone? One variable at a time."""
import tempfile
from pathlib import Path

import mujoco
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from so101_sim import ARM_JOINTS, SO101Sim

from so101_bringup import sim_scene
from so101_connect4 import layout

share = Path(get_package_share_directory("so101_bringup")) / "config"
CAMERA = yaml.safe_load((share / "sim_scene.yaml").read_text())["camera"]
HANDOVER = yaml.safe_load((share / "brain.yaml").read_text())["so101_brain"]["ros__parameters"]["connect4"]["handover_pose"]
LAYOUT = layout.load()
T = np.linalg.inv(layout.dispenser_pose(LAYOUT))
scene = sim_scene.generate(Path(tempfile.mkdtemp()), camera=CAMERA, connect4=LAYOUT)


def probe(label, how):
    sim = SO101Sim(scene_path=scene, camera_width=640, camera_height=480)
    sim.connect()
    m, d = sim.model, sim.data
    for name, value in zip(ARM_JOINTS, HANDOVER, strict=False):
        d.qpos[m.joint(name).qposadr[0]] = value
        d.ctrl[m.actuator(name).id] = value
    mujoco.mj_forward(m, d)
    for _ in range(1500):
        mujoco.mj_step(m, d)
    asleep = [not d.body_awake[m.body(f"token_yellow_{i}").id] for i in range(7)]
    j = m.joint("token_yellow_0_free")
    q, v = j.qposadr[0], j.dofadr[0]
    how(m, d, q, v)
    for _ in range(750):
        mujoco.mj_step(m, d)
    x = (T @ [*d.body("token_yellow_1").xpos, 1.0])[0] * 1e3
    print(f"{label}: asleep before {all(asleep)}; token 1 at x {x:+.1f} mm after 1.5 s; awake after {bool(d.body_awake[m.body('token_yellow_1').id])}")
    sim.disconnect()


def qpos_only(m, d, q, v):
    d.qpos[q : q + 3] = [-0.6, -0.4, -0.05]


def qpos_qvel(m, d, q, v):
    d.qpos[q : q + 3] = [-0.6, -0.4, -0.05]
    d.qvel[v : v + 6] = 0


def qpos_forward(m, d, q, v):
    d.qpos[q : q + 3] = [-0.6, -0.4, -0.05]
    mujoco.mj_forward(m, d)


def lifted(m, d, q, v, nudge=0.0):  # as jaws would: carried straight up at 0.1 m/s for 40 mm, then put away
    for _ in range(700):  # 42 mm at 0.03 m/s, the speed of the brain's lift
        d.qvel[v : v + 6] = [0, 0, 0.03, 0, 0, 0]
        mujoco.mj_step(m, d)
    d.qpos[q : q + 3] = [-0.6, -0.4, -0.05]
    d.qvel[v : v + 6] = 0
    if nudge:
        for i in range(1, 7):
            d.qfrc_applied[m.joint(f"token_yellow_{i}_free").dofadr[0] + 2] = -nudge
        mujoco.mj_step(m, d)
        d.qfrc_applied[:] = 0


def lifted_and_nudged(m, d, q, v):
    lifted(m, d, q, v, 1e-6)


for label, how in (("qpos only", qpos_only), ("qpos and qvel", qpos_qvel), ("qpos then mj_forward", qpos_forward), ("carried up", lifted), ("carried up, then a nudge", lifted_and_nudged)):
    probe(label, how)
