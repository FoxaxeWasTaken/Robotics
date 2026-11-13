"""rail.py [azimuth_deg r z slope_deg tokens roll_damping sleep open_pct nudge_N]: a spike of a gravity rail that feeds the robot its tokens.

The rail stands in the arm's own vertical plane (the jaws close across it, as at a drop): tokens on edge roll down
to a stop, the pick station, whose walls are low so that the jaws reach the token's faces. Measured here:
1. the feed: each time the station's token is taken away, does the next one come, and where does it stop;
2. the arm's clearance to the rail at the pick pose and on the way down;
3. a pick with the real contacts: descend with open jaws, close, lift; where the token sits in the jaws after.
"""

import sys
import tempfile
from pathlib import Path

import mujoco
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from so101_sim import ARM_JOINTS, SO101Sim

from so101_brain.drop import release_pose
from so101_brain.fsm import Params
from so101_brain.kinematics import PAN_AXIS_X, Kinematics
from so101_bringup import sim_scene
from so101_common.robot import pct_to_rad
from so101_connect4 import layout

AZIMUTH, R_STATION, Z_STATION, SLOPE, COUNT = (float(v) for v in (sys.argv[1:6] or (20, 0.27, 0.05, 10, 8)))
COUNT = int(COUNT)
share = Path(get_package_share_directory("so101_bringup")) / "config"
CAMERA = yaml.safe_load((share / "sim_scene.yaml").read_text())["camera"]
HANDOVER = yaml.safe_load((share / "brain.yaml").read_text())["so101_brain"]["ros__parameters"]["connect4"][
    "handover_pose"
]
LAYOUT = layout.load()
TOKEN, CONTACT = LAYOUT["token"], LAYOUT["contact"]
RADIUS, THICK = TOKEN["diameter"] / 2, TOKEN["thickness"]
GAP = THICK + 0.003  # the slot across: the grid's own slots leave 3 mm too
WALL, LOW, HIGH, STOP = 0.002, 0.007, 0.030, 0.010  # m: walls' thickness, their height at the station and behind
ARM_BODIES = ["shoulder", "upper_arm", "lower_arm", "wrist", "gripper", "moving_jaw_so101_v1"]

a, s = np.radians(AZIMUTH), np.radians(SLOPE)
e_r, e_t, e_z = np.array([np.cos(a), np.sin(a), 0]), np.array([-np.sin(a), np.cos(a), 0]), np.array([0, 0, 1.0])
station = np.array([PAN_AXIS_X, 0, Z_STATION]) + R_STATION * e_r  # the centre of the token to take
AXES = np.column_stack([np.cos(s) * e_r + np.sin(s) * e_z, e_t, -np.sin(s) * e_r + np.cos(s) * e_z])  # the rail's


def build() -> Path:
    path = sim_scene.generate(Path(tempfile.mkdtemp()), camera=CAMERA, connect4=LAYOUT)
    spec = mujoco.MjSpec.from_file(str(path))
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, AXES.ravel())
    body = spec.worldbody.add_body(name="rail", pos=station, quat=quat)
    solid = {
        "type": mujoco.mjtGeom.mjGEOM_BOX,
        "friction": [TOKEN["friction"], 0.005, 0.0001],
        "solref": [CONTACT["solref"], 1],
        "margin": 0,
        "condim": 3,
        "rgba": [0.85, 0.85, 0.9, 1],
    }
    length = COUNT * 2 * RADIUS + 0.02
    x0 = -RADIUS - 0.003  # the stop's outer face
    body.add_geom(name="rail_floor", size=[length / 2, GAP / 2 + WALL, 0.0015], pos=[x0 + length / 2, 0, -RADIUS - 0.0015], **solid)
    body.add_geom(name="rail_stop", size=[0.0015, 0.002, RADIUS], pos=[x0 + 0.0015, 0, 0], **solid)
    cut = RADIUS + 0.003  # the low walls end there: the station is open above them
    for side, name in ((1, "left"), (-1, "right")):
        y = side * (GAP / 2 + WALL / 2)
        body.add_geom(name=f"rail_low_{name}", size=[(cut - x0) / 2, WALL / 2, LOW / 2], pos=[(cut + x0) / 2, y, -RADIUS + LOW / 2], **solid)
        end = x0 + length
        body.add_geom(name=f"rail_high_{name}", size=[(end - cut) / 2, WALL / 2, HIGH / 2], pos=[(end + cut) / 2, y, -RADIUS + HIGH / 2], **solid)
    path.write_text(spec.to_xml())
    return path


sim = SO101Sim(scene_path=build(), camera_width=640, camera_height=480)
sim.connect()
model, data = sim.model, sim.data
kin, p = Kinematics(str(Path(get_package_share_directory("so101_description")) / "urdf" / "so101.urdf")), Params()
DT = model.opt.timestep
ROLL_DAMPING = float(sys.argv[6]) if len(sys.argv) > 6 else 0.0
SLEEP = int(sys.argv[7]) if len(sys.argv) > 7 else 1
OPEN = float(sys.argv[8]) if len(sys.argv) > 8 else p.open_pct
NUDGE = float(sys.argv[9]) if len(sys.argv) > 9 else 0.0
if ROLL_DAMPING >= 0:
    for i in range(21):
        dof = model.joint(f"token_yellow_{i}_free").dofadr[0]
        model.dof_damping[dof + 3 : dof + 6] = ROLL_DAMPING
if not SLEEP:
    model.opt.enableflags &= ~int(mujoco.mjtEnableBit.mjENBL_SLEEP)
print(f"roll damping {ROLL_DAMPING}, sleep {SLEEP}, jaws open at {OPEN} %")
start = data.qpos.copy()


def set_arm(q, gripper_pct, teleport=False):
    for name, value in zip([*ARM_JOINTS[:5], "gripper"], [*q, pct_to_rad(gripper_pct)], strict=True):
        data.ctrl[model.actuator(name).id] = value
        if teleport:
            data.qpos[model.joint(name).qposadr[0]] = value


def run(seconds):
    for _ in range(round(seconds / DT)):
        mujoco.mj_step(model, data)


def token(i):
    return f"token_yellow_{i}"


def in_rail(name):
    return AXES.T @ (data.body(name).xpos - station)


def load_rail():
    """The scene as it starts, COUNT yellow tokens on the rail, the arm at HANDOVER with open jaws."""
    data.qpos[:], data.qvel[:] = start, 0
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, np.column_stack([AXES[:, 0], -AXES[:, 2], AXES[:, 1]]).ravel())  # the disc's z across
    for i in range(COUNT):
        q = model.joint(f"{token(i)}_free").qposadr[0]
        data.qpos[q : q + 3] = station + AXES @ [i * (2 * RADIUS + 0.001) + 0.002, 0, 0.001]
        data.qpos[q + 3 : q + 7] = quat
    set_arm(HANDOVER, p.open_pct, teleport=True)
    mujoco.mj_forward(model, data)
    run(2.0)


def clearance():
    rail = [g for g in range(model.ngeom) if model.geom(g).name.startswith("rail_")]
    arm = {model.body(n).id for n in ARM_BODIES}
    arm_geoms = [g for g in range(model.ngeom) if model.geom_bodyid[g] in arm and model.geom_contype[g]]
    gaps = [(mujoco.mj_geomDistance(model, data, g, r, 0.05, None), model.geom(r).name) for g in arm_geoms for r in rail]
    return min(gaps)


print(f"rail: station at azimuth {AZIMUTH:.0f} deg, r {R_STATION} m, z {Z_STATION:+} m; slope {SLOPE:.0f} deg; {COUNT} tokens")
print("== the jaws' span across at the pick pose, mm from the token's centre, by opening (z -10..+20 mm)")
load_rail()
q_pick = release_pose(kin, data.body(token(0)).xpos.copy())
for pct in (30, 40, 50, 70, 100):
    set_arm(q_pick, pct, teleport=True)
    mujoco.mj_forward(model, data)
    spans = []
    for name in ("gripper", "moving_jaw_so101_v1"):
        chunks = []
        for g in range(model.ngeom):
            if model.geom_bodyid[g] == model.body(name).id and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
                mesh = model.geom_dataid[g]
                v = model.mesh_vert[model.mesh_vertadr[mesh] : model.mesh_vertadr[mesh] + model.mesh_vertnum[mesh]]
                world = data.geom_xpos[g] + v @ data.geom_xmat[g].reshape(3, 3).T
                chunks.append((world - data.body(token(0)).xpos) @ np.column_stack([e_r, e_t, e_z]) * 1e3)
        pts = np.vstack(chunks)
        band = pts[(pts[:, 2] > -10) & (pts[:, 2] < 20) & (pts[:, 0] > -30)]
        spans.append(f"{name[:6]} t {band[:, 1].min():+4.0f}..{band[:, 1].max():+4.0f}" if len(band) else f"{name[:6]} out of the band")
    print(f"  open {pct:3d} %: " + "; ".join(spans))

print("== every token of the rail picked in turn (mm): where it waited, how it is held, the feed after")
load_rail()
fed = held_ok = 0
for i in range(COUNT):
    waited = 0.0
    if NUDGE:  # the queue fell asleep while it followed the lifted token (wake_probe.py): a force wakes it
        for j in range(i, COUNT):
            data.qfrc_applied[model.joint(f"{token(j)}_free").dofadr[0] + 2] = -NUDGE
        run(DT)
        data.qfrc_applied[:] = 0
    while waited < 3.0 and (abs(in_rail(token(i))[0]) > 0.001 or np.linalg.norm(data.cvel[model.body(token(i)).id][3:]) > 0.005):
        run(0.1)
        waited += 0.1
    at = in_rail(token(i)) * 1e3
    arrived = abs(at[0]) < 1.0
    fed += arrived
    if not arrived:
        print(f"  token {i}: waited {waited:.1f} s at x {at[0]:+6.2f}  NOT FED")
        break
    first = data.body(token(i)).xpos.copy()
    way = [release_pose(kin, first + [0, 0, 0.005 * k]) for k in range(8, -1, -1)]
    set_arm(way[0], OPEN)
    run(1.5)
    closest = clearance()
    for q in way[1:]:
        set_arm(q, OPEN)
        run(0.15)
        closest = min(closest, clearance())
    run(0.3)
    set_arm(way[-1], p.close_pct)
    run(1.2)
    for q in way[-2::-1]:
        set_arm(q, p.close_pct)
        run(0.15)
    run(0.6)
    R = data.body("gripper").xmat.reshape(3, 3)
    held = R.T @ (data.body(token(i)).xpos - data.site("gripperframe").xpos) * 1e3
    tilt = np.degrees(np.arccos(min(1.0, abs(R[:, 0] @ data.body(token(i)).xmat.reshape(3, 3)[:, 2]))))
    rose = (data.body(token(i)).xpos[2] - first[2]) * 1e3
    held_ok += rose > 35
    print(
        f"  token {i}: waited {waited:.1f} s at x {at[0]:+6.2f} y {at[1]:+5.2f}{'' if arrived else '  NOT FED'}; arm to rail {1e3 * closest[0]:+.1f};"
        f" rose {rose:3.0f}; held {np.round(held, 1)}, tilt {tilt:.1f} deg"
    )
    set_arm(way[0], OPEN)  # the token dropped away from the rail: turn aside first
    aside = way[0].copy()
    aside[0] += 0.5
    set_arm(aside, p.close_pct)
    run(1.0)
    set_arm(aside, OPEN)
    run(0.8)
print(f"  fed {fed}/{COUNT}, held {held_ok}/{COUNT}")
sim.disconnect()

# The reference: a token given by the simulated hand at HANDOVER (the flow of the 50 drops of 50), closed on.
from so101_driver.connect4_sim import Connect4Sim  # noqa: E402

sim = SO101Sim(scene_path=build(), camera_width=640, camera_height=480)
sim.connect()
model, data = sim.model, sim.data
person = Connect4Sim(sim, LAYOUT, np.random.default_rng(0))
for trial in range(3):
    set_arm(HANDOVER, p.open_pct, teleport=True)
    mujoco.mj_forward(model, data)
    run(0.5)
    ok, name = person.give_token(p.open_pct)
    name = person.floating
    set_arm(HANDOVER, p.close_pct)
    for _ in range(120):
        run(0.02)
        person.release_if_pinched()
    R = data.body("gripper").xmat.reshape(3, 3)
    held = R.T @ (data.body(name).xpos - data.site("gripperframe").xpos) * 1e3
    tilt = np.degrees(np.arccos(min(1.0, abs(R[:, 0] @ data.body(name).xmat.reshape(3, 3)[:, 2]))))
    print(f"  hand-given token {trial}: in the jaws (x across, y, z) {np.round(held, 1)}, tilted {tilt:.1f} deg")
    person.reset_board() if hasattr(person, "reset_board") else None
    set_arm(HANDOVER, p.open_pct)
    run(1.0)
sim.disconnect()
