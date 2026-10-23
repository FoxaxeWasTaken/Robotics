"""Position IK solutions that collide with the arm itself, and the seed of test_ik_accuracy. Feeds MES-43 (#142).

/go_to_target solves the position only (ikpy, no collision volumes) from the measured joints. Over the 20 seeded
targets of test_ik_accuracy (obstacles of the seed-0 /sim/randomize), each seed below gets 5 solves with 1e-4 rad
of noise, the settling noise between runs. A solution is bad when it folds the elbow past 1.6 rad or when MuJoCo
finds a contact on the arm (the cube and box of the default scene are left out: they are not where the bench
puts them). Also: the contacts of the configuration that stayed 28.4 mm short of target 4.
"""
import sys

sys.path.insert(0, "/ros2_ws/src/so101_brain")
sys.path.insert(0, "/ros2_ws/src/so101_common")
import mujoco
import numpy as np
from so101_brain.kinematics import Kinematics
from so101_common.robot import ARM_JOINT_NAMES
from so101_sim import DEFAULT_SCENE

URDF = "/ros2_ws/src/so101_description/urdf/so101.urdf"
OBSTACLES = {"object": np.array([0.283, -0.077]), "drop_box": np.array([0.087, -0.246])}
CLEARANCE = {"object": 0.08, "drop_box": 0.12}  # as test_ik_accuracy
SEEDS = {"q = 0 (start pose, arm straight)": [0, 0, 0, 0, 0], "HOME of the test": [0, -1.0, 0.7, 0, 0]}


def targets(n):  # test_ik_accuracy.targets, same seed
    rng, chosen = np.random.default_rng(41), []
    while len(chosen) < n:
        r, a, z = rng.uniform(0.16, 0.30), np.radians(rng.uniform(-90.0, 60.0)), rng.uniform(0.03, 0.12)
        p = np.array([r * np.cos(a), r * np.sin(a), z])
        if all(np.linalg.norm(p[:2] - xy) > CLEARANCE[k] for k, xy in OBSTACLES.items()):
            chosen.append(p)
    return chosen


kin = Kinematics(URDF)
m = mujoco.MjModel.from_xml_path(str(DEFAULT_SCENE)); d = mujoco.MjData(m)
name = lambda b: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b)  # noqa: E731
props = {m.body("pickup_object").id, m.body("drop_box").id}


def arm_contacts(q):
    for n, v in zip(ARM_JOINT_NAMES, q, strict=True):
        d.qpos[m.joint(n).qposadr[0]] = v
    mujoco.mj_forward(m, d)
    pairs = {tuple(sorted((name(m.geom_bodyid[c.geom1]), name(m.geom_bodyid[c.geom2]))))
             for c in d.contact[: d.ncon] if not {m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]} & props}
    return sorted(pairs), d.site("gripperframe").xpos.copy()


print("stopped short of target 4 (elbow 1.5933, command 1.6898):",
      arm_contacts([-0.4215, -1.5059, 1.5933, -0.0752, 0.3238])[0])
rng, elbow, ts = np.random.default_rng(1), ARM_JOINT_NAMES.index("elbow_flex"), targets(20)
for label, seed in SEEDS.items():
    bad = []
    for i, t in enumerate(ts, start=1):
        for _ in range(5):
            q = kin.ik_pos(t, q_seed=np.array(seed, float) + rng.normal(0.0, 1e-4, 5))
            contacts = arm_contacts(q)[0]
            if abs(q[elbow]) > 1.6 or contacts:
                bad.append(f"target {i}: elbow {q[elbow]:.3f}, {contacts}"); break
    home_contacts, tcp = arm_contacts(seed)
    print(f"{label} {seed}: TCP {tcp.round(3)}, contacts {home_contacts}; bad solutions: {len(bad)}/20 {bad}")
