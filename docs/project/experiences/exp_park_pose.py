"""PARK pose in sim: reached within the brain's 0.01 rad tolerance? Does the arm hide the cube or the box?
Feeds MES-39 (step 4.2a).

The earlier spikes set PARK in qpos directly; here the arm gets there through its actuators, behind the
driver's 50 Hz tick and speed limiter (6 s from rest). Then, over 50 scenes drawn as /sim/randomize does,
the cube and box pixels the external camera sees (MuJoCo segmentation) with the arm parked, against the
same scene with the arm lifted 10 m out of the picture.
"""
import sys

sys.path.insert(0, "/ros2_ws/src/so101_driver")
sys.path.insert(0, "/ros2_ws/src/so101_common")
import mujoco
import numpy as np
from so101_common.robot import ARM_JOINT_NAMES
from so101_driver.limiter import step_towards
from so101_driver.randomize import sample_box_xy
from so101_sim import SO101Sim

POSES = {"elbow 1.6 (06 §6 before)": [1.9, -1.7, 1.6, 0.7, 0.0], "elbow 1.5": [1.9, -1.7, 1.5, 0.7, 0.0]}
ARM = ("base", "shoulder", "upper_arm", "lower_arm", "wrist", "gripper")
sim = SO101Sim(); sim.connect()
m, d = sim.model, sim.data
renderer = mujoco.Renderer(m, height=sim.camera_height, width=sim.camera_width)
renderer.enable_segmentation_rendering()
cube, box, shoulder = m.body("pickup_object").id, m.body("drop_box").id, m.body("shoulder").id


def park(goal):
    d.qpos[:] = 0.0; d.qvel[:] = 0.0; mujoco.mj_forward(m, d)
    setpoint = dict.fromkeys(ARM_JOINT_NAMES, 0.0) | {"gripper": 100.0}
    target = dict(zip(ARM_JOINT_NAMES, goal)) | {"gripper": 100.0}
    for _ in range(300):
        setpoint = step_towards(setpoint, target, 0.02, 1.5, 200.0)
        sim.send_action({f"{n}.pos": v for n, v in setpoint.items()})
        obs = sim.get_observation()
    return np.array([obs[f"{n}.pos"] for n in ARM_JOINT_NAMES]) - goal


def self_contacts():
    arm = {m.body(name).id for name in ARM}
    pairs = [tuple(m.geom_bodyid[[c.geom1, c.geom2]]) for c in d.contact[:d.ncon]]
    return sorted({tuple(sorted(m.body(b).name for b in p)) for p in pairs if set(p) <= arm})


def visible():
    renderer.update_scene(d, camera=sim.camera_name)
    geom = renderer.render()[..., 0]
    body = np.where(geom >= 0, m.geom_bodyid[np.clip(geom, 0, None)], -1)
    return np.array([np.sum(body == cube), np.sum(body == box)])


for name, goal in POSES.items():
    error = park(goal)
    print(f"{name}: error {np.round(error, 4).tolist()} rad, self-contacts {self_contacts()}")
    sim._rng, box_rng = np.random.default_rng(0), np.random.default_rng(1)
    fractions = []
    for _ in range(50):
        m.body_pos[box, :2] = (2.0, 0.0)  # as /sim/randomize: box parked while the cube settles
        sim.randomize_object()
        m.body_pos[box, :2] = sample_box_xy(box_rng, d.body("pickup_object").xpos)
        mujoco.mj_forward(m, d)
        parked = visible()
        m.body_pos[shoulder, 2] += 10.0; mujoco.mj_forward(m, d)
        fractions.append(parked / visible())
        m.body_pos[shoulder, 2] -= 10.0; mujoco.mj_forward(m, d)
    f = np.array(fractions)
    for k, obj in enumerate(("cube", "box")):
        print(f"  {obj}: partly hidden in {np.sum(f[:, k] < 0.999)}/50 scenes, worst {f[:, k].min():.3f} visible")
