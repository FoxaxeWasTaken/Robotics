"""Closed-loop grasp validation in sim (docs/project/04 §5, ADR-009, step 2.4b): the repository's planner
and trajectories grasp the cube and drop it in the box when perception is perfect (ground truth, allowed in
tests).

In-process, no ROS: each 20 ms tick does what the stack does, the brain's min-jerk sample, then the driver's
speed limiter, send_action and get_observation. Parameters are the deployed ones: so101_bringup/config,
brain_node's constants and the planner's defaults (06 §6). Sim-backed, so here and not in so101_brain (C-05).
"""

import time
from pathlib import Path

import mujoco
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from so101_sim import SO101Sim

from so101_brain.brain_node import MIN_MOTION_DURATION, TIMEOUT_MARGIN
from so101_brain.grasp_planner import GraspPlanner
from so101_brain.kinematics import Kinematics
from so101_brain.trajectory import min_jerk
from so101_common.robot import ARM_JOINT_NAMES
from so101_driver.limiter import step_towards
from so101_driver.randomize import sample_box_xy

CONFIG = Path(get_package_share_directory("so101_bringup")) / "config"
BRAIN = yaml.safe_load((CONFIG / "brain.yaml").read_text())["so101_brain"]["ros__parameters"]
DRIVER = yaml.safe_load((CONFIG / "driver.yaml").read_text())["so101_driver"]["ros__parameters"]
URDF = Path(get_package_share_directory("so101_description")) / "urdf" / "so101.urdf"
DT = 1.0 / DRIVER["control_rate_hz"]  # send_action and get_observation step 10 ms of physics each
OPEN, CLOSED = 100.0, 0.0  # grasp.open_pct / close_pct (06 §6)
EPISODES = 40


class Arm:
    """The sim behind the driver's tick, commanded like the brain: one min-jerk motion at a time."""

    def __init__(self, sim):
        self.sim = sim
        self.setpoint = {name: 0.0 for name in ARM_JOINT_NAMES} | {"gripper": OPEN}
        self.goal = dict(self.setpoint)
        self.obs = sim.get_observation()
        self.motion_s = 0.0  # time spent streaming motions

    def q(self):
        return np.array([self.obs[f"{name}.pos"] for name in ARM_JOINT_NAMES])

    def tick(self):
        self.setpoint = step_towards(
            self.setpoint, self.goal, DT, DRIVER["max_joint_velocity"], DRIVER["max_gripper_velocity"]
        )
        self.sim.send_action({f"{name}.pos": value for name, value in self.setpoint.items()})
        self.obs = self.sim.get_observation()

    def move(self, q_goal):
        """Streams the trajectory from the measured joints; True when reached, False at T + margin (04 §6)."""
        trajectory = min_jerk(self.q(), q_goal, BRAIN["motion"]["v_max_rad_s"], MIN_MOTION_DURATION)
        t = 0.0
        while True:
            self.goal.update(zip(ARM_JOINT_NAMES, trajectory.sample(t), strict=True))
            self.tick()
            t += DT
            self.motion_s += DT
            if (
                t >= trajectory.duration
                and np.max(np.abs(self.q() - q_goal)) < BRAIN["motion"]["reach_tol_rad"]
            ):
                return True
            if t > trajectory.duration + TIMEOUT_MARGIN:
                return False

    def gripper(self, pct, seconds):
        self.goal["gripper"] = pct
        for _ in range(round(seconds / DT)):
            self.tick()


def reset(sim, box_rng):
    """Arm back at rest, then what /sim/randomize does: box parked, cube randomized and settled, box drawn."""
    for name in ARM_JOINT_NAMES:
        sim.data.qpos[sim.model.joint(name).qposadr[0]] = 0.0
    sim.data.qvel[:] = 0.0
    sim.data.ctrl[:] = 0.0
    box = sim.model.body("drop_box").id
    sim.model.body_pos[box, :2] = (2.0, 0.0)
    sim.randomize_object()
    sim.model.body_pos[box, :2] = sample_box_xy(box_rng, sim.data.body("pickup_object").xpos)
    mujoco.mj_forward(sim.model, sim.data)


def cube_yaw(sim):
    w, x, y, z = sim.data.body("pickup_object").xquat
    return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def test_40_seeded_episodes_put_the_cube_in_the_box():
    start = time.monotonic()
    sim = SO101Sim()
    sim.connect()
    sim_seed, box_seed = np.random.SeedSequence(0).spawn(2)  # as the driver's seed parameter (03 §7)
    sim._rng = np.random.default_rng(sim_seed)
    box_rng = np.random.default_rng(box_seed)
    planner = GraspPlanner(Kinematics(str(URDF)))
    successes = 0
    for episode in range(1, EPISODES + 1):
        reset(sim, box_rng)
        arm = Arm(sim)
        cube = sim.data.body("pickup_object").xpos.copy()  # ground truth: perfect perception
        box = sim.data.body("drop_box").xpos.copy()
        plan, reason = planner.plan(cube, cube_yaw(sim), box, arm.q())
        if plan is None:
            print(f"episode {episode}: no plan ({reason}), cube {cube.round(3)}")
            continue
        arm.gripper(OPEN, 0.5)
        reached = [arm.move(plan.pre_grasp), arm.move(plan.grasp)]
        arm.gripper(CLOSED, 1.0)
        reached += [arm.move(plan.pre_grasp), arm.move(plan.lift), arm.move(plan.drop)]
        arm.gripper(OPEN, 1.0)
        arm.gripper(OPEN, 1.5)  # the cube settles
        final = sim.data.body("pickup_object").xpos
        in_box = abs(final[0] - box[0]) < 0.05 and abs(final[1] - box[1]) < 0.05 and final[2] < 0.06  # 08 §3
        successes += in_box
        print(
            f"episode {episode}: {'OK' if in_box else 'MISS'}, r {np.hypot(*cube[:2]):.3f} m, "
            f"tilt {plan.tilt_deg:.0f}°, {'aligned' if plan.aligned else 'free'}, "
            f"motions {arm.motion_s:.1f} s, all reached: {all(reached)}, cube at {np.round(final, 3)}"
        )
    print(f"closed loop: {successes}/{EPISODES} in the box, {time.monotonic() - start:.0f} s")
    assert successes >= 38  # spike: 39/40 [MES-09]
