"""Gripper opening where the FSM judges the grasp (#69): can the measured position tell a held cube from an
empty close, as the real arm needs (MES-56: its Present_Load cannot)?

The mechanics of exp_grasp_effort.py (MES-48): the repository's planner, min-jerk motions and the driver's
limiter on SO101Sim in-process, ground-truth cube pose, the 40 scenes of the driver's seed 0. Per episode, the
measured gripper opening in % (get_observation()["gripper.pos"], what /joint_states carries) at every tick of
the GRASP window (grasp_wait_s to grasp_timeout_s after the close command) and once LIFT is reached.
Modes: "aligned" (jaws on a face), "free" (yaw unknown, free roll), "empty" (the cube moved 30 cm away before
closing). Each close goes to CLOSE_PCT (default 0, the sim's grasp.close_pct). Feeds MES-57.

In a throwaway tek5/control container, the workspace built and sourced:
    python3 exp_grasp_position.py [episodes, default 40] [close_pct, default 0]
"""

import sys

import numpy as np

sys.path.insert(0, "/ros2_ws/src/so101_bringup/test")
from test_grasp_closed_loop import BRAIN, DT, OPEN, URDF, Arm, cube_yaw, reset  # noqa: E402

from so101_brain.grasp_planner import GraspPlanner  # noqa: E402
from so101_brain.kinematics import Kinematics  # noqa: E402
from so101_sim import SO101Sim  # noqa: E402

EPISODES = int(sys.argv[1]) if len(sys.argv) > 1 else 40
CLOSE_PCT = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
FSM = BRAIN["fsm"]


def close_and_sample(arm):
    """Close command, then tick until grasp_timeout_s; the measured openings in [grasp_wait_s, timeout]."""
    arm.goal["gripper"] = CLOSE_PCT
    samples, t = [], 0.0
    while t < FSM["grasp_timeout_s"]:
        arm.tick()
        t += DT
        if t >= FSM["grasp_wait_s"]:
            samples.append(arm.obs["gripper.pos"])
    return np.array(samples)


def episode(sim, planner, mode):
    """(grasp-window openings, end-of-lift opening, cube lifted) for one scene, or None without a plan."""
    arm = Arm(sim)
    cube = sim.data.body("pickup_object").xpos.copy()
    box = sim.data.body("drop_box").xpos.copy()
    plan, _ = planner.plan(cube, None if mode == "free" else cube_yaw(sim), box, arm.q())
    if plan is None:
        return None
    arm.gripper(OPEN, 0.5)
    arm.move(plan.pre_grasp)
    arm.move(plan.grasp)
    if mode == "empty":  # the cube leaves between the plan and the close
        adr = sim.model.joint("pickup_object_free").qposadr[0]
        sim.data.qpos[adr : adr + 2] += 0.30 * cube[:2] / np.linalg.norm(cube[:2])
        sim.data.qvel[:] = 0.0
    window = close_and_sample(arm)
    arm.move(plan.pre_grasp)
    arm.move(plan.lift)
    lifted = sim.data.body("pickup_object").xpos[2] > cube[2] + 0.05
    return window, arm.obs["gripper.pos"], lifted


def stats(values):
    v = np.asarray(values)
    return f"min {v.min():.2f}  p5 {np.percentile(v, 5):.2f}  median {np.median(v):.2f}  max {v.max():.2f}"


def main():
    planner = GraspPlanner(Kinematics(str(URDF)))
    print(f"close_pct {CLOSE_PCT:g} %, window {FSM['grasp_wait_s']}–{FSM['grasp_timeout_s']} s")
    for mode in ("aligned", "free", "empty"):
        sim = SO101Sim()
        sim.connect()
        sim_seed, box_seed = np.random.SeedSequence(0).spawn(2)  # the driver's seed 0: the nightly's scenes
        sim._rng = np.random.default_rng(sim_seed)
        box_rng = np.random.default_rng(box_seed)
        lows, lifts, held, no_plan = [], [], [], 0
        for _ in range(EPISODES):
            reset(sim, box_rng)
            result = episode(sim, planner, mode)
            if result is None:
                no_plan += 1
                continue
            window, lift, lifted = result
            lows.append(window.min())  # the most closed the jaws got in the window
            lifts.append(lift)
            held.append(lifted)
        held = np.array(held)
        print(f"== {mode}: {len(lows)} grasps ({no_plan} without a plan), cube really lifted {held.sum()}")
        print(f"   GRASP window, lowest opening (%): {stats(lows)}")
        print(f"   end of LIFT opening (%):          {stats(lifts)}")
        if held.any() and (~held).any():
            lows, lifts = np.array(lows), np.array(lifts)
            print(f"   lifted: lowest in window {stats(lows[held])}; end of lift {stats(lifts[held])}")
            print(f"   not lifted: lowest in window {stats(lows[~held])}; end of lift {stats(lifts[~held])}")


if __name__ == "__main__":
    main()
