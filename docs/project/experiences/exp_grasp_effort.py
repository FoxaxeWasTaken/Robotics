"""Gripper effort where the FSM judges it (step 4.4a, #54): is grasp.effort_threshold = 0.06 well placed?

The repository's planner, min-jerk motions and the driver's limiter on SO101Sim in-process, the mechanics of
so101_bringup/test/test_grasp_closed_loop.py (imported, so the two cannot drift), ground-truth cube pose.
Per episode, the normalized gripper effort (so101_driver.effort, what /joint_states carries):
- GRASP window: every tick from grasp_wait_s to grasp_timeout_s after the close command (06 §3.1, the FSM
  goes to LIFT on the first sample above the threshold, to RETRY if none is);
- end of LIFT: the one sample the FSM judges once the lift is reached.
Three modes on the same seeded scenes: "aligned" (yaw known, jaws on a face), "free" (yaw unknown, free roll,
#128), "empty" (aligned plan, the cube moved 30 cm away before closing: /sim/randomize during APPROACH).
Feeds MES-48.

In a throwaway tek5/control container, the workspace built and sourced:
    python3 exp_grasp_effort.py [episodes, default 40]
"""

import sys

import numpy as np

sys.path.insert(0, "/ros2_ws/src/so101_bringup/test")
from test_grasp_closed_loop import BRAIN, CLOSED, DT, OPEN, URDF, Arm, cube_yaw, reset  # noqa: E402

from so101_brain.grasp_planner import GraspPlanner  # noqa: E402
from so101_brain.kinematics import Kinematics  # noqa: E402
from so101_driver.effort import normalized_effort  # noqa: E402
from so101_sim import SO101Sim  # noqa: E402

EPISODES = int(sys.argv[1]) if len(sys.argv) > 1 else 40
FSM = BRAIN["fsm"]
THRESHOLD = BRAIN["grasp"]["effort_threshold"]


def gripper_effort(sim):
    return normalized_effort(sim.model, sim.data, ["gripper"])[0]


def close_and_sample(arm):
    """Close command, then tick until grasp_timeout_s; the samples in [grasp_wait_s, grasp_timeout_s]."""
    arm.goal["gripper"] = CLOSED
    samples, t = [], 0.0
    while t < FSM["grasp_timeout_s"]:
        arm.tick()
        t += DT
        if t >= FSM["grasp_wait_s"]:
            samples.append(gripper_effort(arm.sim))
    return np.array(samples)


def episode(sim, planner, mode):
    """(grasp-window samples, end-of-lift effort or None, cube lifted) for one scene, or None without a plan."""
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
    if window.max() <= THRESHOLD:  # the FSM would RETRY here: no lift
        return window, None, False
    arm.move(plan.pre_grasp)
    arm.move(plan.lift)
    lifted = sim.data.body("pickup_object").xpos[2] > cube[2] + 0.05
    return window, gripper_effort(sim), lifted


def main():
    planner = GraspPlanner(Kinematics(str(URDF)))
    for mode in ("aligned", "free", "empty"):
        sim = SO101Sim()
        sim.connect()
        sim_seed, box_seed = np.random.SeedSequence(0).spawn(2)  # the driver's seed 0: the nightly's scenes
        sim._rng = np.random.default_rng(sim_seed)
        box_rng = np.random.default_rng(box_seed)
        firsts, maxes, lifts, held, no_plan, to_lift = [], [], [], 0, 0, 0
        for _ in range(EPISODES):
            reset(sim, box_rng)
            result = episode(sim, planner, mode)
            if result is None:
                no_plan += 1
                continue
            window, lift, lifted = result
            firsts.append(window[0])
            maxes.append(window.max())
            if lift is not None:
                to_lift += 1
                lifts.append(lift)
                held += lifted
        firsts, maxes = np.array(firsts), np.array(maxes)
        print(f"== {mode}: {len(maxes)} grasps ({no_plan} without a plan), threshold {THRESHOLD}")
        print(f"   GRASP window max:  min {maxes.min():.3f}  p5 {np.percentile(maxes, 5):.3f}  "
              f"median {np.median(maxes):.3f}  max {maxes.max():.3f}")
        print(f"   first sample (at grasp_wait_s): min {firsts.min():.3f}  median {np.median(firsts):.3f}")
        print(f"   above the threshold in the window: {np.sum(maxes > THRESHOLD)}/{len(maxes)}")
        if lifts:
            lifts = np.array(lifts)
            print(f"   end of LIFT: min {lifts.min():.3f}  median {np.median(lifts):.3f}  max {lifts.max():.3f}; "
                  f"above the threshold {np.sum(lifts > THRESHOLD)}/{to_lift}; cube really lifted {held}/{to_lift}")


if __name__ == "__main__":
    main()
