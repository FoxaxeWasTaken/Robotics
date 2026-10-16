"""Driver speed limiter in sim: how fast does a joint really move behind a 1.5 rad/s setpoint? Feeds MES-33.

Replays the driver's 50 Hz tick on SO101Sim without ROS (limiter, send_action, get_observation) and logs
shoulder_pan at every tick, in exact sim time (20 ms per tick), for a 0.5 rad step, with and without the
limiter. Sets the bound of the speed check in so101_driver/test/test_driver_launch.py.
"""
import sys

sys.path.insert(0, "/ros2_ws/src/so101_driver")
from so101_driver.limiter import step_towards
from so101_sim import SO101Sim

NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
sim = SO101Sim(); sim.connect()
dt_tick = 2 * sim.sim_dt_per_step * sim.model.opt.timestep   # send_action + get_observation per tick
obs = sim.get_observation()
goal = {n: obs[f"{n}.pos"] for n in NAMES}; sp = dict(goal)


def tick(setpoint):
    sim.send_action({f"{n}.pos": v for n, v in setpoint.items()})
    return sim.get_observation()


def cross(traj, level):
    for (ta, pa), (tb, pb) in zip(traj, traj[1:]):
        if pa < level <= pb:
            return ta + (level - pa) / (pb - pa) * (tb - ta)


for limited in (True, False):
    for _ in range(80):                                          # settle
        obs = tick(sp if limited else goal)
    p0 = obs["shoulder_pan.pos"]; goal["shoulder_pan"] = p0 + 0.5; traj = []
    for k in range(100):
        if limited:
            sp = step_towards(sp, goal, 0.02, 1.5, 200.0)
        obs = tick(sp if limited else goal)
        traj.append(((k + 1) * dt_tick, obs["shoulder_pan.pos"] - p0))
    v = [(pb - pa) / (tb - ta) for (ta, pa), (tb, pb) in zip(traj, traj[1:])]
    print(f"{'limiter   ' if limited else 'no limiter'}: mean 20-80 % {0.3 / (cross(traj, 0.40) - cross(traj, 0.10)):.2f} rad/s"
          f" | max per tick {max(v):.2f} rad/s | final {traj[-1][1]:.4f} rad (sim time per tick {dt_tick * 1e3:.0f} ms)")
    if limited:
        print("  per-tick speed:", " ".join(f"{x:.2f}" for x in v[:20]))
    sp = {n: obs[f"{n}.pos"] for n in NAMES}; goal = dict(sp)
