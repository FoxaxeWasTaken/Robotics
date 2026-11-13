"""e13_grid_clearance.py: how near the grid does a move of the game pass, and does the brain's model say what
MuJoCo says? (#260, MES-78)

The game's scene with the grid where the layout puts it, then set down elsewhere. For each move (a drop to
each column, a pick at each station) the arm is put, without dynamics, at every 0.02 rad of the joint lines
its steps follow, and two things are measured there:
- truth: MuJoCo's distance between the arm's collision geoms and every part of the grid, its posts and base
  included (the scene draws them without contacts: no game ever showed the arm touching them);
- the model: so101_brain.clearance, points on the convex hulls of the URDF's collision meshes against the
  grid's two boxes (so101_connect4.layout.grid_boxes).
Prints, per move, where it passes nearest the grid in truth, then how far the model is from the truth over
every pose within 10 cm of the grid.

Usage (a container with the workspace built): python3 e13_grid_clearance.py [moved] [wide N]
"""

import sys
import tempfile
import time
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[5] / "ros2_ws" / "src" / "so101_bringup" / "test"))
from test_connect4_dispenser import (  # noqa: E402
    ARM_BODIES,
    CAMERA,
    HANDOVER,
    LAYOUT,
    T_DISPENSER,
    URDF,
    Kinematics,
    Params,
    SO101Sim,
    board,
    drop,
    layout,
    pct_to_rad,
    set_arm,
    sim_scene,
)

from so101_brain import clearance  # noqa: E402

GRID = LAYOUT["grid"]
BOXES = np.array(layout.grid_boxes(LAYOUT))
kin, p, arm = Kinematics(str(URDF)), Params(), clearance.Arm(URDF)
STEP = 0.02  # rad between two poses of a joint line
OFF = []  # (model - truth, truth) of every pose within 10 cm of the grid, m


def placed(nearer=0.0, around=0.0, across=0.0) -> dict:
    """The layout with the grid `nearer` the arm along its line (m), carried `around` it (rad) and moved
    `across` its plane (m)."""
    place = dict(LAYOUT["placement"])
    place["first_column_r"] -= nearer
    place["azimuth"] += around
    place["plane_offset"] -= across
    return {**LAYOUT, "placement": place}


def moves(T) -> dict:
    """{name: its steps or None} of a game's moves on a grid at T."""
    handover = np.array(HANDOVER)
    out = {
        f"drop {column}": drop.drop_steps(kin, board.column_target(LAYOUT, T, column), handover, p)
        for column in range(GRID["columns"])
    }
    for rail in range(LAYOUT["dispenser"]["rails"]):
        out[f"pick {rail}"] = drop.pick_steps(kin, board.station_target(LAYOUT, T_DISPENSER, rail), handover, p)
    return out


def report(name, variant):
    T = layout.grid_pose(variant)
    sim = SO101Sim(
        scene_path=sim_scene.generate(tempfile.mkdtemp(), camera=CAMERA, connect4=variant),
        camera_width=64,
        camera_height=48,
    )
    sim.connect()
    model = sim.model
    bodies = {model.body(body).id for body in ARM_BODIES}
    arm_geoms = [g for g in range(model.ngeom) if model.geom_bodyid[g] in bodies and model.geom_contype[g]]
    plane = mujoco.mjtGeom.mjGEOM_PLANE
    grid_geoms = [
        g for g in range(model.ngeom) if model.geom_bodyid[g] == model.body("grid").id and model.geom_type[g] != plane
    ]
    for move, steps in moves(T).items():
        if steps is None:
            print(f"{name} | {move}: unreachable", flush=True)
            continue
        q, gripper, nearest = np.array(HANDOVER), p.open_pct, (np.inf, "", "")
        for step in steps:
            gripper = gripper if step.gripper is None else step.gripper
            goal = q if step.q is None else step.q
            count = max(1, int(np.ceil(np.max(np.abs(goal - q)) / STEP)))
            for k in range(1, count + 1):
                at = q + (goal - q) * k / count
                set_arm(sim, at)
                sim.data.qpos[model.joint("gripper").qposadr[0]] = pct_to_rad(gripper)
                mujoco.mj_forward(model, sim.data)
                gaps = [
                    (mujoco.mj_geomDistance(model, sim.data, a, g, 0.2, None), model.body(model.geom_bodyid[a]).name)
                    for a in arm_geoms
                    for g in grid_geoms
                ]
                truth, body = min(gaps)
                nearest = min(nearest, (truth, body, step.name))
                if truth < 0.1:
                    OFF.append((clearance.distance(arm.points(kin, at, gripper), T, BOXES) - truth, truth))
            q = goal
        start = time.perf_counter()
        judged = min(gap for _, gap in clearance.along(arm, kin, np.array(HANDOVER), steps, p.open_pct, T, BOXES))
        cost = time.perf_counter() - start
        print(
            f"{name} | {move}: {nearest[0] * 1e3:5.1f} mm ({nearest[1]}, in {nearest[2]}); "
            f"the brain says {judged * 1e3:5.1f} mm in {cost * 1e3:.0f} ms",
            flush=True,
        )
    sim.disconnect()


if __name__ == "__main__":
    report("as the layout puts it", placed())
    if "moved" in sys.argv:
        for mm in (20, 15, 10, -3, -5):
            report(f"{mm:+d} mm nearer", placed(nearer=mm * 1e-3))
        for degrees in (-10, 10):
            report(f"carried {degrees:+d} deg around the arm", placed(around=np.radians(degrees)))
        for mm in (-5, 5):
            report(f"{mm:+d} mm across", placed(across=mm * 1e-3))
        report("10 mm nearer, carried 3 deg", placed(nearer=0.010, around=np.radians(3.0)))
    if "wide" in sys.argv:
        rng = np.random.default_rng(0)
        for i in range(int(sys.argv[sys.argv.index("wide") + 1])):
            report(
                f"wide {i}",
                placed(rng.uniform(-0.04, 0.08), np.radians(rng.uniform(-60, 60)), rng.uniform(-0.03, 0.03)),
            )
    off = np.array(OFF)
    apart = off[off[:, 1] > 0.0, 0] * 1e3  # where the arm is not inside the grid
    print(
        f"model - truth over {len(apart)} poses the arm within 10 cm of the grid and out of it, mm: "
        f"min {apart.min():.1f}, p1 {np.percentile(apart, 1):.1f}, p50 {np.median(apart):.1f}, "
        f"p99 {np.percentile(apart, 99):.1f}, max {apart.max():.1f}"
    )
    inside = off[off[:, 1] <= 0.0]
    if len(inside):
        print(
            f"{len(inside)} poses the arm inside the grid in truth: the model says "
            f"{(inside[:, 0] + inside[:, 1]).max() * 1e3:.1f} mm at most there"
        )
