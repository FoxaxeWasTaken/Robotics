"""e14_edge_on.py: how far from edge-on can the grid, or the dispenser, stand and the move still work? (#260,
MES-79)

The arm has 5 joints: it holds a token in its own vertical plane, the one through the pan axis. A slot turned
out of that plane gets a token turned by as much. The game's scene with the grid turned about one column (or
the dispenser about one station), the move aimed at the true pose, with the scene's contacts and the brain's
tick: a token given at HANDOVER and dropped, or taken at the station. For each turn: the angle between the
arm's plane at the release and the slot's, and what happened.

Usage (a container with the workspace built): python3 e14_edge_on.py grid|dispenser [degrees ...]
"""

import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[5] / "ros2_ws" / "src" / "so101_bringup" / "test"))
import test_connect4_dispenser as tests  # noqa: E402
from test_connect4_dispenser import (  # noqa: E402
    CAMERA,
    HANDOVER,
    LAYOUT,
    ROBOT,
    URDF,
    Connect4Sim,
    Kinematics,
    Params,
    SO101Sim,
    board,
    drop,
    layout,
    run,
    set_arm,
    sim_scene,
)

T_GRID, T_DISPENSER = layout.grid_pose(LAYOUT), layout.dispenser_pose(LAYOUT)
kin, p = Kinematics(str(URDF)), Params()
DEGREES = (-8, -6, -4, -3, -2, -1, 1, 2, 3, 4, 6, 8)


def about_z(angle, centre):
    c, s = np.cos(angle), np.sin(angle)
    T = np.eye(4)
    T[:2, :2] = [[c, -s], [s, c]]
    T[:3, 3] = centre - T[:3, :3] @ centre
    return T


def off_plane(T, q) -> float:
    """Degrees between the slot's plane (the x-z plane of the frame T) and the arm's at the joints q."""
    angle = np.arctan2(T[1, 0], T[0, 0]) + q[0]  # pan = -azimuth
    return float(np.degrees((angle + np.pi) % (2 * np.pi) - np.pi))


def scene(grid=T_GRID, dispenser=T_DISPENSER):
    sim_scene.connect4_layout.grid_pose = lambda layout_: grid  # the scene builds them there
    sim_scene.connect4_layout.dispenser_pose = lambda layout_: dispenser
    sim = SO101Sim(
        scene_path=sim_scene.generate(tempfile.mkdtemp(), camera=CAMERA, connect4=LAYOUT),
        camera_width=64,
        camera_height=48,
    )
    sim.connect()
    set_arm(sim, HANDOVER)
    person = Connect4Sim(sim, LAYOUT, np.random.default_rng(0))
    sim.data.ctrl[sim.model.actuator("gripper").id] = tests.pct_to_rad(p.open_pct)
    run(sim, person, 1.0)
    return sim, person


def drop_at(T, column) -> str:
    steps = drop.drop_steps(kin, board.column_target(LAYOUT, T, column), np.array(HANDOVER), p)
    if steps is None:
        return "unreachable"
    angle = off_plane(T, next(step.q for step in steps if step.name == "descent"))
    sim, person = scene(grid=T)
    assert person.give_token(100.0)[0]
    result, _, _ = tests.drive(sim, person, drop.Move(steps, p))
    run(sim, person, 1.0)
    landed = [i % 7 for i, v in enumerate(person.board()) if v == ROBOT]
    sim.disconnect()
    return f"{angle:+.2f} deg off: " + (result or ("done" if landed == [column] else f"token in {landed or 'no cell'}"))


def pick_at(T, rail) -> str:
    steps = drop.pick_steps(kin, board.station_target(LAYOUT, T, rail), np.array(HANDOVER), p)
    if steps is None:
        return "unreachable"
    angle = off_plane(T, next(step.q for step in steps if step.name == "seat"))
    sim, person = scene(dispenser=T)
    result, _, touched = tests.drive(sim, person, drop.Move(steps, p))
    name = tests.token(rail, 0)
    held = np.linalg.norm(sim.data.body(name).xpos - sim.data.site("gripperframe").xpos) * 1e3
    sim.disconnect()
    outcome = result or ("done" if held < 6.0 else f"token {held:.0f} mm from the jaws")
    return f"{angle:+.2f} deg off: {outcome}" + (f", touched {sorted(touched)}" if touched else "")


if __name__ == "__main__":
    what = sys.argv[1]
    degrees = [float(a) for a in sys.argv[2:]] or (0, *DEGREES)
    for a in degrees:
        if what == "grid":
            for column in (0, 3, 6):
                centre = (T_GRID @ [column * LAYOUT["grid"]["column_pitch"], 0.0, 0.0, 1.0])[:3]
                print(f"grid turned {a:+g} deg about column {column}: {drop_at(about_z(np.radians(a), centre) @ T_GRID, column)}", flush=True)
        else:
            for rail in range(LAYOUT["dispenser"]["rails"]):
                centre = (T_DISPENSER @ [*layout.station(LAYOUT, rail), 1.0])[:3]
                print(f"dispenser turned {a:+g} deg about station {rail}: {pick_at(about_z(np.radians(a), centre) @ T_DISPENSER, rail)}", flush=True)
