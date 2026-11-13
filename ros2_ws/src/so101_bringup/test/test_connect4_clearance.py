"""How near the grid the arm passes, on the game's scene (#260, phase2/P2-02 §2.5): the brain's model
(so101_brain.clearance: points on the hulls of the URDF's meshes, the grid as two boxes) against MuJoCo's own
distance between the arm's collision geoms and every part of the grid, its posts and base included (the scene
draws them without contacts, so no game ever showed the arm touching them).
- brain.yaml's connect4.grid_boxes are the layout's.
- The model says what MuJoCo says, within 3 mm.
- A game's moves keep twice the margin the brain asks; the grid 2 cm nearer, or 5 mm further, is refused.
The scene's placement is read here to build scenes and to score, as a test may (C-06)."""

from pathlib import Path

import mujoco
import numpy as np
import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from so101_sim import SO101Sim
from test_connect4_scene import ARM_BODIES, CAMERA, HANDOVER, LAYOUT, URDF, set_arm

from so101_brain import clearance, drop
from so101_brain.fsm import Params
from so101_brain.kinematics import Kinematics
from so101_bringup import sim_scene
from so101_common.robot import pct_to_rad
from so101_connect4 import board, layout

GRID = LAYOUT["grid"]
BOXES = np.array(layout.grid_boxes(LAYOUT))
T_DISPENSER = layout.dispenser_pose(LAYOUT)
P = Params()


@pytest.fixture(scope="module")
def kin():
    return Kinematics(str(URDF))


@pytest.fixture(scope="module")
def arm():
    return clearance.Arm(URDF)


def placed(nearer=0.0, around=0.0) -> dict:
    """The layout with the grid set down `nearer` the arm along its line (m) and carried `around` it (rad)."""
    place = dict(LAYOUT["placement"])
    place["first_column_r"] -= nearer
    place["azimuth"] += around
    return {**LAYOUT, "placement": place}


def moves(kin, variant) -> dict:
    """{name: its steps} of a game's moves on that layout, the poses as a perfect camera would give them."""
    T, handover = layout.grid_pose(variant), np.array(HANDOVER)
    out = {
        f"drop {column}": drop.drop_steps(kin, board.column_target(variant, T, column), handover, P)
        for column in range(GRID["columns"])
    }
    for rail in range(LAYOUT["dispenser"]["rails"]):
        target = board.station_target(variant, T_DISPENSER, rail)
        out[f"pick {rail}"] = drop.pick_steps(kin, target, handover, P)
    return out


def nearest(kin, arm, variant) -> dict:
    """{move: (m, the step)} where each move of a game passes nearest the grid, as the brain judges it."""
    T = layout.grid_pose(variant)
    out = {}
    for name, steps in moves(kin, variant).items():
        gaps = clearance.along(arm, kin, np.array(HANDOVER), steps, P.open_pct, T, BOXES)
        out[name] = min((gap, step) for step, gap in gaps)
    return out


def test_the_brains_boxes_are_the_layouts_grid():
    config = Path(get_package_share_directory("so101_bringup")) / "config" / "brain.yaml"
    brain = yaml.safe_load(config.read_text())["so101_brain"]["ros__parameters"]["connect4"]
    assert np.allclose(brain["grid_boxes"], BOXES.ravel(), atol=1e-9)


def test_the_model_says_what_mujoco_says_within_3_mm(tmp_path, kin, arm):
    """Along the moves of a game, the jaws open, part open or closed: the truth is MuJoCo's distance between
    the arm's collision geoms and the grid's geoms [MES-78]."""
    scene = sim_scene.generate(tmp_path, camera=CAMERA, connect4=LAYOUT)
    sim = SO101Sim(scene_path=scene, camera_width=64, camera_height=48)
    sim.connect()
    model, T = sim.model, layout.grid_pose(LAYOUT)
    bodies = {model.body(name).id for name in ARM_BODIES}
    arm_geoms = [g for g in range(model.ngeom) if model.geom_bodyid[g] in bodies and model.geom_contype[g]]
    grid = model.body("grid").id
    plane = (
        mujoco.mjtGeom.mjGEOM_PLANE
    )  # the markers' faces: no distance to a plane geom, and inside the body
    grid_geoms = [
        g for g in range(model.ngeom) if model.geom_bodyid[g] == grid and model.geom_type[g] != plane
    ]
    rng, off = np.random.default_rng(0), []
    for steps in moves(kin, LAYOUT).values():
        q = np.array(HANDOVER)
        for step in (s for s in steps if s.q is not None):
            at = q + (step.q - q) * rng.uniform(0.0, 1.0)
            opening = float(rng.choice([P.open_pct, drop.PICK_OPEN_PCT, P.close_pct]))
            set_arm(sim, at)
            sim.data.qpos[model.joint("gripper").qposadr[0]] = pct_to_rad(opening)
            mujoco.mj_forward(model, sim.data)
            truth = min(
                mujoco.mj_geomDistance(model, sim.data, a, g, 0.2, None)
                for a in arm_geoms
                for g in grid_geoms
            )
            if truth < 0.1:
                off.append(clearance.distance(arm.points(kin, at, opening), T, BOXES) - truth)
            q = step.q
    sim.disconnect()
    assert len(off) > 80
    print(f"model - MuJoCo over {len(off)} poses, mm: min {min(off) * 1e3:.1f}, max {max(off) * 1e3:.1f}")
    assert min(off) > -0.003
    assert max(off) < 0.003


def test_a_games_moves_keep_twice_the_margin_where_the_layout_puts_the_grid(kin, arm):
    gaps = nearest(kin, arm, LAYOUT)
    print({name: (round(gap * 1e3, 1), step) for name, (gap, step) in gaps.items()})
    assert min(gap for gap, _ in gaps.values()) > 2 * clearance.MARGIN


def test_the_grid_set_down_1_cm_nearer_and_carried_3_degrees_around_the_arm_is_played(kin, arm):
    """Where /sim/connect4/move_grid puts it between two games (test_connect4_game_launch)."""
    gaps = nearest(kin, arm, placed(nearer=0.010, around=np.radians(3.0)))
    assert min(gap for gap, _ in gaps.values()) > clearance.MARGIN + 0.003  # 3 mm for the camera's pose


@pytest.mark.parametrize(
    ("nearer", "move", "step"), [(0.020, "drop 3", "column"), (-0.005, "drop 6", "column")]
)
def test_a_grid_2_cm_nearer_or_5_mm_further_is_too_near_a_leg_of_a_drop(kin, arm, nearer, move, step):
    """Nearer, the shoulder passes 2 mm from the near post; further, the lower arm 3.5 mm above the guide's
    end at column 6 [MES-78]: every column still has a pose."""
    variant = placed(nearer=nearer)
    steps = moves(kin, variant)
    assert all(steps.values())
    gaps = clearance.along(
        arm, kin, np.array(HANDOVER), steps[move], P.open_pct, layout.grid_pose(variant), BOXES
    )
    assert next(name for name, gap in gaps if gap < clearance.MARGIN) == step  # the first leg too near
