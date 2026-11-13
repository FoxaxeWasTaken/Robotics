"""The token dispenser on the game's scene (#257, ADR-041): three straight rails stacked in the arm's plane,
built by sim_scene from connect4_grid.yaml, the robot's 21 tokens waiting in them.
- The tokens rest on edge where layout.waiting puts them; the person's stay parked on the table.
- Each token of a rail comes to its station when the one before is taken: the driver's simulated scene wakes
  the queue (MuJoCo does not wake a sleeping body whose support is gone [MES-75]).
- The arm at a station's pick pose, and straight above it, touches nothing of the dispenser.
- The simulated hand takes its tokens at the top of a queue: the stations stay served.
The tokens' true poses are read here to score, as a test may (C-06)."""

import os
from pathlib import Path

import mujoco
import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory
from so101_sim import SO101Sim
from test_connect4_scene import ARM_BODIES, CAMERA, GRID, HANDOVER, LAYOUT, SCENE, T_GRID, URDF, set_arm

from so101_brain import drop
from so101_brain.drop import release_pose
from so101_brain.fsm import Params
from so101_brain.kinematics import Kinematics
from so101_bringup import sim_scene
from so101_common.robot import pct_to_rad
from so101_connect4 import board, layout
from so101_driver.camera import opencv_intrinsics
from so101_driver.connect4_sim import HUMAN, ROBOT, Connect4Sim
from so101_perception.calibration import detect_markers
from so101_perception.geometry import load_extrinsics

DISPENSER, TOKEN = LAYOUT["dispenser"], LAYOUT["token"]
RAILS, PER_RAIL = DISPENSER["rails"], DISPENSER["tokens_per_rail"]
T_DISPENSER = layout.dispenser_pose(LAYOUT)
COLORS = layout.colors(LAYOUT)
TICK = 10  # sim steps in one control tick of the driver (50 Hz)


@pytest.fixture(scope="module")
def scene(tmp_path_factory):
    return sim_scene.generate(tmp_path_factory.mktemp("scene"), camera=CAMERA, connect4=LAYOUT)


@pytest.fixture
def sim(scene):
    sim = SO101Sim(scene_path=scene, camera_width=640, camera_height=480)
    sim.connect()
    set_arm(sim, HANDOVER)
    yield sim
    sim.disconnect()


@pytest.fixture
def person(sim):
    return Connect4Sim(sim, LAYOUT, np.random.default_rng(0))


def token(rail, place) -> str:
    return f"token_{COLORS[ROBOT]}_{rail * PER_RAIL + place}"


def in_dispenser(sim, name) -> np.ndarray:
    return (np.linalg.inv(T_DISPENSER) @ [*sim.data.body(name).xpos, 1.0])[:3]


def run(sim, person, seconds):
    """Control ticks of the driver: the sim steps, then the simulated scene's own tick."""
    for _ in range(round(seconds / (TICK * sim.model.opt.timestep))):
        for _ in range(TICK):
            mujoco.mj_step(sim.model, sim.data)
        person.tick()


def off_place(sim, name, place) -> tuple:
    """(m along the rail and off its floor, m across the slot) between a token and a place of the dispenser
    frame: a token stands with 1.5 mm of play across, by design."""
    x, y, z = in_dispenser(sim, name) - place
    return float(np.hypot(x, z)), abs(float(y))


PLAY = (LAYOUT["dispenser"]["slot_gap"] - TOKEN["thickness"]) / 2 + 0.0002  # m across the slot


def take_away(sim, person, name):
    """As the jaws would: the token carried straight up out of its rail at the speed of the arm's lift, then
    put down far from it. Slow on purpose: the token behind follows the leaving rim, slower than the sleep
    tolerance, and falls asleep 3 mm further unless the simulated scene wakes it [MES-75]."""
    joint = sim.model.joint(f"{name}_free")
    q, v = joint.qposadr[0], joint.dofadr[0]
    for step in range(700):  # 42 mm at 0.03 m/s
        sim.data.qvel[v : v + 6] = [0, 0, 0.03, 0, 0, 0]
        mujoco.mj_step(sim.model, sim.data)
        if step % TICK == TICK - 1:
            person.tick()
    sim.data.qpos[q : q + 3] = [-0.6, -0.4, -LAYOUT["placement"]["riser"] + 0.05]
    sim.data.qvel[v : v + 6] = 0


def test_the_robots_tokens_wait_in_the_rails_on_edge_the_persons_stay_parked(sim, person):
    run(sim, person, 2.0)
    across = T_DISPENSER[:3, 1]
    for rail in range(RAILS):
        for place in range(PER_RAIL):
            name = token(rail, place)
            off, aside = off_place(sim, name, layout.waiting(LAYOUT, rail, place))
            assert off < 0.001, (rail, place, off)
            assert aside < PLAY, (rail, place, aside)
            axis = sim.data.body(name).xmat.reshape(3, 3)[:, 2]
            assert abs(axis @ across) > np.cos(np.radians(3.0)), (rail, place)  # on edge, in the rails' plane
    human = f"token_{COLORS[HUMAN]}_0"
    table = -LAYOUT["placement"]["riser"]
    assert abs(sim.data.body(human).xpos[2] - table - TOKEN["thickness"] / 2) < 0.002  # flat on the table
    assert np.linalg.norm(in_dispenser(sim, human)) > 0.3


def test_each_token_of_a_rail_comes_to_its_station_when_the_one_before_is_taken(sim, person):
    run(sim, person, 2.0)
    for rail in range(RAILS):
        for place in range(PER_RAIL):
            name = token(rail, place)
            off, aside = off_place(sim, name, layout.station(LAYOUT, rail))
            assert off < 0.001, (rail, place, off)
            assert aside < PLAY, (rail, place, aside)
            run(sim, person, 2.0)  # long enough for the queue to fall asleep: the wake is what brings it down
            assert not any(
                sim.data.body_awake[sim.model.body(token(rail, i)).id] for i in range(place, PER_RAIL)
            )
            take_away(sim, person, name)
            run(sim, person, 1.5)
    for rail in range(RAILS):  # nothing left behind in a rail
        for place in range(PER_RAIL):
            assert np.linalg.norm(in_dispenser(sim, token(rail, place))) > 0.3


def test_a_full_dispenser_at_rest_sleeps(sim, person):
    """The wake is for a queue that lost its front token: 21 tokens awake cost the control tick [MES-67]."""
    run(sim, person, 3.0)
    names = [token(rail, place) for rail in range(RAILS) for place in range(PER_RAIL)]
    assert [name for name in names if sim.data.body_awake[sim.model.body(name).id]] == []


def clearance(sim) -> tuple:
    """(mm, arm body, dispenser geom): the smallest distance between the arm and the dispenser."""
    model = sim.model
    parts = [g for g in range(model.ngeom) if model.geom(g).name.startswith("dispenser_")]
    arm = {model.body(name).id for name in ARM_BODIES}
    arm_geoms = [g for g in range(model.ngeom) if model.geom_bodyid[g] in arm and model.geom_contype[g]]
    assert parts
    assert arm_geoms
    gaps = [mujoco.mj_geomDistance(model, sim.data, a, g, 0.05, None) for a in arm_geoms for g in parts]
    pair = int(np.argmin(gaps))
    body = model.body(model.geom_bodyid[arm_geoms[pair // len(parts)]]).name
    return round(min(gaps) * 1e3, 1), body, model.geom(parts[pair % len(parts)]).name


@pytest.mark.parametrize("open_pct", [40.0, 100.0])
def test_the_arm_at_a_station_and_straight_above_it_touches_nothing_of_the_dispenser(sim, open_pct):
    kin = Kinematics(str(URDF))
    gripper = sim.model.joint("gripper").qposadr[0]
    closest = {}
    for rail in range(RAILS):
        target = (T_DISPENSER @ [*layout.station(LAYOUT, rail), 1.0])[:3]
        for lift in (0.0, 0.01, 0.02, 0.03, 0.04):
            q = release_pose(kin, target + np.array([0.0, 0.0, lift]))
            assert q is not None, (rail, lift)  # each station within the pick pose's reach [MES-75]
            sim.data.qpos[gripper] = pct_to_rad(open_pct)
            set_arm(sim, q)
            closest[rail, lift] = clearance(sim)
    print("arm to dispenser, mm:", closest)
    assert min(gap for gap, *_ in closest.values()) > 2.0, closest


def test_the_simulated_hand_takes_its_tokens_at_the_top_of_a_queue(sim, person):
    """With token_source: hand the stations stay served, and a queue keeps no hole."""
    run(sim, person, 2.0)
    assert person.drop_token(3, ROBOT)[0]

    def left(rail, place):
        return off_place(sim, token(rail, place), layout.waiting(LAYOUT, rail, place))[0] > 0.05

    taken = [(rail, place) for rail in range(RAILS) for place in range(PER_RAIL) if left(rail, place)]
    assert taken == [(0, PER_RAIL - 1)]  # the top of the fullest queue, the lowest rail first
    assert person.drop_token(4, ROBOT)[0]
    assert left(1, PER_RAIL - 1)
    run(sim, person, 1.0)
    for rail in range(RAILS):
        assert off_place(sim, token(rail, 0), layout.station(LAYOUT, rail))[0] < 0.001, rail


def test_a_reset_puts_every_token_back_in_its_rail(sim, person):
    run(sim, person, 2.0)
    for place in range(3):
        take_away(sim, person, token(0, place))
        run(sim, person, 1.0)
    person.reset_board()
    run(sim, person, 2.0)
    for place in range(PER_RAIL):
        assert off_place(sim, token(0, place), layout.waiting(LAYOUT, 0, place))[0] < 0.001, place


def test_a_whole_drop_from_handover_stays_clear_of_the_dispenser(sim):
    """With token_source: hand the arm never goes to the dispenser: HANDOVER and the legs of a drop to the
    nearest and the farthest column pass it by."""
    kin, p = Kinematics(str(URDF)), Params()
    gripper = sim.model.joint("gripper").qposadr[0]
    closest = {"handover": clearance(sim)}
    for column in (0, GRID["columns"] - 1):
        steps = drop.drop_steps(kin, board.column_target(LAYOUT, T_GRID, column), np.array(HANDOVER), p)
        move, q, tick = drop.Move(steps, p), np.array(HANDOVER), 0
        while True:
            out = move.step(tick * 0.02, q, 0.5, 0.0)  # a token held: the closing effort
            q = q if out.command is None else out.command
            if out.gripper is not None:
                sim.data.qpos[gripper] = pct_to_rad(out.gripper)
            if tick % 2 == 0 or out.result is not None:
                set_arm(sim, q)
                closest[column] = min(closest.get(column, (np.inf,)), clearance(sim))
            if out.result is not None:
                break
            tick += 1
        assert out.result == "", (column, out.result)
    print("handover and drops, arm to dispenser, mm:", closest)
    assert min(gap for gap, *_ in closest.values()) > 10.0, closest


# The dispenser read by the camera (#258): its pose from its own markers, its stations by the tokens' color.

PLACES = int(os.environ.get("CONNECT4_DISPENSER_PLACES", "3"))
EXTRINSICS = Path(get_package_share_directory("so101_perception")) / "config" / "camera_extrinsics.yaml"


def moved(rng) -> dict:
    """The layout with the dispenser set down a little elsewhere: +-3 deg, +-10 mm out, +-10 mm up."""
    place = dict(DISPENSER["placement"])
    place["azimuth"] += np.radians(rng.uniform(-3.0, 3.0))
    place["station_r"] += rng.uniform(-0.01, 0.01)
    place["station_z"] += rng.uniform(-0.01, 0.01)
    return {**LAYOUT, "dispenser": {**DISPENSER, "placement": place}}


def view(folder, variant, lighting=None):
    """(sim, an image of the scene camera at 640x480, K): the arm at HANDOVER, the tokens settled."""
    scene = sim_scene.generate(folder, camera=CAMERA, connect4=variant, lighting=lighting)
    sim = SO101Sim(scene_path=scene, camera_width=640, camera_height=480)
    sim.connect()
    set_arm(sim, HANDOVER)
    for _ in range(500):
        mujoco.mj_step(sim.model, sim.data)
    return sim, opencv_intrinsics(sim.get_camera_intrinsics())


def image(sim):
    renderer = mujoco.Renderer(sim.model, height=480, width=640)
    renderer.update_scene(sim.data, camera=sim.camera_name)
    rgb = renderer.render()
    renderer.close()
    return rgb


@pytest.mark.parametrize("light", ["default", "dim"])
def test_the_camera_places_each_station_within_2_mm_wherever_the_dispenser_stands(tmp_path, light):
    """The pose from the dispenser's markers, with our calibration of the phase-1 camera: where it says a
    station is against where the scene has it. The scene's placement only scores (C-06)."""
    lighting = None if light == "default" else SCENE["lighting"]["dim"]
    rng = np.random.default_rng(258)
    T_base_cam = load_extrinsics(EXTRINSICS).T_base_cam
    errors, reprojections = [], []
    for k in range(PLACES):
        variant = LAYOUT if k == 0 else moved(rng)
        sim, K = view(tmp_path / f"scene_{k}", variant, lighting)
        found = detect_markers(image(sim), LAYOUT["markers"]["dictionary"])
        sim.disconnect()
        pose = board.dispenser_in_camera(found, variant, K)
        assert pose is not None, (k, sorted(found))  # from 3 markers or 4, as the reader takes it
        T_measured, T_true = T_base_cam @ pose[0], layout.dispenser_pose(variant)
        for rail in range(RAILS):
            there = [*layout.station(variant, rail), 1.0]
            errors.append(np.linalg.norm((T_measured @ there - T_true @ there)[:3]) * 1e3)
        reprojections.append(pose[1])
    print(
        f"dispenser pose, {light} light, {PLACES} placements: station error max {max(errors):.2f} mm, "
        f"mean {np.mean(errors):.2f} mm; reprojection max {max(reprojections):.2f} px"
    )
    assert max(errors) < 2.0, errors


def test_the_camera_reads_which_stations_hold_a_token(tmp_path):
    sim, K = view(tmp_path, LAYOUT)
    found = detect_markers(image(sim), LAYOUT["markers"]["dictionary"])
    T_cam_dispenser = board.dispenser_in_camera(found, LAYOUT, K)[0]
    assert board.stations(image(sim), LAYOUT, T_cam_dispenser, K) == (True, True, True)
    for place in range(PER_RAIL):  # the middle rail emptied
        q = sim.model.joint(f"{token(1, place)}_free").qposadr[0]
        sim.data.qpos[q : q + 3] = [-0.6, -0.4 - 0.04 * place, -LAYOUT["placement"]["riser"] + 0.01]
    mujoco.mj_forward(sim.model, sim.data)
    assert board.stations(image(sim), LAYOUT, T_cam_dispenser, K) == (True, False, True)
    sim.disconnect()


def test_the_grids_markers_are_still_the_four_the_reader_needs(tmp_path):
    sim, K = view(tmp_path, LAYOUT)
    found = detect_markers(image(sim), LAYOUT["markers"]["dictionary"])
    sim.disconnect()
    assert board.grid_in_camera(found, LAYOUT, K)[2] == LAYOUT["markers"]["ids"]
