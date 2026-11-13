"""Reading the Connect Four grid on the game's scene (#82, target P2-01 of phase2/P2-01 §4): random states of
a game, the tokens settled in the slots, the arm at HANDOVER with a token in its jaws; the phase-1 camera at
640x480, the markers, the grid's pose and the 42 cells through so101_connect4.board, as board_reader_node
does.
Every cell of every state is read, and the pose is the scene's within 5 mm. The truth (the scene's board and
the grid's body) only scores (C-06). CONNECT4_READING_STATES=50 runs the measurement of MES-68."""

import os
from pathlib import Path

import mujoco
import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory
from so101_sim import SO101Sim
from test_connect4_scene import CAMERA, HANDOVER, LAYOUT, SCENE, T_GRID, set_arm

from so101_bringup import sim_scene
from so101_common.robot import pct_to_rad
from so101_connect4 import board, layout
from so101_driver.camera import opencv_intrinsics
from so101_driver.connect4_sim import Connect4Sim
from so101_perception.calibration import detect_markers
from so101_perception.geometry import load_extrinsics

STATES = int(os.environ.get("CONNECT4_READING_STATES", "10"))
EXTRINSICS = Path(get_package_share_directory("so101_perception")) / "config" / "camera_extrinsics.yaml"
DISC_IN_SLOT = np.array([[1.0, 0, 0], [0, 0, 1], [0, -1, 0]])


@pytest.fixture(scope="module", params=["default", "dim"])
def game(tmp_path_factory, request):
    """The game's scene under the default light, then under the robustness demo's dim, warm one (g5)."""
    lighting = None if request.param == "default" else SCENE["lighting"]["dim"]
    folder = tmp_path_factory.mktemp("scene")
    scene = sim_scene.generate(folder, camera=CAMERA, connect4=LAYOUT, lighting=lighting)
    sim = SO101Sim(scene_path=scene, camera_width=640, camera_height=480)
    sim.connect()
    renderer = mujoco.Renderer(sim.model, height=480, width=640)
    yield sim, Connect4Sim(sim, LAYOUT, np.random.default_rng(0)), renderer
    renderer.close()
    sim.disconnect()


def step(sim, seconds, person=None):
    for _ in range(round(seconds / sim.model.opt.timestep)):
        mujoco.mj_step(sim.model, sim.data)
        if person:
            person.release_if_pinched()


def set_state(sim, person, rng, moves):
    """A game after `moves` moves: the tokens at rest in their cells, a yellow one held at HANDOVER."""
    person.reset_board()
    set_arm(sim, HANDOVER)
    gripper = sim.model.joint("gripper").qposadr[0]
    sim.data.qpos[gripper] = sim.data.ctrl[sim.model.actuator("gripper").id] = pct_to_rad(60.0)
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, (T_GRID[:3, :3] @ DISC_IN_SLOT).ravel())
    heights, used, player = [0] * 7, {1: 0, 2: 0}, 1
    for _ in range(moves):
        column = int(rng.choice([c for c in range(7) if heights[c] < 6]))
        token = f"token_{'red' if player == 1 else 'yellow'}_{used[player]}"
        q = sim.model.joint(f"{token}_free").qposadr[0]
        jitter = [rng.uniform(-0.001, 0.001), rng.uniform(-0.001, 0.001), 0.0005]  # in the slot's play
        sim.data.qpos[q : q + 3] = (T_GRID @ [*(layout.cell(LAYOUT, column, heights[column]) + jitter), 1.0])[
            :3
        ]
        sim.data.qpos[q + 3 : q + 7] = quat
        heights[column] += 1
        used[player] += 1
        player = 3 - player
    mujoco.mj_forward(sim.model, sim.data)
    if used[2] < 21:  # the robot's next token, in its jaws
        assert person.give_token(60.0)[0]
        sim.data.ctrl[sim.model.actuator("gripper").id] = pct_to_rad(0.0)
    step(sim, 1.0, person)


def test_every_cell_of_random_game_states_is_read_and_the_grid_is_where_the_scene_has_it(game, request):
    sim, person, renderer = game
    light = request.node.callspec.params["game"]
    rng = np.random.default_rng(82)
    K = opencv_intrinsics(sim.get_camera_intrinsics())
    T_base_cam = load_extrinsics(EXTRINSICS).T_base_cam  # our calibration of the phase-1 camera (05 §5)
    wrong, states, pose_errors, reprojections = 0, [], [], []
    for k in range(STATES):
        moves = 42 if k == 0 else int(rng.integers(0, 43))
        set_state(sim, person, rng, moves)
        truth = tuple(person.board())
        assert sum(1 for cell in truth if cell) == moves, (k, moves)  # the state is the one asked
        renderer.update_scene(sim.data, camera=sim.camera_name)
        rgb = renderer.render()
        found = detect_markers(rgb, LAYOUT["markers"]["dictionary"])
        pose = board.grid_in_camera(found, LAYOUT, K)
        assert pose is not None, (k, sorted(found))
        assert len(pose[2]) == 4, (k, sorted(found))
        cells = board.read(rgb, LAYOUT, pose[0], K)
        wrong += sum(a != b for a, b in zip(cells, truth, strict=True))
        states.append(moves)
        T_base_grid = T_base_cam @ pose[0]
        true_position = sim.data.body("grid").xpos
        pose_errors.append(np.abs(np.linalg.inv(T_GRID)[:3, :3] @ (T_base_grid[:3, 3] - true_position)) * 1e3)
        reprojections.append(pose[1])
    along, across, up = np.max(pose_errors, axis=0)
    print(
        f"connect4 reading, {light} light: {42 * STATES - wrong}/{42 * STATES} cells on {STATES} states "
        f"({min(states)} to {max(states)} tokens); grid pose error max {along:.1f} mm along, {across:.1f} mm "
        f"across, {up:.1f} mm up; reprojection max {max(reprojections):.2f} px"
    )
    assert wrong == 0
    assert max(along, across, up) < 5.0
