"""The simulated person of the Connect Four game (connect4_sim.py, #80, phase2/P2-02 §7): a token dropped
above a column, one put between the open jaws and held by nothing until they close, the board cleared, the
true board read. On a small scene (connect4_fake.py); what falls and stacks is tested on the real one, in
so101_bringup/test. No ROS."""

import mujoco
import numpy as np
import pytest
from connect4_fake import LAYOUT, PER_COLOR, parking, scene
from so101_sim import ARM_JOINTS, SO101Sim

from so101_common.robot import pct_to_rad
from so101_connect4 import layout
from so101_driver.connect4_sim import HUMAN, MIN_OPEN_PCT, ROBOT, Connect4Sim

GRID, TOKEN = LAYOUT["grid"], LAYOUT["token"]
T_GRID = layout.grid_pose(LAYOUT)
HANDOVER = [-0.8, -0.6, 0.6, 1.1, 1.57]  # the candidate of P2-02 §7: the jaws' axis horizontal


@pytest.fixture
def sim(tmp_path):
    sim = SO101Sim(scene_path=scene(tmp_path))
    sim.connect()
    yield sim
    sim.disconnect()


@pytest.fixture
def person(sim):
    return Connect4Sim(sim, LAYOUT, np.random.default_rng(0))


def in_grid(sim, name):
    return (np.linalg.inv(T_GRID) @ [*sim.data.body(name).xpos, 1.0])[:3]


def put(sim, name, column, row):
    """A token at rest in a cell, as if it had fallen there."""
    q = sim.model.joint(f"{name}_free").qposadr[0]
    sim.data.qpos[q : q + 3] = (T_GRID @ [*layout.cell(LAYOUT, column, row), 1.0])[:3]
    mujoco.mj_forward(sim.model, sim.data)


def parked(sim, name):
    color, i = name.split("_")[1:]
    return np.allclose(sim.data.body(name).xpos, parking(color, int(i)), atol=5e-3)


def set_arm(sim, q, gripper_pct):
    for name, value in zip([*ARM_JOINTS[:5], "gripper"], [*q, pct_to_rad(gripper_pct)], strict=True):
        sim.data.qpos[sim.model.joint(name).qposadr[0]] = value
        sim.data.ctrl[sim.model.actuator(name).id] = value
    mujoco.mj_forward(sim.model, sim.data)


def tick(sim, person, gripper_pct=None):
    """One control tick of the driver: 10 steps, then the check a floating token needs."""
    if gripper_pct is not None:
        sim.data.ctrl[sim.model.actuator("gripper").id] = pct_to_rad(gripper_pct)
    for _ in range(10):
        mujoco.mj_step(sim.model, sim.data)
    person.release_if_pinched()


def in_gripper(sim, name):
    """A token's centre from the TCP, in the gripper body's axes: x the jaws' axis, y, z."""
    R = sim.data.body("gripper").xmat.reshape(3, 3)
    return R.T @ (sim.data.body(name).xpos - sim.data.site("gripperframe").xpos)


def test_a_dropped_token_waits_above_its_column_its_disc_in_the_slot_plane(sim, person):
    ok, message = person.drop_token(3, HUMAN)
    assert ok, message
    x, y, z = in_grid(sim, "token_red_0")
    assert np.isclose(x, 3 * GRID["column_pitch"], atol=1e-6)
    assert np.isclose(y, 0.0, atol=1e-6)
    entry = (GRID["rows"] - 0.5) * GRID["row_pitch"] + GRID["headroom"]
    assert entry < z - TOKEN["diameter"] / 2 < entry + GRID["depth_guide_height"]  # inside the guide
    axis = sim.data.body("token_red_0").xmat.reshape(3, 3)[:, 2]
    assert np.isclose(abs(axis @ T_GRID[:3, 1]), 1.0, atol=1e-6)  # the disc's axis along the grid's normal
    assert not np.any(sim.data.qvel[sim.model.joint("token_red_0_free").dofadr[0] :][:6])


def test_each_drop_takes_the_next_spare_token_of_its_color(sim, person):
    assert person.drop_token(0, HUMAN)[0]
    assert person.drop_token(1, ROBOT)[0]
    assert person.drop_token(2, HUMAN)[0]
    moved = {name for name in ("token_red_0", "token_red_1", "token_yellow_0") if not parked(sim, name)}
    assert moved == {"token_red_0", "token_red_1", "token_yellow_0"}
    assert parked(sim, "token_red_2")
    assert parked(sim, "token_yellow_1")


def test_a_full_column_an_unknown_column_and_an_unknown_color_are_refused(sim, person):
    for row in range(GRID["rows"]):
        put(sim, f"token_red_{row}", 2, row)
    for column, color, why in ((2, HUMAN, "full"), (7, HUMAN, "column"), (0, 3, "color")):
        ok, message = person.drop_token(column, color)
        assert not ok, (column, color, message)
        assert why in message, (column, color, message)
    assert parked(sim, "token_red_6")  # nothing moved
    assert parked(sim, "token_yellow_0")


def test_a_drop_with_no_spare_token_left_is_refused(sim, person):
    for i in range(PER_COLOR):
        put(sim, f"token_red_{i}", i % 7, i // 7)
    ok, message = person.drop_token(6, HUMAN)
    assert not ok
    assert "spare" in message


def test_the_board_is_row_major_from_the_bottom_red_for_the_human_yellow_for_the_robot(sim, person):
    put(sim, "token_red_0", 0, 0)
    put(sim, "token_yellow_0", 0, 1)
    put(sim, "token_red_1", 6, 5)
    cells = person.board()
    assert len(cells) == 42
    assert (cells[0], cells[7], cells[5 * 7 + 6]) == (HUMAN, ROBOT, HUMAN)
    assert sum(1 for cell in cells if cell) == 3  # the parked tokens are not on the board


def test_a_token_still_falling_is_not_on_the_board(sim, person):
    put(sim, "token_red_0", 4, 3)
    sim.data.qvel[sim.model.joint("token_red_0_free").dofadr[0] + 2] = -0.5
    mujoco.mj_forward(sim.model, sim.data)
    assert not any(person.board())


def test_a_given_token_floats_between_the_open_jaws_on_the_fixed_one(sim, person):
    set_arm(sim, HANDOVER, 50.0)
    ok, message = person.give_token(50.0)
    assert ok, message
    x, y, z = in_gripper(sim, "token_yellow_0")
    assert np.isclose(x, TOKEN["thickness"] / 2 + 0.0005, atol=1e-6)  # half a thickness off the fixed jaw
    assert abs(y) <= 0.003  # the hand's seeded offset
    assert abs(z) <= 0.005
    assert (y, z) != (0.0, 0.0)
    axis = sim.data.body("token_yellow_0").xmat.reshape(3, 3)[:, 2]
    jaws = sim.data.body("gripper").xmat.reshape(3, 3)[:, 0]
    assert np.isclose(abs(axis @ jaws), 1.0, atol=1e-6)
    height = sim.data.body("token_yellow_0").xpos[2]
    for _ in range(25):  # 0.5 s: a free token falls 30 mm in 0.1 s
        tick(sim, person)
    assert abs(sim.data.body("token_yellow_0").xpos[2] - height) < 1e-3
    assert person.floating == "token_yellow_0"


def test_give_token_is_refused_while_the_jaws_are_not_open(sim, person):
    set_arm(sim, HANDOVER, 5.0)
    ok, message = person.give_token(MIN_OPEN_PCT - 1.0)
    assert not ok
    assert "open" in message
    assert parked(sim, "token_yellow_0")
    assert person.floating is None


def test_the_closing_jaws_take_the_token_and_it_holds_under_gravity(sim, person):
    set_arm(sim, HANDOVER, 50.0)
    assert person.give_token(50.0)[0]
    for _ in range(50):  # 1 s: the jaws need 0.46 s
        tick(sim, person, gripper_pct=0.0)
    assert person.floating is None
    dof = sim.model.joint("token_yellow_0_free").dofadr[0]
    assert not np.any(sim.data.qfrc_applied[dof : dof + 6])  # gravity is back
    held = in_gripper(sim, "token_yellow_0")
    for _ in range(50):
        tick(sim, person)
    assert np.linalg.norm(in_gripper(sim, "token_yellow_0") - held) < 0.002  # it stays in the jaws


def test_a_second_give_moves_the_token_still_floating_and_takes_no_other(sim, person):
    set_arm(sim, HANDOVER, 50.0)
    assert person.give_token(50.0)[0]
    assert person.give_token(50.0)[0]
    assert person.floating == "token_yellow_0"
    assert parked(sim, "token_yellow_1")


def test_reset_board_parks_every_token_and_drops_the_one_given(sim, person):
    set_arm(sim, HANDOVER, 50.0)
    put(sim, "token_red_0", 1, 0)
    assert person.drop_token(4, ROBOT)[0]
    assert person.give_token(50.0)[0]
    person.reset_board()
    assert all(parked(sim, f"token_{color}_{i}") for color in ("red", "yellow") for i in range(PER_COLOR))
    assert person.floating is None
    assert not np.any(sim.data.qfrc_applied)
    assert not any(person.board())


@pytest.fixture
def red_robot(sim):
    """#255: the person's hand when the layout gives the robot the red tokens."""
    layout_red = {**LAYOUT, "token": {**TOKEN, "robot_color": "red"}}
    return Connect4Sim(sim, layout_red, np.random.default_rng(0))


def test_with_the_robot_on_red_the_hand_gives_a_red_token_and_drops_a_yellow_one(sim, red_robot):
    set_arm(sim, HANDOVER, 50.0)
    assert red_robot.give_token(50.0)[0]
    assert red_robot.floating == "token_red_0"
    assert red_robot.drop_token(3, HUMAN)[0]
    assert np.isclose(in_grid(sim, "token_yellow_0")[0], 3 * GRID["column_pitch"], atol=1e-6)
    assert parked(sim, "token_red_1")


def test_with_the_robot_on_red_the_true_board_counts_red_as_the_robots(sim, red_robot):
    put(sim, "token_red_0", 0, 0)
    put(sim, "token_yellow_0", 0, 1)
    cells = red_robot.board()
    assert cells[0] == ROBOT
    assert cells[7] == HUMAN


def test_on_a_scene_without_a_dispenser_there_is_none_to_empty_or_to_fill(sim, person):
    """The dispenser of the game's scene is so101_bringup's test_connect4_dispenser.py (#259)."""
    assert person.empty_dispenser() == (False, "no dispenser in this scene")
    assert person.fill_dispenser() == (False, "no dispenser in this scene")


def test_the_person_sets_the_empty_grid_down_nearer_the_arm_and_carried_around_it(sim, person):
    """#260: where the grid stands is the camera's to find, so the scene can move it between two games."""
    grid, pan = sim.data.body("grid"), np.array(LAYOUT["placement"]["pan_axis"])
    reply = person.move_grid(0.010, np.radians(3.0))
    assert reply == (True, "the grid 10 mm nearer, 3.0 deg around the arm")
    R = grid.xmat.reshape(3, 3)
    assert np.isclose(np.arctan2(R[1, 0], R[0, 0]), LAYOUT["placement"]["azimuth"] + np.radians(3.0))
    assert np.allclose(R[:, 2], [0.0, 0.0, 1.0])
    from_axis = grid.xpos[:2] - pan
    # the pan axis is the model's joint, 0.0388 m to a few hundredths of a millimetre [MES-06]
    assert np.isclose(from_axis @ R[:2, 0], LAYOUT["placement"]["first_column_r"] - 0.010, atol=1e-4)
    assert np.isclose(from_axis @ R[:2, 1], -LAYOUT["placement"]["plane_offset"], atol=1e-4)  # as edge-on
    assert np.isclose(grid.xpos[2], T_GRID[2, 3])
    # the person drops a token where the grid now stands, and the true board counts it there
    assert person.drop_token(4, HUMAN)[0]
    x, y, _ = R.T @ (sim.data.body("token_red_0").xpos - grid.xpos)
    assert np.allclose([x, y], [4 * GRID["column_pitch"], 0.0], atol=1e-6)
    put_at = grid.xpos + R @ layout.cell(LAYOUT, 4, 0)
    q = sim.model.joint("token_red_0_free").qposadr[0]
    sim.data.qpos[q : q + 3] = put_at
    mujoco.mj_forward(sim.model, sim.data)
    assert person.board()[4] == HUMAN
    # back where the scene starts it
    person.reset_board()
    assert person.move_grid(0.0, 0.0)[0]
    assert np.allclose(grid.xpos, T_GRID[:3, 3])
    assert np.allclose(grid.xmat.reshape(3, 3), T_GRID[:3, :3], atol=1e-9)


def test_a_grid_with_a_token_in_it_stays_where_it_is(sim, person):
    put(sim, "token_red_0", 1, 0)
    assert person.move_grid(0.010, 0.0) == (False, "the grid is not empty")
    assert np.allclose(sim.data.body("grid").xpos, T_GRID[:3, 3])
