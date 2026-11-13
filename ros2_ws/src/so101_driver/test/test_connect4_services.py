"""The driver in game mode (#80, phase2/P2-02 §7): with a connect4_layout it serves /sim/connect4/* instead of
/sim/randomize, frees a given token when the jaws close on it, and publishes the true board only with
expose_ground_truth (C-06). The real DriverNode, in-process, on a small scene (connect4_fake.py)."""

import numpy as np
import pytest
import rclpy
from connect4_fake import layout_path, scene
from std_srvs.srv import Trigger

from so101_driver.driver_node import DriverNode
from so101_interfaces.srv import DropToken, MoveGrid

SERVICES = {
    "/sim/connect4/drop_token",
    "/sim/connect4/give_token",
    "/sim/connect4/reset_board",
    "/sim/connect4/empty_dispenser",
    "/sim/connect4/fill_dispenser",
    "/sim/connect4/move_grid",
}
TRUTH = "/sim/ground_truth/connect4_board"


def start(*parameters):
    rclpy.init(args=["--ros-args", *(arg for p in parameters for arg in ("-p", p))])
    return DriverNode()


@pytest.fixture
def game(tmp_path, request):
    truth = getattr(request, "param", False)
    node = start(
        f"scene_path:={scene(tmp_path)}", f"connect4_layout:={layout_path()}", f"expose_ground_truth:={truth}"
    )
    yield node
    node.destroy_node()
    rclpy.shutdown()


def names(listing):
    return {name for name, _ in listing()}


def run(node, seconds, gripper=None):
    if gripper is not None:
        node._goal["gripper"] = gripper
    for _ in range(round(seconds * 50)):
        node._control_step()


def test_game_mode_serves_the_sim_services_of_the_game_and_not_randomize(game):
    services = names(game.get_service_names_and_types)
    assert services >= SERVICES
    assert "/sim/randomize" not in services


def test_without_a_layout_the_driver_serves_randomize_and_nothing_of_the_game():
    node = start()
    try:
        services = names(node.get_service_names_and_types)
        assert "/sim/randomize" in services
        assert not services & SERVICES
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_drop_token_answers_and_moves_a_token_of_the_color(game):
    reply = game._srv_drop_token(
        DropToken.Request(column=3, color=DropToken.Request.HUMAN), DropToken.Response()
    )
    assert reply.success, reply.message
    full = DropToken.Request(column=9, color=DropToken.Request.HUMAN)
    reply = game._srv_drop_token(full, DropToken.Response())
    assert not reply.success
    assert "column" in reply.message


def test_a_given_token_is_freed_once_the_jaws_close_on_it_and_reset_parks_it(game):
    run(game, 0.5, gripper=0.0)
    reply = game._srv_give_token(Trigger.Request(), Trigger.Response())
    assert not reply.success  # the jaws are closed
    assert "open" in reply.message
    run(game, 1.0, gripper=60.0)
    reply = game._srv_give_token(Trigger.Request(), Trigger.Response())
    assert reply.success, reply.message
    assert game._connect4.floating is not None
    run(game, 1.5, gripper=0.0)
    assert game._connect4.floating is None  # _control_step saw both jaws touch it
    assert game._srv_reset_board(Trigger.Request(), Trigger.Response()).success
    assert not any(game._connect4.board())


@pytest.mark.parametrize("game", [True], indirect=True)
def test_the_true_board_is_published_with_expose_ground_truth(game):
    assert TRUTH in names(game.get_topic_names_and_types)
    sent = []
    game._pub_board.publish = sent.append
    game._publish_ground_truth()
    (board,) = sent
    assert board.valid
    assert len(board.cells) == 42
    assert not any(board.cells)
    assert board.header.frame_id == "base_link"


def test_no_true_board_without_expose_ground_truth(game):
    assert not [topic for topic in names(game.get_topic_names_and_types) if "ground_truth" in topic]


def test_emptying_and_filling_the_dispenser_answer_what_the_simulated_person_says(game):
    """The driver's own small scene has no dispenser: the refusal passes through."""
    for serve in (game._srv_empty_dispenser, game._srv_fill_dispenser):
        reply = serve(Trigger.Request(), Trigger.Response())
        assert (reply.success, reply.message) == (False, "no dispenser in this scene")


def test_the_person_sets_the_grid_down_elsewhere_over_its_service(game):
    grid = game._robot.data.body("grid")
    before = grid.xpos.copy()
    reply = game._srv_move_grid(MoveGrid.Request(nearer=0.010, around=0.05), MoveGrid.Response())
    assert (reply.success, reply.message) == (True, "the grid 10 mm nearer, 2.9 deg around the arm")
    assert 0.005 < np.linalg.norm(grid.xpos - before) < 0.03
    assert game._srv_move_grid(MoveGrid.Request(), MoveGrid.Response()).success
    assert np.allclose(grid.xpos, before)
