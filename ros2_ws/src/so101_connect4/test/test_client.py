"""client.py (#232): the console's and the bench's side of a game, against the real game node and the fake
stack of test_game_node.py. The hand is a fake: the client names no sim service."""

import pytest
from std_srvs.srv import Trigger
from test_game_node import stack  # noqa: F401  (the fixture)
from test_turns import EMPTY_BOARD, with_tokens

from so101_connect4 import client
from so101_connect4.game import HUMAN


class FakeHand:
    """The person's hand on the fake stack's grid; `calls` in order."""

    def __init__(self, stack):  # noqa: F811
        self.stack, self.calls, self.refuse = stack, [], None

    def _done(self, call):
        self.calls.append(call)
        return (False, "no spare token") if call[0] == self.refuse else (True, "")

    def drop_token(self, column):
        self.stack.cells = with_tokens(self.stack.cells, (column, HUMAN))
        return self._done(("drop", column))

    def give_token(self):
        return self._done(("give",))

    def reset_board(self):
        self.stack.cells = EMPTY_BOARD
        return self._done(("reset",))


@pytest.fixture
def game(stack):  # noqa: F811
    stack.up()
    hand = FakeHand(stack)
    return client.GameClient(stack, hand, spin_once=lambda: stack.executor_.spin_once(timeout_sec=0.01))


def test_a_new_game_is_asked_then_the_grid_cleared(game, stack):  # noqa: F811
    """The game node takes a board not empty as "not yet": the reset may come after the new game."""
    stack.cells = with_tokens(EMPTY_BOARD, (3, HUMAN))  # the last game's
    assert game.new_game() == (True, "")
    assert game.hand.calls == [("reset",)]
    assert game.wait(lambda state: state["state"] == "WAIT_HUMAN", 10.0)
    assert game.state["turn"] == "human"


def test_a_refused_new_game_leaves_the_grid_as_it_is(game, stack):  # noqa: F811
    stack.hold = True
    assert game.new_game()[0]
    assert game.wait(lambda state: state["state"] == "MOVE", 5.0)
    assert game.new_game() == (False, "refused in MOVE")
    assert game.hand.calls == [("reset",)]


def test_a_turn_gives_the_robot_its_token_drops_the_persons_and_says_done(game, stack):  # noqa: F811
    game.new_game()
    game.wait(lambda state: state["state"] == "WAIT_HUMAN", 10.0)
    assert game.play(3) == (True, "")
    assert game.hand.calls[1:] == [("give",), ("drop", 3)]
    assert game.wait(lambda state: state["moves"] == 2 and state["state"] == "WAIT_HUMAN", 10.0)


def test_the_robots_turn_only_gives_its_token(game, stack):  # noqa: F811
    game.new_game(robot_first=True)
    game.wait(lambda state: state["state"] == "WAIT_HUMAN", 10.0)
    assert game.play() == (True, "")
    assert game.hand.calls[1:] == [("give",)]
    assert game.wait(lambda state: state["moves"] == 1, 10.0)


@pytest.mark.parametrize(("refused", "calls"), [("give", [("give",)]), ("drop", [("give",), ("drop", 2)])])
def test_a_hand_that_fails_stops_the_turn_with_its_message(game, stack, refused, calls):  # noqa: F811
    game.new_game()
    game.wait(lambda state: state["state"] == "WAIT_HUMAN", 10.0)
    game.hand.refuse = refused
    assert game.play(2) == (False, "no spare token")
    assert game.hand.calls[1:] == calls
    stack.spin_for(0.3)
    assert game.state["state"] == "WAIT_HUMAN"  # human_done was not said


def test_a_turn_out_of_turn_gives_the_nodes_refusal_and_stop_ends_the_game(game, stack):  # noqa: F811
    assert game.wait(lambda state: True, 5.0)
    assert game.play() == (False, "refused in IDLE")
    assert game.stop() == (True, "")
    assert game.wait(lambda state: state["reason"] == "stopped", 5.0)


def test_a_service_that_is_not_there_raises(game, stack):  # noqa: F811
    nowhere = stack.create_client(Trigger, "/nowhere")
    with pytest.raises(RuntimeError, match="/nowhere"):
        client.call(lambda: stack.executor_.spin_once(timeout_sec=0.01), nowhere, timeout_s=0.3)


ROBOT_MOVE = {"who": "robot", "column": 3, "planned": 3, "score": 2, "depth": 10}


@pytest.mark.parametrize(
    ("last", "said"),
    [
        (None, ""),
        ({"who": "human", "column": 0, "planned": 0}, "human: column 1"),
        (ROBOT_MOVE, "robot: column 4 (depth 10)"),
        (ROBOT_MOVE | {"wins_in": 1}, "robot: column 4 (depth 10, it wins)"),
        (ROBOT_MOVE | {"wins_in": 3}, "robot: column 4 (depth 10, it wins in 1 move)"),
        (ROBOT_MOVE | {"wins_in": 7}, "robot: column 4 (depth 10, it wins in 3 moves)"),
        (ROBOT_MOVE | {"loses_in": 2}, "robot: column 4 (depth 10, it loses in 1 move)"),
        (ROBOT_MOVE | {"loses_in": 6}, "robot: column 4 (depth 10, it loses in 3 moves)"),
        # its token fell in another column: the engine's proof was for the one it meant
        (ROBOT_MOVE | {"planned": 4, "wins_in": 3}, "robot: column 4 (it meant column 5)"),
    ],
)
def test_the_last_move_is_said_with_the_robots_reason(last, said):
    """wins_in and loses_in are plies, the robot's own included (turns.py): said as the moves the winner
    still has to play once this one is on the grid."""
    assert client.last_move(last) == said


def test_the_level_is_the_game_nodes_depth_set_from_outside(game, stack):  # noqa: F811
    assert game.wait(lambda state: state["max_depth"] == 10, 5.0)
    assert game.set_depth(4) == (True, "")
    assert game.wait(lambda state: state["max_depth"] == 4, 5.0)
    refused = game.set_depth(0)
    assert not refused[0] and "engine.max_depth" in refused[1]
    stack.spin_for(0.3)
    assert game.state["max_depth"] == 4
