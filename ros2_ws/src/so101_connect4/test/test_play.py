"""play.py (#232), the console of `tools/so101 play`: the drawing and the keys, on a fake game. It holds no
rule: whose turn it is and the legal columns are read in /connect4/state. No ROS."""

import pytest
from test_turns import EMPTY_BOARD, with_tokens

from so101_connect4 import play
from so101_connect4.game import HUMAN, ROBOT


def a_state(state="WAIT_HUMAN", turn="human", cells=EMPTY_BOARD, **fields):
    """A /connect4/state, as the game node publishes it."""
    legal = [c for c in range(7) if cells[35 + c] == 0]
    base = {"ready": True, "max_depth": 10, "state": state, "reason": "", "turn": turn, "board": list(cells)}
    return base | {"legal": legal, "last": None, "result": "", "moves": 0, "lost": 0, "missed": 0} | fields


class FakeGame:
    """client.GameClient as the console uses it; `calls` in order."""

    def __init__(self, state):
        self.state, self.calls, self.reply = state, [], (True, "")

    def new_game(self, robot_first=False):
        self.calls.append(("new_game", robot_first))
        return self.reply

    def play(self, column=None):
        self.calls.append(("play", column))
        return self.reply

    def stop(self):
        self.calls.append(("stop",))
        return self.reply

    def set_depth(self, depth):
        self.calls.append(("set_depth", depth))
        return self.reply


@pytest.fixture
def console():
    game, lines = FakeGame(a_state()), []
    console = play.Console(game, lines.append)
    console.game, console.lines = game, lines
    return console


def test_the_board_is_drawn_top_row_first_with_the_keys_of_its_columns():
    cells = with_tokens(EMPTY_BOARD, (0, HUMAN), (0, ROBOT), (6, HUMAN))
    text = play.render(a_state(cells=cells, moves=3))
    assert text.splitlines()[:7] == [
        "  1 2 3 4 5 6 7",
        "| . . . . . . . |",
        "| . . . . . . . |",
        "| . . . . . . . |",
        "| . . . . . . . |",
        "| Y . . . . . . |",
        "| R . . . . . R |",
    ]


@pytest.mark.parametrize(
    ("fields", "said"),
    [
        ({"state": "WAIT_HUMAN", "turn": "human"}, "your turn (R): 1 to 7"),
        ({"state": "WAIT_HUMAN", "turn": "robot", "reason": "no_token"}, "no_token"),
        ({"state": "MOVE"}, "the robot plays"),
        ({"state": "THINK"}, "the robot thinks"),
        ({"state": "GAME_OVER", "result": "robot"}, "the robot wins"),
        ({"state": "GAME_OVER", "result": "human"}, "you win"),
        ({"state": "GAME_OVER", "result": "draw"}, "a draw"),
        ({"state": "FIX_BOARD", "reason": "not_seen:human"}, "FIX_BOARD: not_seen:human"),
        ({"state": "ERROR", "reason": "stopped"}, "ERROR: stopped"),
        ({"state": "IDLE", "ready": False}, "waiting for the stack"),
        ({"state": "IDLE"}, "n: new game"),
    ],
)
def test_the_status_line_says_what_the_person_can_do(fields, said):
    assert said in play.render(a_state(**fields)).splitlines()[-1]


def test_the_last_move_is_shown_with_its_column_as_the_keys_number_it():
    last = {"who": "robot", "column": 2, "planned": 2, "score": 3, "depth": 10}
    assert "robot: column 3" in play.render(a_state(last=last))


def test_the_colors_only_wrap_the_tokens():
    cells = with_tokens(EMPTY_BOARD, (0, HUMAN), (1, ROBOT))
    plain, colored = play.render(a_state(cells=cells)), play.render(a_state(cells=cells), color=True)
    assert "\x1b[" not in plain
    assert "\x1b[31mR\x1b[0m \x1b[33mY\x1b[0m" in colored


def test_the_state_is_drawn_once_then_again_when_it_changes(console):
    console.step()
    console.step()
    assert len(console.lines) == 1
    console.game.state = a_state(state="THINK", turn="robot")
    console.step()
    assert len(console.lines) == 2


def test_a_column_key_plays_it_on_the_persons_turn_once(console):
    assert console.step("4")
    assert console.game.calls == [("play", 3)]
    console.step("4")  # the state has not changed yet: the turn is being played
    assert console.game.calls == [("play", 3)]


def test_a_full_column_changes_nothing_and_says_so(console):
    console.game.state = a_state(cells=with_tokens(EMPTY_BOARD, *[(2, HUMAN), (2, ROBOT)] * 3))
    console.step("3")
    assert console.game.calls == []
    assert "full" in console.lines[-1]


@pytest.mark.parametrize("state", ["MOVE", "THINK", "READ_BOARD", "GAME_OVER", "IDLE"])
def test_a_column_key_outside_the_persons_turn_changes_nothing_and_says_why(console, state):
    """A token dropped while the robot moves would end the game in FIX_BOARD."""
    console.game.state = a_state(state=state, result="draw" if state == "GAME_OVER" else "")
    console.step("4")
    assert console.game.calls == []
    assert state in console.lines[-1]


def test_a_key_that_means_nothing_changes_nothing_and_shows_the_keys(console):
    console.step("x")
    assert console.game.calls == []
    assert console.lines[-1] == play.KEYS


def test_the_robots_turn_gives_its_token_without_a_key_once(console):
    """The robot starts, or its token was lost: nobody has a key for that."""
    console.game.state = a_state(turn="robot")
    console.step()
    console.step()
    assert console.game.calls == [("play", None)]
    console.step("4")
    assert console.game.calls == [("play", None)]


@pytest.mark.parametrize(
    ("key", "call"), [("n", ("new_game", False)), ("r", ("new_game", True)), ("s", ("stop",))]
)
def test_the_game_keys_call_the_game_and_show_a_refusal(console, key, call):
    console.game.reply = (False, "refused in MOVE")
    assert console.step(key)
    assert console.game.calls == [call]
    assert console.lines[-1] == "refused in MOVE"


def test_q_quits_and_nothing_is_drawn_before_the_first_state(console):
    console.game.state = None
    assert console.step("4")
    assert console.lines == []
    assert console.game.calls == []
    assert not console.step("q")


def test_a_service_that_does_not_answer_is_said_and_the_console_goes_on(console):
    def gone(column=None):
        raise RuntimeError("/connect4/human_done: no such service")

    console.game.play = gone
    assert console.step("4")
    assert console.lines[-1] == "/connect4/human_done: no such service"


def test_each_drawing_shows_at_once_through_a_pipe(monkeypatch):
    """Through a pipe stdout is block-buffered: unflushed, nothing showed for minutes."""

    class Pipe:
        def __init__(self):
            self.waiting, self.shown = "", ""

        def write(self, text):
            self.waiting += text

        def flush(self):
            self.shown, self.waiting = self.shown + self.waiting, ""

    pipe = Pipe()
    monkeypatch.setattr("sys.stdout", pipe)
    play.Console(FakeGame(a_state())).step()
    assert "your turn" in pipe.shown


@pytest.mark.parametrize(("depth", "level"), [(1, "easy"), (4, "medium"), (10, "hard"), (7, "depth 7")])
def test_the_level_and_the_robots_reason_are_shown_above_the_status(depth, level):
    last = {"who": "robot", "column": 2, "planned": 2, "score": 990, "depth": 6, "wins_in": 3}
    lines = play.render(a_state(max_depth=depth, last=last)).splitlines()
    assert lines[-2] == f"level: {level} | robot: column 3 (depth 6, it wins in 1 move)"
    assert play.render(a_state(max_depth=depth)).splitlines()[-2] == f"level: {level}"


@pytest.mark.parametrize(("key", "depth"), [("e", 1), ("m", 4), ("h", 10)])
def test_a_level_key_sets_the_engines_depth_and_shows_a_refusal(console, key, depth):
    console.game.reply = (False, "/game_node/set_parameters: no such service")
    assert console.step(key)
    assert console.game.calls == [("set_depth", depth)]
    assert console.lines[-1] == "/game_node/set_parameters: no such service"


def test_the_hint_is_the_engines_move_for_the_person_found_in_the_console():
    """Three of the person's in the bottom row, the fourth cell free: the hint is that column, not column 7
    where the robot would win."""
    cells = with_tokens(EMPTY_BOARD, (0, HUMAN), (6, ROBOT), (1, HUMAN), (6, ROBOT), (2, HUMAN), (6, ROBOT))
    game, lines = FakeGame(a_state(cells=cells, moves=6)), []
    console = play.Console(game, lines.append)
    assert console.step("?")
    assert lines[-1] == "hint: column 4"
    assert game.calls == []  # the game node is not asked: its engine thinks for the robot only


def test_a_hint_outside_the_persons_turn_is_refused(console):
    console.game.state = a_state(state="MOVE", turn="robot")
    console.step("?")
    assert console.lines[-1] == "not your turn: MOVE"


def test_the_winning_four_is_marked_the_other_tokens_stay():
    cells = with_tokens(
        EMPTY_BOARD, (0, HUMAN), (0, ROBOT), (1, HUMAN), (1, ROBOT), (2, HUMAN), (2, ROBOT), (3, HUMAN)
    )
    over = a_state(state="GAME_OVER", result="human", cells=cells, moves=7)
    assert play.render(over).splitlines()[5:7] == ["| Y Y Y . . . . |", "| * * * * . . . |"]
    assert "\x1b[31m*\x1b[0m \x1b[31m*\x1b[0m" in play.render(over, color=True)


def test_the_keys_line_names_every_key():
    for key in ("1 to 7", "n ", "r ", "s ", "q ", "e, m, h", "?"):
        assert key in play.KEYS


RED_ROBOT = {HUMAN: "yellow", ROBOT: "red"}  # layout.colors of a file whose token.robot_color is red


def test_with_the_robot_on_red_its_cells_are_r_and_the_person_plays_y():
    """#255: the console shows each side in the color of its tokens (token.robot_color)."""
    cells = with_tokens(EMPTY_BOARD, (0, HUMAN), (1, ROBOT))
    lines = play.render(a_state(cells=cells), colors=RED_ROBOT).splitlines()
    assert lines[6] == "| Y R . . . . . |"
    assert lines[-1] == "your turn (Y): 1 to 7"
    assert "\x1b[33mY\x1b[0m \x1b[31mR\x1b[0m" in play.render(
        a_state(cells=cells), color=True, colors=RED_ROBOT
    )


def test_the_console_draws_with_the_color_it_was_given():
    game, lines = FakeGame(a_state(cells=with_tokens(EMPTY_BOARD, (1, ROBOT)))), []
    play.Console(game, lines.append, colors=RED_ROBOT).step()
    assert "| . R . . . . . |" in lines[0]
