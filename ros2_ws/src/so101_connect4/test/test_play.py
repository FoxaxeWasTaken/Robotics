"""play.py (#232), the console of `tools/so101 play`: the drawing and the keys, on a fake game. It holds no
rule: whose turn it is and the legal columns are read in /connect4/state. No ROS."""

import os
import pty
import select
import sys
import tty

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
        self.dropped = False  # as GameClient: the person's token went into the grid at the last play
        self.drops = None  # what `dropped` says after a play; None: what the reply says
        self.gone = False  # play raises, as a service that is gone

    def new_game(self, robot_first=False):
        self.calls.append(("new_game", robot_first))
        return self.reply

    def play(self, column=None):
        self.calls.append(("play", column))
        self.dropped = self.reply[0] if self.drops is None else self.drops
        if self.gone:
            raise RuntimeError("/connect4/human_done: no such service")
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
        (
            {"state": "IDLE", "ready": False, "not_ready": "no /connect4/grid_pose"},
            "waiting: no /connect4/grid_pose",
        ),
        (
            {"state": "IDLE", "ready": False, "not_ready": "column 7: unsafe_path:column"},
            "waiting: column 7: unsafe_path:column",
        ),
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


def test_q_quits_and_before_the_first_state_a_key_says_what_the_console_waits_for(console):
    console.game.state = None
    assert console.step()
    assert console.lines == []  # nothing to draw yet, and nothing said until a key comes
    assert console.step("4")
    assert console.lines == [play.NO_STATE]
    assert console.game.calls == []
    assert not console.step("q")


def test_a_turn_that_failed_can_be_played_again_with_the_same_key(console):
    """The state does not change when the hand or a service refuses: the key stayed dead, "not your turn"
    under a status that said "your turn"."""
    console.game.reply = (False, "no spare red token")
    console.step("4")
    console.step("4")
    assert console.game.calls == [("play", 3), ("play", 3)]
    assert console.lines[-1] == "no spare red token"
    console.game.reply = (True, "")
    console.step("4")
    console.step("4")  # played now: the state has not changed yet
    assert console.game.calls == [("play", 3)] * 3


def test_the_robots_turn_that_failed_is_tried_again_at_the_next_key_not_at_each_step(console):
    console.game.state = a_state(turn="robot")
    console.game.reply = (False, "/connect4/human_done: no answer in 10 s")
    console.step()
    console.step()
    assert console.game.calls == [("play", None)]  # no retry by itself: a service that is gone takes 10 s
    assert console.lines[-1] == "/connect4/human_done: no answer in 10 s; any key to try again"
    console.game.reply = (True, "")
    console.step("n")  # the game's own keys keep their meaning
    assert console.game.calls == [("play", None), ("new_game", False)]
    console.step(" ")
    assert console.game.calls == [("play", None), ("new_game", False), ("play", None)]
    console.step(" ")
    assert len(console.game.calls) == 3


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
    for key in ("1 to 7", "n ", "r ", "s ", "q ", "e, m, h", "?", "a "):
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


@pytest.mark.parametrize(
    ("source", "reason", "said"),
    [
        ("hand", "", "the robot's turn: a token is given to it"),
        ("hand", "no_token", "the robot's turn (no_token): a token is given to it"),
        ("dispenser", "dispenser_empty", "the robot's turn (dispenser_empty): the dispenser is filled"),
        ("dispenser", "no_token", "the robot's turn (no_token): it takes a token again"),
        ("dispenser", "", "the robot's turn: it takes a token"),
    ],
)
def test_the_robots_turn_says_where_its_token_comes_from(source, reason, said):
    state = a_state(turn="robot", reason=reason, token_source=source)
    assert play.status(state) == said


@pytest.fixture
def terminal(monkeypatch):
    """A terminal in cbreak mode as the console's standard input: returns what types on it."""
    master, slave = pty.openpty()
    tty.setcbreak(slave)
    stdin = os.fdopen(slave, "r")
    monkeypatch.setattr(sys, "stdin", stdin)

    def type_keys(keys):
        os.write(master, keys)
        assert select.select([stdin], [], [], 2.0)[0], "the keys did not arrive"

    yield type_keys
    stdin.close()
    os.close(master)


def test_keys_typed_ahead_are_read_one_at_a_time_and_none_stays_hidden(terminal):
    """Two keys within one pass of the loop: read through Python's buffered input, the second one stayed
    there, unseen by select, and came out at the next key press, as a move in another column."""
    assert play.read_key() is None
    terminal(b"45")
    assert [play.read_key(), play.read_key(), play.read_key()] == ["4", "5", None]
    terminal(b"2")
    assert [play.read_key(), play.read_key()] == ["2", None]


def test_a_special_key_plays_no_column(terminal):
    """Delete is ESC [ 3 ~, Page Up ESC [ 5 ~, F5 ESC [ 1 5 ~: read key by key, their digits were columns."""
    for sequence in (b"\x1b[3~", b"\x1b[5~", b"\x1b[15~", b"\x1b[A"):
        terminal(sequence)
        assert [play.read_key(), play.read_key()] == [None, None]
    terminal(b"3")
    assert play.read_key() == "3"


def test_a_byte_that_is_no_key_is_not_the_end_of_the_input(terminal):
    terminal("é".encode())  # two bytes, neither a key
    assert [play.read_key(), play.read_key(), play.read_key()] == [None, None, None]


def test_the_end_of_the_input_quits(monkeypatch):
    read, write = os.pipe()
    os.close(write)
    with os.fdopen(read, "r") as stdin:
        monkeypatch.setattr(sys, "stdin", stdin)
        assert play.read_key() == "q"


# Comfort for the demo (#265): robot against robot, the screen redrawn in place, keys in either case.
def test_the_automatic_player_plays_the_persons_turns_with_the_engine_until_a_key_stops_it(console):
    """A whole game to watch: `a` plays the person's turns, the engine at the hint's depth."""
    console.step("a")
    assert console.lines[-1] == "automatic player: on, any key stops it"
    assert console.game.calls == [("play", 3)]  # the engine's move on an empty board: the middle column
    console.step()
    assert console.game.calls == [("play", 3)]  # once per state
    cells = with_tokens(EMPTY_BOARD, (3, HUMAN), (3, ROBOT))
    console.game.state = a_state(cells=cells, moves=2)
    console.step()
    assert len(console.game.calls) == 2
    assert console.game.calls[-1][0] == "play"
    console.game.state = a_state(cells=with_tokens(cells, (2, HUMAN), (2, ROBOT)), moves=4)
    console.step("x")  # any key stops it, and is not taken for a move
    assert console.lines[-1] == "automatic player: off"
    assert len(console.game.calls) == 2
    console.step()
    assert len(console.game.calls) == 2


def test_the_automatic_player_never_plays_out_of_the_persons_turn(console):
    console.game.state = a_state(state="MOVE", turn="robot")
    console.step("a")
    console.step()
    assert console.game.calls == []
    console.game.state = a_state(state="GAME_OVER", result="robot")
    console.step()
    assert console.game.calls == []
    console.game.state = a_state(turn="robot")  # the robot's own turn stays the console's usual answer
    console.step()
    assert console.game.calls == [("play", None)]


def test_a_key_in_capitals_is_the_same_key(console):
    console.step("N")
    assert console.game.calls == [("new_game", False)]
    console.step("H")
    assert console.game.calls[-1] == ("set_depth", play.LEVEL_KEYS["h"])


def test_the_board_goes_to_the_drawing_and_the_messages_to_the_writing():
    """On a terminal the board is redrawn in place and the last message stays under it: the console gives
    them apart."""
    game, boards, messages = FakeGame(a_state()), [], []
    console = play.Console(game, messages.append, draw=boards.append)
    console.step("x")
    assert len(boards) == 1
    assert boards[0].splitlines()[0] == "  1 2 3 4 5 6 7"
    assert messages == [play.KEYS]


def test_a_screen_keeps_the_keys_the_last_board_and_the_last_message_and_redraws_them_in_place():
    out = []
    screen = play.Screen(out.append)
    screen.draw("board 1")
    screen.write("column 3 is full")
    screen.draw("board 2")
    assert out[-1] == "\x1b[H\x1b[J" + play.KEYS + "\nboard 2\ncolumn 3 is full"
    assert len(out) == 3  # one repaint per change, the whole screen each time


def test_a_token_in_the_grid_is_not_dropped_again_when_the_end_of_the_turn_fails(console):
    """The hand dropped the token, then human_done was refused, or its service was gone: the key played the
    turn again, a second token in the same column, and the game ended in FIX_BOARD."""
    console.game.reply, console.game.drops = (False, "not the person's turn"), True
    console.step("4")
    console.step("4")
    assert console.game.calls == [("play", 3)]
    assert "not the person's turn" in console.lines
    console.game.state = a_state()  # a new state, and the service gone this time
    console.game.calls.clear()
    console.game.gone = True
    console.step("4")
    console.step("4")
    assert console.game.calls == [("play", 3)]


def test_the_automatic_player_stops_when_a_service_is_gone(console):
    """It stayed on and dropped one more token at each pass, about every 10 s, with no key pressed."""
    console.game.gone, console.game.drops = True, False
    console.step("a")
    console.step()
    console.step()
    assert console.game.calls == [("play", 3)]
    assert console.auto is False


def test_a_key_of_the_game_typed_as_the_robots_turn_is_asked_is_not_lost(console):
    """`s` in the pass that asks for the robot's turn: the turn is asked, then the stop."""
    console.game.state = a_state(turn="robot")
    console.step("s")
    assert console.game.calls == [("play", None), ("stop",)]
