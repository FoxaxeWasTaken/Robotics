"""The turns of a game (turns.py, phase2/P2-02 §5): each transition and each row of the failure table on a
fake clock, then random games against a fake world (ported from the prototype's test_turns.py): the arm moves
in MOVE only (target P2-05), a game always ends, no illegal drop. No ROS, no sim."""

import random
from collections import deque

import pytest

from so101_connect4 import game
from so101_connect4.game import CELLS, COLUMNS, EMPTY, HUMAN, ROBOT
from so101_connect4.turns import NO_TOKEN, Board, Inputs, State, Stations, Turns

EMPTY_BOARD = (EMPTY,) * CELLS


def with_tokens(cells, *tokens):
    """cells with (column, player) dropped in order."""
    cells = list(cells)
    for column, player in tokens:
        row = next(r for r in range(game.ROWS) if cells[r * COLUMNS + column] == EMPTY)
        cells[row * COLUMNS + column] = player
    return tuple(cells)


class Clock:
    """Turns stepped at 10 Hz on a fake clock; `board` is the last /connect4/board, as the node keeps it."""

    def __init__(self, dispenser=False):
        self.turns, self.now, self.board, self.out = Turns(dispenser=dispenser), 0.0, None, None
        self.stations = None  # the last /connect4/stations

    def step(self, dt=0.1, **inputs):
        self.now += dt
        self.out = self.turns.step(self.now, Inputs(board=self.board, stations=self.stations, **inputs))
        return self.out

    def sees(self, *waiting):
        """A stable read of the dispenser's stations arrives, then a step sees it."""
        self.now += 0.05
        self.stations = Stations(tuple(bool(there) for there in waiting), self.now)
        return self.step()

    def read(self, cells, valid=True, reason=""):
        """A stable read arrives, then a step sees it."""
        self.now += 0.05
        self.board = Board(tuple(cells), valid, reason, self.now)
        return self.step()

    def wait(self, seconds):
        for _ in range(round(seconds / 0.1)):
            self.step()

    @property
    def state(self):
        return self.turns.state

    @property
    def label(self):
        return self.turns.label


def choice(column, score=0, depth=1):
    """The engine's answer, as game.best_move gives it."""
    return game.Choice(column, score, depth, nodes=1, seconds=0.0)


def new_game(robot_first=False, dispenser=False):
    """A clock at WAIT_HUMAN of a new game: HANDOVER reached, the empty board read."""
    c = Clock(dispenser)
    assert c.step(new_game=True, robot_first=robot_first).handover
    assert c.state is State.MOVE
    c.step(move_reply=(True, "at HANDOVER"))
    assert c.state is State.READ_BOARD
    c.read(EMPTY_BOARD)
    assert c.state is State.WAIT_HUMAN
    assert c.turns.turn == ("robot" if robot_first else "human")
    return c


def human_plays(c, column):
    """The human's token lands, human_done, the reader sees it: THINK."""
    cells = with_tokens(c.turns.cells, (column, HUMAN))
    c.step(human_done=True)
    assert c.state is State.READ_BOARD
    c.read(cells)
    return cells


def robot_plays(c, column, lands=None):
    """From THINK: the engine's column, the drop, its reply, the reader sees the token in `lands`."""
    out = c.step(engine=choice(column))
    assert c.state is State.MOVE
    assert out.drop == column
    assert not out.handover
    c.step(move_reply=(True, "dropped"))
    assert c.state is State.READ_BOARD
    cells = with_tokens(c.turns.cells, (column if lands is None else lands, ROBOT))
    c.read(cells)
    return cells


# Found by mutation testing (mutmut on turns.py, 08 §1) [MES-74]


def test_before_any_game_the_state_says_idle_an_empty_board_and_nothing_played():
    assert Turns().snapshot() == {
        "state": "IDLE",
        "reason": "",
        "turn": "human",
        "board": [EMPTY] * CELLS,
        "legal": list(range(COLUMNS)),
        "last": None,
        "result": "",
        "moves": 0,
        "first": "human",
        "history": [],
        "token_source": "hand",
        "seconds": {"move": 0.0, "read": 0.0, "wait": 0.0, "think": 0.0},
        "lost": 0,
        "missed": 0,
    }
    assert Turns().accepts_new_game(ready=True)


def test_each_transition_is_said_once_with_the_seconds_spent_in_the_state_left():
    """The node logs it: the game's timeline in the stack's log."""
    c = Clock()
    assert c.step(new_game=True).transition == "IDLE → MOVE (0.00 s in IDLE)"
    assert c.label == "MOVE"  # no reason on a plain transition
    c.wait(0.3)
    assert c.out.transition is None
    assert c.step(move_reply=(True, "at HANDOVER")).transition == "MOVE → READ_BOARD (0.40 s in MOVE)"


def test_the_engine_is_asked_for_the_robots_side_once_it_has_tokens():
    c = new_game()
    human_plays(c, 3)
    robot_plays(c, 2)
    cells = human_plays(c, 4)
    me, mask = c.out.think
    assert me != 0
    assert (me, mask) == game.from_cells(cells, ROBOT)


def test_a_win_with_the_move_itself_is_a_win_in_one_ply():
    c = new_game()
    cells = human_plays(c, 3)
    c.step(engine=choice(2, score=1000 - 1 - 1))  # 1 stone on the board, the win is this move
    c.step(move_reply=(True, "dropped"))
    c.read(with_tokens(cells, (2, ROBOT)))
    assert c.turns.last["wins_in"] == 1
    assert "loses_in" not in c.turns.last


def test_the_seconds_are_published_to_the_hundredth():
    c = new_game()
    c.step(dt=0.123456, human_done=True)
    assert c.turns.snapshot()["seconds"]["wait"] == 0.12


def test_a_new_game_forgets_the_last_games_result():
    c = new_game()
    c.turns.state, c.turns.result = State.GAME_OVER, "robot"
    c.step(new_game=True)
    assert (c.turns.result, c.turns.last) == ("", None)


def test_two_tokens_lost_in_a_game_are_counted_each_time():
    c = new_game()
    cells = human_plays(c, 3)
    for lost in (1, 2):
        c.step(engine=choice(4))
        c.step(move_reply=(True, ""))
        for _ in range(52):  # fresh reads without the robot's token, to the read timeout
            c.read(cells)
        assert (c.label, c.turns.lost) == ("WAIT_HUMAN:token_lost", lost)
        c.step(human_done=True)  # a token given again
        c.read(cells)
        assert c.state is State.THINK
    c.step(engine=choice(4))
    c.step(move_reply=(False, NO_TOKEN))
    assert c.turns.lost == 3


def test_the_turn_is_the_robots_from_the_persons_token_to_its_own():
    c = new_game()
    human_plays(c, 3)
    assert (c.state, c.turns.turn) == (State.THINK, "robot")
    c.step(engine=choice(4))
    assert (c.state, c.turns.turn) == (State.MOVE, "robot")


def test_tokens_in_another_column_are_counted_each_time_the_others_never():
    c = new_game()
    human_plays(c, 3)
    robot_plays(c, 4)
    assert c.turns.missed == 0
    for missed in (1, 2):
        human_plays(c, 3)
        robot_plays(c, 4, lands=5)
        assert c.turns.missed == missed


# A turn, call by call (P2-02 §5)


def test_a_new_game_takes_the_arm_to_handover_then_reads_the_empty_board():
    c = new_game()
    assert c.turns.snapshot()["legal"] == list(range(COLUMNS))


def test_a_human_move_then_a_robot_move():
    c = new_game()
    human_plays(c, 3)
    assert c.state is State.THINK
    assert c.out.think == game.from_cells(c.turns.cells, ROBOT)
    robot_plays(c, 2)
    assert c.state is State.WAIT_HUMAN
    assert c.turns.turn == "human"
    assert c.turns.moves == 2
    assert c.turns.last == {"who": "robot", "column": 2, "planned": 2, "score": 0, "depth": 1}


def test_the_robots_last_move_says_why_score_depth_and_a_proven_win():
    """#244: what the engine knows of its move, for the console and the debug image."""
    c = new_game()
    cells = human_plays(c, 3)
    c.step(engine=choice(2, score=4, depth=10))
    c.step(move_reply=(True, "dropped"))
    c.read(with_tokens(cells, (2, ROBOT)))
    assert c.turns.last == {"who": "robot", "column": 2, "planned": 2, "score": 4, "depth": 10}
    cells = human_plays(c, 3)
    c.step(engine=choice(6, score=1000 - 3 - 3))  # 3 stones on the board, the win 3 plies away
    c.step(move_reply=(True, "dropped"))
    c.read(with_tokens(cells, (6, ROBOT)))
    assert c.turns.last["wins_in"] == 3
    assert "loses_in" not in c.turns.last
    cells = human_plays(c, 0)
    c.step(engine=choice(0, score=5 + 2 - 1000, depth=3))  # 5 stones, the loss 2 plies away
    c.step(move_reply=(True, "dropped"))
    c.read(with_tokens(cells, (0, ROBOT)))
    assert c.turns.last["loses_in"] == 2
    assert c.turns.last["depth"] == 3


def test_the_humans_last_move_carries_no_engine_reason():
    c = new_game()
    human_plays(c, 3)
    robot_plays(c, 2)
    human_plays(c, 4)
    assert c.turns.last == {"who": "human", "column": 4, "planned": 4}


def test_the_history_and_who_started_rebuild_the_board():
    """#245: the columns in the order the tokens were seen; `moves` is only a count."""
    c = new_game(robot_first=True)
    c.step(human_done=True)
    c.read(EMPTY_BOARD)
    robot_plays(c, 3)
    human_plays(c, 3)
    robot_plays(c, 4, lands=5)  # the camera says where it went
    human_plays(c, 0)
    state = c.turns.snapshot()
    assert (state["first"], state["history"]) == ("robot", [3, 3, 5, 0])
    players = (ROBOT, HUMAN) if state["first"] == "robot" else (HUMAN, ROBOT)
    rebuilt = with_tokens(
        EMPTY_BOARD, *[(column, players[i % 2]) for i, column in enumerate(state["history"])]
    )
    assert rebuilt == c.turns.cells == tuple(state["board"])


def test_a_new_game_starts_the_history_again():
    c = new_game()
    human_plays(c, 2)
    c.step(stop=True)
    c.step(new_game=True)
    assert (c.turns.snapshot()["first"], c.turns.snapshot()["history"]) == ("human", [])


def test_the_seconds_of_a_game_are_counted_by_state():
    """#245: where a game's time goes, for the bench's CSV. Each state's time is added when it is left."""
    c = Clock()
    c.step(new_game=True)
    c.wait(0.5)  # the arm goes to HANDOVER
    c.step(move_reply=(True, ""))
    c.read(EMPTY_BOARD)
    c.wait(2.0)  # the person thinks
    human_plays(c, 3)
    c.wait(0.3)  # the engine searches
    c.step(engine=choice(3))
    c.wait(14.0)  # the drop
    c.step(move_reply=(True, ""))
    c.read(with_tokens(c.turns.cells, (3, ROBOT)))
    seconds = c.turns.snapshot()["seconds"]
    assert seconds == {
        "move": pytest.approx(0.6 + 14.1),
        "read": pytest.approx(0.15 + 0.15 + 0.15),
        "wait": pytest.approx(2.1),
        "think": pytest.approx(0.4),
    }
    c.wait(1.0)  # the person waits, then asks for a new game
    c.step(new_game=True)
    assert set(c.turns.snapshot()["seconds"].values()) == {0.0}


def test_a_stale_read_is_not_yet_an_error():
    c = new_game()
    c.step(human_done=True)
    c.read(EMPTY_BOARD)  # the token is still falling
    assert c.state is State.READ_BOARD
    c.read(with_tokens(EMPTY_BOARD, (0, HUMAN)))
    assert c.state is State.THINK


def test_the_robot_plays_first():
    c = new_game(robot_first=True)
    c.step(human_done=True)  # a token given, nothing dropped
    c.read(EMPTY_BOARD)
    assert c.state is State.THINK


def test_a_token_where_none_was_expected():
    c = new_game(robot_first=True)
    assert c.turns.accepts_human_done
    c.step(human_done=True)
    c.read(with_tokens(EMPTY_BOARD, (2, HUMAN)))  # the robot's turn, yet a token appeared
    assert c.label == "FIX_BOARD:unexpected_token"
    assert c.turns.accepts_human_done


def test_one_transition_at_most_per_step():
    c = Clock()
    c.board = Board(EMPTY_BOARD, True, "", 0.0)
    c.step(new_game=True, move_reply=(True, ""))  # the reply of a move not started yet: ignored
    assert c.state is State.MOVE


# The failure table (P2-02 §5)


def test_a_new_game_needs_the_stack_ready_and_is_refused_in_a_move():
    c = Clock()
    c.step(new_game=True, ready=False)
    assert c.state is State.IDLE
    assert not c.turns.accepts_new_game(ready=False)
    c.step(new_game=True)
    for state in (State.MOVE,):
        assert c.state is state
        assert not c.turns.accepts_new_game(ready=True)
        assert c.step(new_game=True).transition is None


@pytest.mark.parametrize("state", [State.THINK, State.READ_BOARD])
def test_a_new_game_is_refused_while_thinking_or_reading(state):
    c = new_game()
    human_plays(c, 0) if state is State.THINK else c.step(human_done=True)
    assert c.state is state
    assert not c.turns.accepts_new_game(ready=True)
    c.step(new_game=True)
    assert c.state is state


def test_a_board_not_empty_at_a_new_game_is_not_yet_then_a_board_to_fix():
    """The reader may still show the last game's board right after the reset: only the timeout judges."""
    c = Clock()
    c.step(new_game=True)
    c.step(move_reply=(True, ""))
    for _ in range(20):  # 3 s of reads
        c.read(with_tokens(EMPTY_BOARD, (3, HUMAN)))
    assert c.state is State.READ_BOARD
    c.wait(5.1)
    assert c.label == "FIX_BOARD:board_not_empty"


def test_the_last_games_board_then_the_empty_one_starts_the_game():
    c = Clock()
    c.step(new_game=True)
    c.step(move_reply=(True, ""))
    c.read(with_tokens(EMPTY_BOARD, (3, HUMAN)))
    c.read(EMPTY_BOARD, valid=False, reason="markers:2")
    assert c.state is State.READ_BOARD
    c.read(EMPTY_BOARD)
    assert c.state is State.WAIT_HUMAN


def test_a_grid_never_read_at_a_new_game_takes_the_readers_reason():
    c = Clock()
    c.step(new_game=True)
    c.step(move_reply=(True, ""))
    for _ in range(60):
        c.read(EMPTY_BOARD, valid=False, reason="markers:2")
    assert c.label == "FIX_BOARD:markers:2"


def test_a_human_token_never_seen():
    c = new_game()
    c.step(human_done=True)
    c.wait(5.1)
    assert c.label == "FIX_BOARD:not_seen:human"


@pytest.mark.parametrize(
    ("tokens", "why"),
    [
        pytest.param([(0, HUMAN), (1, HUMAN)], "two tokens", id="two"),
        pytest.param([(0, ROBOT)], "the robot's color", id="color"),
    ],
)
def test_not_one_human_token(tokens, why):
    c = new_game()
    c.step(human_done=True)
    c.read(with_tokens(EMPTY_BOARD, *tokens))
    assert c.label == "FIX_BOARD:not_one_human_token", why


def test_a_token_not_at_its_landing_cell():
    c = new_game()
    c.step(human_done=True)
    cells = list(EMPTY_BOARD)
    cells[COLUMNS + 2] = HUMAN  # row 1 over an empty cell: the reader would call it invalid, judged here too
    c.read(cells)
    assert c.label == "FIX_BOARD:not_one_human_token"


def test_an_invalid_board_takes_the_readers_reason():
    c = new_game()
    c.step(human_done=True)
    c.read(EMPTY_BOARD, valid=False, reason="gravity")
    assert c.label == "FIX_BOARD:gravity"


def test_a_board_never_stable_after_the_robots_drop():
    c = new_game()
    human_plays(c, 3)
    c.step(engine=choice(3))
    c.step(move_reply=(True, ""))
    c.board = None  # the reader stays silent
    c.wait(5.1)
    assert c.label == "FIX_BOARD:not_seen:robot"


def test_no_token_in_the_jaws_gives_the_turn_back_to_the_robot():
    c = new_game()
    human_plays(c, 3)
    c.step(engine=choice(4))
    c.step(move_reply=(False, NO_TOKEN))
    assert c.label == "WAIT_HUMAN:no_token"
    assert c.turns.turn == "robot"
    assert c.turns.lost == 1
    c.step(human_done=True)  # a token given
    c.read(c.turns.cells)
    assert c.state is State.THINK


def test_a_token_lost_during_the_move():
    c = new_game()
    cells = human_plays(c, 3)
    c.step(engine=choice(4))
    c.step(move_reply=(True, ""))
    for _ in range(52):  # fresh reads keep coming, all without the robot's token
        c.read(cells)
    assert c.label == "WAIT_HUMAN:token_lost"
    assert c.turns.turn == "robot"
    assert c.turns.lost == 1


def test_a_token_in_the_wrong_column_goes_on_from_where_it_fell():
    c = new_game()
    human_plays(c, 3)
    robot_plays(c, 4, lands=5)
    assert c.state is State.WAIT_HUMAN
    assert c.turns.missed == 1
    assert (c.turns.last["column"], c.turns.last["planned"]) == (5, 4)


def test_the_brain_refuses():
    c = new_game()
    human_plays(c, 3)
    c.step(engine=choice(4))
    c.step(move_reply=(False, "unreachable"))
    assert c.label == "ERROR:brain_refused:unreachable"


def test_a_move_cut_by_a_stop_has_one_reason_whichever_message_comes_first():
    """/brain/stop called by someone else: the drop's reply `stopped` may come before /brain_state."""
    c = new_game()
    human_plays(c, 3)
    c.step(engine=choice(4))
    c.step(move_reply=(False, "stopped"))
    assert c.label == "ERROR:stopped"


def test_the_handover_fails():
    c = Clock()
    c.step(new_game=True)
    c.step(move_reply=(False, "not IDLE"))
    assert c.label == "ERROR:brain_refused:not IDLE"


def test_the_engine_times_out():
    c = new_game()
    human_plays(c, 3)
    c.wait(5.1)
    assert c.label == "ERROR:timeout:THINK"


def test_a_move_times_out_and_stops_the_brain():
    c = new_game()
    human_plays(c, 3)
    c.step(engine=choice(4))
    stopped = False
    for _ in range(901):
        stopped = stopped or c.step().stop_brain
    assert c.label == "ERROR:timeout:MOVE"
    assert stopped


@pytest.mark.parametrize("state", [State.IDLE, State.MOVE, State.READ_BOARD, State.WAIT_HUMAN, State.THINK])
def test_stop_from_any_state_stops_the_brain(state):
    c = Clock()
    if state is not State.IDLE:
        c = new_game()
        if state is State.THINK:
            human_plays(c, 0)
        elif state is State.MOVE:
            human_plays(c, 0)
            c.step(engine=choice(1))
        elif state is State.READ_BOARD:
            c.step(human_done=True)
    assert c.state is state
    assert c.step(stop=True).stop_brain
    assert c.label == "ERROR:stopped"


def test_a_brain_stopped_by_someone_else_ends_the_move():
    c = new_game()
    human_plays(c, 0)
    c.step(engine=choice(1))
    out = c.step(brain_states=("STOPPED",))
    assert c.label == "ERROR:stopped"
    assert not out.stop_brain  # already stopped


@pytest.mark.parametrize("state", [State.WAIT_HUMAN, State.FIX_BOARD, State.ERROR])
def test_a_new_game_leaves_a_game_in_any_resting_state(state):
    c = new_game()
    if state is State.FIX_BOARD:  # a dead end in sim: no hand can fix the grid
        c.step(human_done=True)
        c.wait(5.1)
    elif state is State.ERROR:
        c.step(stop=True)
    assert c.state is state
    assert c.step(new_game=True).handover
    assert c.state is State.MOVE
    assert c.turns.cells == EMPTY_BOARD


def test_human_done_leaves_fix_board_to_read_again():
    c = new_game()
    c.step(human_done=True)
    c.wait(5.1)
    assert c.state is State.FIX_BOARD
    c.step(human_done=True)
    assert c.state is State.READ_BOARD


# The end of a game


def test_the_human_wins():
    c = new_game()
    for column in range(3):
        human_plays(c, column)
        robot_plays(c, column)
    human_plays(c, 3)
    assert c.state is State.GAME_OVER
    assert c.turns.result == "human"


def test_the_robot_wins():
    c = new_game(robot_first=True)
    c.step(human_done=True)
    c.read(EMPTY_BOARD)
    for column in range(3):
        robot_plays(c, column)
        human_plays(c, column)
    robot_plays(c, 3)
    assert c.state is State.GAME_OVER
    assert c.turns.result == "robot"


# A draw: drops that never make four, the human first (the prototype's list)
DRAW = [
    1,
    1,
    4,
    2,
    3,
    3,
    4,
    2,
    2,
    5,
    6,
    2,
    3,
    4,
    2,
    4,
    6,
    5,
    1,
    0,
    1,
    5,
    3,
    4,
    4,
    1,
    1,
    6,
    2,
    6,
    6,
    6,
    0,
    0,
    3,
    0,
]
DRAW += [5, 3, 0, 0, 5, 5]


def test_a_full_board_is_a_draw():
    c = new_game()
    for human, robot in zip(DRAW[0::2], DRAW[1::2], strict=True):
        human_plays(c, human)
        robot_plays(c, robot)
    assert c.state is State.GAME_OVER
    assert c.turns.result == "draw"
    assert c.turns.moves == CELLS


def test_the_snapshot_holds_what_the_console_and_the_bench_need():
    c = new_game()
    human_plays(c, 3)
    snapshot = c.turns.snapshot()
    assert set(snapshot) == {
        "state",
        "reason",
        "turn",
        "board",
        "legal",
        "last",
        "result",
        "moves",
        "lost",
        "missed",
        "first",
        "history",
        "seconds",
        "token_source",
    }
    assert snapshot["state"] == "THINK"
    assert snapshot["board"][3] == HUMAN


# Random games against a fake world


class World:
    """The grid, tokens in flight, the reader (5 Hz, a board once 5 reads agree), the brain (replies at the
    end of a move), and the faults to inject."""

    FALL_S = 0.4

    def __init__(self, rng, faults):
        self.rng, self.f = rng, faults
        self.grid = [EMPTY] * CELLS
        self.flying = []  # (lands at, column, color)
        self.frames = deque(maxlen=5)
        self.next_read = 0.0
        self.held = False
        self.motion = None  # (end time, column or None for HANDOVER)
        self.reply = None
        self.labels = []
        self.board = None
        self.rails = list(faults.get("rails", (7, 7, 7)))  # tokens left in each rail of the dispenser
        self.stations = None

    def drop_token(self, now, column, color):
        self.flying.append((now + self.FALL_S, column, color))

    def land(self, column, color):
        for row in range(game.ROWS):
            if self.grid[row * COLUMNS + column] == EMPTY:
                self.grid[row * COLUMNS + column] = color
                return

    def tick(self, now):
        for flying in [f for f in self.flying if f[0] <= now]:
            self.flying.remove(flying)
            self.land(flying[1], flying[2])
        if now >= self.next_read:
            self.next_read = now + 0.2
            self.frames.append(tuple(self.grid))
            if self.rng.random() >= self.f.get("unseen", 0):
                self.stations = Stations(tuple(left > 0 for left in self.rails), now)
            unstable = self.rng.random() < self.f.get("never_stable", 0)
            if len(self.frames) == 5 and len(set(self.frames)) == 1 and not unstable:
                bad = self.rng.random() < self.f.get("invalid", 0)
                self.board = Board(self.frames[-1], not bad, "gravity" if bad else "", now)
        if self.motion and now >= self.motion[0]:
            column, self.motion = self.motion[1], None
            if column == "pick":
                self.held = True
            elif column is not None:
                self.held = False
                r = self.rng.random()
                if r >= self.f.get("lost", 0):  # else the token fell out of the grid
                    if r < self.f.get("lost", 0) + self.f.get("wrong", 0):
                        column = max(0, min(COLUMNS - 1, column + self.rng.choice((-1, 1))))
                    if self.grid[(game.ROWS - 1) * COLUMNS + column] == EMPTY:
                        self.drop_token(now, column, ROBOT)
            self.reply = (True, "")
        states, self.labels = tuple(self.labels), []
        reply, self.reply = self.reply, None
        return self.board, states, reply

    def brain_move(self, now, column):
        """/brain/drop (a column) or /brain/handover (None): refused at once, or a reply at the end."""
        if self.rng.random() < self.f.get("refuse", 0):
            self.reply = (False, "unreachable")
        elif column is not None and not self.held:
            self.reply = (False, NO_TOKEN)  # the jaws closed on nothing, the arm did not move
        else:
            self.motion = (now + (1.0 if column is None else 6.0), column)

    def brain_pick(self, now, station):
        """/brain/pick: refused at once, or a reply at the end, no_token when the jaws closed on nothing."""
        assert self.rails[station] > 0, "the arm is sent to a station nothing waits at"
        if self.rng.random() < self.f.get("refuse", 0):
            self.reply = (False, "unreachable")
        elif self.rng.random() < self.f.get("slips", 0):
            self.reply = (False, NO_TOKEN)
        else:
            self.rails[station] -= 1
            self.motion = (now + 5.0, "pick")

    def brain_stop(self):
        if self.motion:
            self.motion = None
            self.reply = (False, "stopped")
        self.labels.append("STOPPED")


def play_one(seed, faults, depth=2, dispenser=False):
    """One game; (turns, ticks the arm moved outside MOVE). With a dispenser, nobody gives a token: the
    person fills it when the robot waits."""
    rng = random.Random(seed)
    w, t, now = World(rng, faults), Turns(dispenser=dispenser), 0.0
    inputs = {"new_game": True, "robot_first": seed % 2 == 1}
    engine = None
    stop_at = rng.uniform(5, 200) if rng.random() < faults.get("stop", 0) else None
    moved_outside = 0
    for _ in range(40000):
        now += 0.1
        board, states, reply = w.tick(now)
        stop = stop_at is not None and now >= stop_at
        stop_at = None if stop else stop_at
        out = t.step(
            now,
            Inputs(
                board=board,
                brain_states=states,
                move_reply=reply,
                engine=engine,
                stop=stop,
                stations=w.stations,
                **inputs,
            ),
        )
        inputs, engine = {}, None
        if out.think:
            engine = game.best_move(*out.think, None, depth, seed=seed)
        if out.stop_brain:
            w.brain_stop()
        if out.drop is not None or out.handover:
            assert out.drop is None or out.drop in t.snapshot()["legal"]
            w.brain_move(now, out.drop)
        if out.pick is not None:
            w.brain_pick(now, out.pick)
        moved_outside += w.motion is not None and t.state is not State.MOVE
        snapshot = t.snapshot()
        if t.state is State.WAIT_HUMAN:  # the console's or the bench's key: drop, give a token, done
            if snapshot["turn"] == "human":
                w.drop_token(now, rng.choice(snapshot["legal"]), HUMAN)
            if dispenser and snapshot["turn"] == "robot":
                w.rails = [7, 7, 7]
            elif not dispenser:
                w.held = rng.random() >= faults.get("no_give", 0)
            inputs = {"human_done": True}
        elif t.state in (State.GAME_OVER, State.ERROR, State.FIX_BOARD):
            return t, moved_outside
    raise AssertionError(f"game {seed} never ended: {t.label}")


@pytest.mark.parametrize(
    "faults",
    [
        pytest.param({}, id="clean"),
        pytest.param({"lost": 0.2}, id="token-lost"),
        pytest.param({"wrong": 0.2}, id="wrong-column"),
        pytest.param({"no_give": 0.2}, id="no-token"),
        pytest.param({"refuse": 0.05}, id="brain-refuses"),
        pytest.param({"invalid": 0.01}, id="invalid-reads"),
        pytest.param({"never_stable": 0.95}, id="unstable-board"),
        pytest.param({"stop": 0.3}, id="stops"),
        pytest.param(
            {"lost": 0.1, "wrong": 0.1, "no_give": 0.1, "refuse": 0.02, "stop": 0.1}, id="everything"
        ),
    ],
)
def test_random_games_move_the_arm_in_move_only_and_always_end(faults):
    for seed in range(30):
        t, moved_outside = play_one(seed, faults)
        assert moved_outside == 0, (seed, t.label)
        if not faults:
            assert t.state is State.GAME_OVER, (seed, t.label)


@pytest.mark.parametrize(
    "faults",
    [
        pytest.param({}, id="clean"),
        pytest.param({"rails": (1, 0, 2)}, id="few-tokens"),
        pytest.param({"slips": 0.2}, id="pick-slips"),
        pytest.param({"unseen": 0.9}, id="dispenser-rarely-read"),
        pytest.param({"lost": 0.2, "wrong": 0.1}, id="token-lost-or-wrong-column"),
        pytest.param({"stop": 0.3}, id="stops"),
        pytest.param(
            {"rails": (2, 1, 0), "slips": 0.1, "unseen": 0.5, "lost": 0.1, "refuse": 0.02, "stop": 0.1},
            id="everything",
        ),
    ],
)
def test_random_games_with_a_dispenser_never_pick_at_an_empty_station_and_always_end(faults):
    """The fake brain asserts that the station it is sent to holds a token."""
    for seed in range(30):
        t, moved_outside = play_one(seed, faults, dispenser=True)
        assert moved_outside == 0, (seed, t.label)
        if not {"refuse", "stop", "lost", "wrong"} & set(faults):  # nothing the person cannot answer
            assert t.state is State.GAME_OVER, (seed, t.label)


# The robot takes its tokens at the dispenser (#259): a robot's MOVE is a pick, then the drop.


def to_move(c, column=3):
    """From a new game with a dispenser: the human plays, the engine answers `column`: MOVE, no call yet."""
    human_plays(c, 0)
    out = c.step(engine=choice(column))
    assert c.state is State.MOVE
    assert (out.pick, out.drop, out.handover) == (None, None, False)
    return c


def test_with_a_dispenser_the_state_says_so():
    assert Turns(dispenser=True).snapshot()["token_source"] == "dispenser"


def test_a_new_game_with_a_dispenser_still_starts_at_handover():
    c = Clock(dispenser=True)
    out = c.step(new_game=True)
    assert (out.handover, out.pick, out.drop) == (True, None, None)


def test_the_robot_takes_its_token_at_the_highest_station_that_holds_one_then_drops_it():
    """The highest first: the wrist turns the least from HANDOVER, 8.2 s a pick against 12.3 [MES-77]."""
    c = to_move(new_game(dispenser=True))
    out = c.sees(1, 1, 0)
    assert (out.pick, out.drop) == (1, None)
    assert c.state is State.MOVE
    assert c.step().pick is None  # called once
    out = c.step(move_reply=(True, "done in 9.4 s"))
    assert (out.pick, out.drop) == (None, 3)  # the token is in the jaws at HANDOVER: the drop
    assert c.state is State.MOVE
    c.step(move_reply=(True, "done in 12.0 s"))
    assert c.state is State.READ_BOARD
    c.read(with_tokens(c.turns.cells, (3, ROBOT)))
    assert c.label == "WAIT_HUMAN"
    assert c.turns.turn == "human"
    assert c.turns.last["column"] == 3


def test_the_station_is_chosen_on_a_read_newer_than_the_move():
    """The last read may be older than the last pick: the queue it showed has moved since."""
    c = new_game(dispenser=True)
    c.sees(1, 1, 1)
    to_move(c)
    assert c.step().pick is None
    assert c.sees(1, 1, 1).pick == 2


def test_an_empty_dispenser_waits_for_the_person_who_fills_it_and_says_done():
    c = to_move(new_game(dispenser=True))
    out = c.sees(0, 0, 0)
    assert (out.pick, out.drop, out.stop_brain) == (None, None, False)
    assert c.label == "WAIT_HUMAN:dispenser_empty"
    assert c.turns.turn == "robot"
    assert c.turns.lost == 0  # no token was lost: none was there
    board = c.turns.cells
    c.step(human_done=True)
    c.read(board)  # nothing new on the grid: the robot plays
    assert c.state is State.THINK
    c.step(engine=choice(3))
    assert c.sees(1, 0, 0).pick == 0


def test_a_dispenser_the_camera_does_not_read_waits_for_the_person_after_the_read_timeout():
    c = to_move(new_game(dispenser=True))
    c.wait(c.turns.p.read_timeout_s - 0.3)
    assert c.state is State.MOVE
    c.wait(0.5)
    assert c.label == "WAIT_HUMAN:dispenser_not_seen"
    assert c.turns.turn == "robot"
    assert not c.out.stop_brain  # nothing moves yet


def test_a_pick_that_closes_on_nothing_gives_the_turn_back_as_a_token_not_given():
    c = to_move(new_game(dispenser=True))
    c.sees(1, 1, 1)
    out = c.step(move_reply=(False, NO_TOKEN))
    assert out.drop is None
    assert c.label == "WAIT_HUMAN:no_token"
    assert c.turns.lost == 1


def test_a_pick_the_brain_refuses_is_an_error_and_nothing_is_dropped():
    c = to_move(new_game(dispenser=True))
    c.sees(1, 1, 1)
    out = c.step(move_reply=(False, "unreachable"))
    assert out.drop is None
    assert c.label == "ERROR:brain_refused:unreachable"


def test_the_moves_timeout_covers_the_choice_of_the_station_the_pick_and_the_drop():
    c = to_move(new_game(dispenser=True))
    c.wait(2.0)  # the dispenser read late
    c.sees(1, 1, 1)
    c.wait(40.0)
    c.step(move_reply=(True, "done"))
    c.wait(c.turns.p.move_timeout_s - 45.0)
    assert c.state is State.MOVE
    c.wait(6.0)
    assert c.label == "ERROR:timeout:MOVE"
    assert c.turns.seconds["move"] > c.turns.p.move_timeout_s


def test_a_stop_during_the_pick_ends_the_game():
    c = to_move(new_game(dispenser=True))
    c.sees(1, 1, 1)
    out = c.step(move_reply=(False, "stopped"))
    assert out.drop is None
    assert c.label == "ERROR:stopped"


def test_without_a_dispenser_the_stations_are_ignored():
    c = new_game()
    human_plays(c, 0)
    c.stations = Stations((False, False, False), c.now)
    out = c.step(engine=choice(3))
    assert (out.pick, out.drop) == (None, 3)


def test_a_game_stopped_during_a_pick_starts_again_with_a_handover_not_a_drop():
    c = to_move(new_game(dispenser=True))
    c.sees(1, 1, 1)
    c.step(stop=True)
    assert c.label == "ERROR:stopped"
    assert c.step(new_game=True).handover
    out = c.step(move_reply=(True, "at HANDOVER"))
    assert (out.pick, out.drop) == (None, None)
    assert c.state is State.READ_BOARD
