"""The turns of a game (turns.py, phase2/P2-02 §5): each transition and each row of the failure table on a
fake clock, then random games against a fake world (ported from the prototype's test_turns.py): the arm moves
in MOVE only (target P2-05), a game always ends, no illegal drop. No ROS, no sim."""

import random
from collections import deque

import pytest

from so101_connect4 import game
from so101_connect4.game import CELLS, COLUMNS, EMPTY, HUMAN, ROBOT
from so101_connect4.turns import NO_TOKEN, Board, Inputs, State, Turns

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

    def __init__(self):
        self.turns, self.now, self.board, self.out = Turns(), 0.0, None, None

    def step(self, dt=0.1, **inputs):
        self.now += dt
        self.out = self.turns.step(self.now, Inputs(board=self.board, **inputs))
        return self.out

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


def new_game(robot_first=False):
    """A clock at WAIT_HUMAN of a new game: HANDOVER reached, the empty board read."""
    c = Clock()
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
    out = c.step(engine=column)
    assert c.state is State.MOVE
    assert out.drop == column
    assert not out.handover
    c.step(move_reply=(True, "dropped"))
    assert c.state is State.READ_BOARD
    cells = with_tokens(c.turns.cells, (column if lands is None else lands, ROBOT))
    c.read(cells)
    return cells


# A turn, call by call (P2-02 §5)


def test_a_new_game_takes_the_arm_to_handover_then_reads_the_empty_board():
    c = new_game()
    assert c.turns.snapshot()["legal"] == list(range(COLUMNS))


def test_a_human_move_then_a_robot_move():
    c = new_game()
    human_plays(c, 3)
    assert c.state is State.THINK
    me, mask = c.out.think
    assert (me, mask) == game.from_cells(c.turns.cells, ROBOT)
    robot_plays(c, 2)
    assert c.state is State.WAIT_HUMAN
    assert c.turns.turn == "human"
    assert c.turns.moves == 2
    assert c.turns.last == {"who": "robot", "column": 2, "planned": 2}


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
        assert not c.turns.accepts_new_game()
        assert c.step(new_game=True).transition is None


@pytest.mark.parametrize("state", [State.THINK, State.READ_BOARD])
def test_a_new_game_is_refused_while_thinking_or_reading(state):
    c = new_game()
    human_plays(c, 0) if state is State.THINK else c.step(human_done=True)
    assert c.state is state
    assert not c.turns.accepts_new_game()
    c.step(new_game=True)
    assert c.state is state


def test_a_board_not_empty_at_a_new_game():
    c = Clock()
    c.step(new_game=True)
    c.step(move_reply=(True, ""))
    c.read(with_tokens(EMPTY_BOARD, (3, HUMAN)))
    assert c.label == "FIX_BOARD:board_not_empty"


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
    c.step(engine=3)
    c.step(move_reply=(True, ""))
    c.board = None  # the reader stays silent
    c.wait(5.1)
    assert c.label == "FIX_BOARD:not_seen:robot"


def test_no_token_in_the_jaws_gives_the_turn_back_to_the_robot():
    c = new_game()
    human_plays(c, 3)
    c.step(engine=4)
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
    c.step(engine=4)
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
    assert c.turns.last == {"who": "robot", "column": 5, "planned": 4}


def test_the_brain_refuses():
    c = new_game()
    human_plays(c, 3)
    c.step(engine=4)
    c.step(move_reply=(False, "unreachable"))
    assert c.label == "ERROR:brain_refused:unreachable"


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
    c.step(engine=4)
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
            c.step(engine=1)
        elif state is State.READ_BOARD:
            c.step(human_done=True)
    assert c.state is state
    assert c.step(stop=True).stop_brain
    assert c.label == "ERROR:stopped"


def test_a_brain_stopped_by_someone_else_ends_the_move():
    c = new_game()
    human_plays(c, 0)
    c.step(engine=1)
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
            unstable = self.rng.random() < self.f.get("never_stable", 0)
            if len(self.frames) == 5 and len(set(self.frames)) == 1 and not unstable:
                bad = self.rng.random() < self.f.get("invalid", 0)
                self.board = Board(self.frames[-1], not bad, "gravity" if bad else "", now)
        if self.motion and now >= self.motion[0]:
            column, self.motion = self.motion[1], None
            if column is not None:
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

    def brain_stop(self):
        if self.motion:
            self.motion = None
            self.reply = (False, "stopped")
        self.labels.append("STOPPED")


def play_one(seed, faults, depth=2):
    """One game; (turns, ticks the arm moved outside MOVE)."""
    rng = random.Random(seed)
    w, t, now = World(rng, faults), Turns(), 0.0
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
            Inputs(board=board, brain_states=states, move_reply=reply, engine=engine, stop=stop, **inputs),
        )
        inputs, engine = {}, None
        if out.think:
            engine = game.best_move(*out.think, None, depth, seed=seed).column
        if out.stop_brain:
            w.brain_stop()
        if out.drop is not None or out.handover:
            assert out.drop is None or out.drop in t.snapshot()["legal"]
            w.brain_move(now, out.drop)
        moved_outside += w.motion is not None and t.state is not State.MOVE
        snapshot = t.snapshot()
        if t.state is State.WAIT_HUMAN:  # the console's or the bench's key: drop, give a token, done
            if snapshot["turn"] == "human":
                w.drop_token(now, rng.choice(snapshot["legal"]), HUMAN)
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
