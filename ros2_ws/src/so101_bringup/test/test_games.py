"""The game bench, `evaluate --games` (#87, docs/project/08 §3): the scoring against the ground truth, one
game's flow on a scripted client, the run's checks and its CSV. Pure: no ROS, no sim; the bench on the stack
is the CI's game job."""

import argparse
import csv
import random

import pytest

from so101_bringup import evaluate, games
from so101_connect4 import game
from so101_connect4.game import CELLS, COLUMNS, EMPTY, HUMAN, ROBOT

EMPTY_BOARD = [EMPTY] * CELLS


def with_tokens(cells, *tokens):
    cells = list(cells)
    for column, player in tokens:
        row = next(r for r in range(game.ROWS) if cells[r * COLUMNS + column] == EMPTY)
        cells[row * COLUMNS + column] = player
    return cells


NOBODY_WON = with_tokens(EMPTY_BOARD, (0, HUMAN), (1, ROBOT))
ROBOT_WON = with_tokens(EMPTY_BOARD, *[(c, p) for c in range(3) for p in (ROBOT, HUMAN)], (3, ROBOT))


def a_state(state="GAME_OVER", cells=ROBOT_WON, **fields):
    legal = [c for c in range(COLUMNS) if cells[35 + c] == EMPTY]
    base = {
        "ready": True,
        "state": state,
        "reason": "",
        "turn": "human",
        "board": list(cells),
        "legal": legal,
    }
    played = {"first": "robot", "history": [0, 0, 1, 1, 2, 2, 3], "moves": 7, "lost": 0, "missed": 0}
    seconds = {"move": 56.5, "read": 2.1, "wait": 0.7, "think": 0.8}  # 4 robot moves and the handover
    return base | {"last": None, "result": "robot", "seconds": seconds} | played | fields


# The scoring


def test_a_game_over_the_truth_agrees_with_is_a_success():
    assert games.outcome(a_state(), "", ROBOT_WON, moved_rad=0.0) == (True, "")


@pytest.mark.parametrize(
    ("state", "why", "truth", "moved", "cause"),
    [
        (a_state("FIX_BOARD", reason="not_seen:human"), "", ROBOT_WON, 0.0, "fix_board:not_seen:human"),
        (a_state("ERROR", reason="timeout:MOVE"), "", ROBOT_WON, 0.0, "error:timeout:MOVE"),
        (a_state("THINK"), "bench_timeout", ROBOT_WON, 0.0, "bench_timeout"),
        (
            a_state("IDLE"),
            "new_game:not ready: no /brain/drop",
            ROBOT_WON,
            0.0,
            "new_game:not ready: no /brain/drop",
        ),
        (a_state(), "", ROBOT_WON, 0.2, "arm_moved"),
        (a_state(lost=1), "", ROBOT_WON, 0.0, "token_lost"),
        (a_state(missed=1), "", ROBOT_WON, 0.0, "wrong_column"),
        (a_state(), "", with_tokens(ROBOT_WON, (6, HUMAN)), 0.0, "wrong_board"),
        (a_state(result="human"), "", ROBOT_WON, 0.0, "wrong_winner"),
        (a_state(result="draw"), "", ROBOT_WON, 0.0, "wrong_winner"),
        (a_state(cells=NOBODY_WON, result="draw"), "", NOBODY_WON, 0.0, "wrong_winner"),  # a grid not full
    ],
)
def test_a_failed_game_gives_its_cause(state, why, truth, moved, cause):
    assert games.outcome(state, why, truth, moved) == (False, cause)


def test_the_summary_starts_with_games_never_with_success():
    """bench_report.sh reads the phase-1 line by `^success `: the games' line must not match it."""
    rows = [{"success": True, "duration_s": 90.0, "cause": ""}] * 8
    rows += [{"success": False, "duration_s": 600.0, "cause": "bench_timeout"}] * 2
    text = games.summary(rows)
    assert text == "games 8/10 (80 %); duration p50 90.0 s, p95 90.0 s, max 90.0 s; causes: bench_timeout: 2"


def test_the_summary_of_no_success_has_no_duration():
    assert games.summary([{"success": False, "duration_s": 1.0, "cause": "error:stopped"}]) == (
        "games 0/1 (0 %); duration: no success; causes: error:stopped: 1"
    )


def test_the_arm_is_measured_from_where_each_turn_of_the_person_began():
    """Target P2-05: the furthest an arm joint went from its place at the start of a WAIT_HUMAN."""
    still = games.Still()
    still.update(False, [0.0, 0.0])  # the robot moves: not counted
    still.update(False, [1.0, 0.5])
    still.update(True, [1.0, 0.5])
    still.update(True, [1.0, 0.51])
    assert still.moved == pytest.approx(0.01)
    still.update(False, [0.2, 0.5])  # its own move
    still.update(True, [0.2, 0.5])  # the next turn starts from the new place
    still.update(True, [0.2, 0.3])
    assert still.moved == pytest.approx(0.2)
    still.reset()
    assert still.moved == 0.0


def test_a_games_row_holds_the_fields_of_the_csv_in_order():
    row = games.row(3, 0, "engine", True, a_state(moves=7), "", ROBOT_WON, 0.0, 61.234)
    assert list(row) == games.FIELDS
    assert (row["game"], row["first"], row["human"], row["plies"], row["result"]) == (
        3,
        "robot",
        "engine",
        7,
        "robot",
    )
    assert (row["duration_s"], row["success"], row["cause"]) == (61.2, True, "")
    # #245: the columns in order, and where the time went: a robot's move, its search, a read of the grid
    assert row["history"] == "0011223"
    assert (row["move_s"], row["think_s"], row["read_s"]) == (14.1, 0.2, 0.3)
    assert (
        games.row(4, 0, "random", False, a_state("THINK"), "bench_timeout", ROBOT_WON, 0.0, 600.0)["first"]
        == "human"
    )


# The scripted person


@pytest.mark.parametrize("human", ["random", "engine"])
def test_the_scripted_person_only_plays_legal_columns(human):
    cells = with_tokens(EMPTY_BOARD, *[(3, p) for p in (HUMAN, ROBOT) * 3])  # column 3 is full
    rng = random.Random(0)
    played = {games.choose(human, a_state("WAIT_HUMAN", cells), rng) for _ in range(30)}
    assert played
    assert played <= {0, 1, 2, 4, 5, 6}


def test_the_engine_person_takes_its_win():
    cells = with_tokens(EMPTY_BOARD, *[(c, p) for c in range(3) for p in (HUMAN, ROBOT)])
    assert games.choose("engine", a_state("WAIT_HUMAN", cells), random.Random(0)) == 3


# One game's flow


class ScriptedClient:
    """client.GameClient on a script: each wait() shows the next state; `calls` in order."""

    def __init__(self, states, accepts=(True, ""), plays=(True, "")):
        self.script, self.state, self.calls = list(states), a_state("IDLE"), []
        self.accepts, self.plays = accepts, plays

    def new_game(self, robot_first=False):
        self.calls.append(("new_game", robot_first))
        return self.accepts

    def wait(self, condition, timeout_s):
        while not condition(self.state):  # the state it has first, as the client
            if not self.script:
                return False
            self.state = self.script.pop(0)
        return True

    def play(self, column=None):
        self.calls.append(("play", column))
        assert len(self.calls) < 50, "the same turn is played again and again"
        if isinstance(self.plays, Exception):
            raise self.plays
        return self.plays

    def stop(self):
        self.calls.append(("stop",))
        return True, ""


def waiting(turn, cells=EMPTY_BOARD):
    return a_state("WAIT_HUMAN", cells, turn=turn, result="", moves=0)


def test_a_game_plays_each_turn_once_up_to_game_over():
    states = [
        a_state("MOVE"),
        waiting("human"),
        a_state("THINK"),
        waiting("robot"),
        a_state("MOVE"),
        a_state(),
    ]
    client = ScriptedClient(states)
    state, why = games.play(client, lambda state: 5, robot_first=False, timeout_s=60.0)
    assert (state["state"], why) == ("GAME_OVER", "")
    assert client.calls == [("new_game", False), ("play", 5), ("play", None)]


def test_a_refused_new_game_ends_the_game_with_the_refusal():
    client = ScriptedClient([], accepts=(False, "not ready: no /brain/drop"))
    assert games.play(client, None, False, 60.0)[1] == "new_game:not ready: no /brain/drop"
    assert client.calls == [("new_game", False)]


def test_a_game_that_never_ends_is_stopped_at_the_bench_timeout():
    client = ScriptedClient([a_state("MOVE"), waiting("human"), a_state("THINK")])
    state, why = games.play(client, lambda state: 0, False, 60.0)
    assert (state["state"], why) == ("THINK", "bench_timeout")
    assert client.calls[-1] == ("stop",)


def test_a_hand_that_fails_stops_the_game_with_its_message():
    client = ScriptedClient([a_state("MOVE"), waiting("human")], plays=(False, "no spare token"))
    assert games.play(client, lambda state: 0, False, 60.0)[1] == "hand:no spare token"
    assert client.calls[-1] == ("stop",)


def test_a_service_without_a_reply_fails_the_game_not_the_bench():
    """One hiccup of the stack must not cost the night its report: the game fails, the next one is played."""
    gone = RuntimeError("/connect4/human_done: no answer in 10 s")
    client = ScriptedClient([a_state("MOVE"), waiting("human")], plays=gone)
    assert (
        games.play(client, lambda state: 0, False, 60.0)[1] == "bench:/connect4/human_done: no answer in 10 s"
    )


@pytest.mark.parametrize("end", ["FIX_BOARD", "ERROR"])
def test_a_board_to_fix_or_an_error_ends_the_game_there(end):
    client = ScriptedClient([a_state("MOVE"), a_state(end, reason="x")])
    assert games.play(client, None, True, 60.0)[0]["state"] == end
    assert client.calls == [("new_game", True)]


# The run: its checks, its CSV


class FakeBench:
    """What games.run() reads of the bench, without ROS."""

    fields, topics, unit = games.FIELDS, games.BAG_TOPICS, "game"

    def __init__(self, missing=None, successes=None):
        self.missing_, self.successes, self.played, self.attempts = missing, successes, [], 1

    def driver_seed(self, timeout_s):
        return 0

    def missing(self, timeout_s):
        return self.missing_

    def game(self, number, seed, timeout_s):
        self.played.append((number, timeout_s))
        success = self.successes is None or self.successes[number - 1]
        row = dict.fromkeys(games.FIELDS, 0) | {"game": number, "seed": seed, "final_state": "GAME_OVER"}
        return row | {"duration_s": 90.0, "success": success, "cause": "" if success else "bench_timeout"}


def bench_args(tmp_path, count=2, **more):
    base = {"games": count, "out": tmp_path / "games.csv", "bags": None, "seed": None, "timeout": 600.0}
    return argparse.Namespace(**base | {"min_success": None, "human": "random"} | more)


def test_no_game_starts_without_the_ground_truth_or_the_game_node(tmp_path, capsys):
    bench = FakeBench(missing="evaluate: no /sim/ground_truth/connect4_board: expose_ground_truth:=true")
    assert games.run(bench_args(tmp_path), bench) == 2
    assert bench.played == []
    assert "connect4_board" in capsys.readouterr().err


def test_each_game_is_a_csv_row_and_a_line_then_the_summary(tmp_path, capsys):
    bench = FakeBench(successes=[True, False])
    assert games.run(bench_args(tmp_path), bench) == 0
    rows = list(csv.DictReader((tmp_path / "games.csv").open()))
    assert [row["game"] for row in rows] == ["1", "2"]
    assert list(rows[0]) == games.FIELDS
    out = capsys.readouterr().out.splitlines()
    assert out[-3:] == [
        "game 1: GAME_OVER, 90.0 s, success",
        "game 2: GAME_OVER, 90.0 s, bench_timeout",
        "games 1/2 (50 %); duration p50 90.0 s, p95 90.0 s, max 90.0 s; causes: bench_timeout: 1",
    ]


def test_min_success_fails_the_run_below_it(tmp_path):
    assert games.run(bench_args(tmp_path, min_success=2), FakeBench(successes=[True, False])) == 1
    assert games.run(bench_args(tmp_path, min_success=1), FakeBench(successes=[True, False])) == 0


def test_the_cli_takes_games_or_episodes_never_both_and_a_game_gets_ten_minutes(monkeypatch, tmp_path):
    seen = {}
    monkeypatch.setattr(evaluate, "start", lambda args: seen.update(vars(args)) or 0)
    assert evaluate.main(["--games", "3", "--out", str(tmp_path / "g.csv")]) == 0
    assert (seen["games"], seen["episodes"], seen["timeout"], seen["human"]) == (3, None, 600.0, "random")
    assert evaluate.main(["--episodes", "3", "--out", str(tmp_path / "e.csv")]) == 0
    assert (seen["games"], seen["episodes"], seen["timeout"]) == (None, 3, 60.0)
    for arguments in (["--out", "x.csv"], ["--games", "1", "--episodes", "1", "--out", "x.csv"]):
        with pytest.raises(SystemExit):
            evaluate.main(arguments)


def test_a_failed_games_rosbag_holds_what_a_pick_and_a_drop_were_aimed_at():
    """#264: the stations and the dispenser's pose were missing: a failed pick could not be replayed."""
    for topic in ("/connect4/grid_pose", "/connect4/dispenser_pose", "/connect4/stations", "/connect4/board"):
        assert topic in games.BAG_TOPICS
