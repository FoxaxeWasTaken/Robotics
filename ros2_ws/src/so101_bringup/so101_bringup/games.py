"""games - the game bench of phase 2 (docs/project/08 §3), behind `evaluate --games`:

    ros2 run so101_bringup evaluate --games N --out <csv> [--human random|engine] [--bags <dir>]
                                    [--seed S] [--timeout 600] [--min-success M]

Each game: /connect4/new_game (the robot first in the even games), the grid cleared, then a scripted person
plays through so101_connect4's client and the simulated hand until GAME_OVER, FIX_BOARD, ERROR or the bench
timeout (then /connect4/stop). The person plays a random legal column, or the engine's at its medium level.

Scored with the driver's ground truth (expose_ground_truth, for this bench only, C-06): a success is a game
over that the true grid agrees with, the board and the winner, with no token lost or in another column than
asked, and an arm that stayed still while the person played (target P2-05). One CSV row per game, a rosbag
per failed game with --bags, and the summary at the end, which starts with `games`.
"""

import random
import sys
import time
from collections import Counter

import numpy as np
from sensor_msgs.msg import JointState

from so101_bringup import evaluate
from so101_bringup.sim_hand import SimHand
from so101_connect4 import game
from so101_connect4.client import GameClient
from so101_interfaces.msg import Connect4Board

FIELDS = [
    "game", "seed", "human", "first", "final_state", "result", "plies", "missed", "lost",
    "duration_s", "success", "cause",
]  # fmt: skip
BAG_TOPICS = [
    "/joint_states", "/joint_command", "/brain_state", "/connect4/state", "/connect4/board",
    "/connect4/grid_pose", "/sim/ground_truth/connect4_board",
]  # fmt: skip
END = ("GAME_OVER", "FIX_BOARD", "ERROR")
STILL_RAD = (
    0.05  # an arm joint that moved more while the person played: the arm moved (a drop turns it by 1 rad)
)
WINNERS = {"human": game.HUMAN, "robot": game.ROBOT, "draw": game.EMPTY}


def outcome(state, why, truth, moved_rad) -> tuple[bool, str]:
    """(success, cause) of a game: its last /connect4/state, why the bench ended it ("" when the game node
    did), the true cells, and how far an arm joint moved during the person's turns."""
    label = state["state"]
    if why:
        return False, why
    if label != "GAME_OVER":
        return False, f"{'fix_board' if label == 'FIX_BOARD' else 'error'}:{state['reason']}"
    if moved_rad > STILL_RAD:
        return False, "arm_moved"
    if state["lost"]:
        return False, "token_lost"
    if state["missed"]:
        return False, "wrong_column"
    if list(truth) != state["board"]:
        return False, "wrong_board"
    full = game.EMPTY not in truth
    if game.winner(truth) != WINNERS[state["result"]] or (state["result"] == "draw" and not full):
        return False, "wrong_winner"
    return True, ""


def summary(rows) -> str:
    """Games won by the bench's count, p50/p95 duration of the successful ones, causes. It starts with
    `games`: tools/ci/bench_report.sh reads the pick & place line by `success`."""
    done = [r["duration_s"] for r in rows if r["success"]]
    rate = f"games {len(done)}/{len(rows)} ({100 * len(done) / len(rows):.0f} %)"
    duration = (
        f"duration p50 {np.percentile(done, 50):.1f} s, p95 {np.percentile(done, 95):.1f} s"
        if done
        else "duration: no success"
    )
    causes = Counter(r["cause"] for r in rows if not r["success"])
    return f"{rate}; {duration}; causes: {', '.join(f'{c}: {n}' for c, n in causes.most_common()) or 'none'}"


def choose(human, state, rng) -> int:
    """The scripted person's column: a random legal one, or the engine's at its medium level."""
    if human == "random":
        return rng.choice(state["legal"])
    me, mask = game.from_cells(state["board"], game.HUMAN)
    return game.best_move(me, mask, None, game.LEVELS["medium"], rng.randrange(1 << 16)).column


def play(client, choose_column, robot_first, timeout_s) -> tuple[dict, str]:
    """One game through the client: (its last state, "" or why the bench ended it). A service that does not
    reply fails the game, not the bench: the next game is played."""
    try:
        return _play(client, choose_column, robot_first, timeout_s)
    except RuntimeError as error:
        return client.state, f"bench:{error}"


def _play(client, choose_column, robot_first, timeout_s):
    before = client.state
    accepted, message = client.new_game(robot_first)
    if not accepted:
        return client.state, f"new_game:{message}"
    end, played = time.monotonic() + timeout_s, before

    def to_act(state):  # a state of the new game that ends it, or a turn not played yet
        waits = state["state"] == "WAIT_HUMAN" and state is not played
        return state is not before and (state["state"] in END or waits)

    while client.wait(to_act, end - time.monotonic()):
        state = played = client.state
        if state["state"] in END:
            return state, ""
        done, message = client.play(choose_column(state) if state["turn"] == "human" else None)
        if not done:
            client.stop()
            return client.state, f"hand:{message}"
    client.stop()  # the arm is free for the next game
    return client.state, "bench_timeout"


class Still:
    """How far the arm went while the person played (target P2-05): `moved`, the furthest an arm joint was
    from its place at the start of a WAIT_HUMAN, rad."""

    def __init__(self):
        self._rest, self.moved = None, 0.0

    def reset(self):
        self.moved = 0.0

    def update(self, waiting, arm):
        """`arm`: the arm joints of a /joint_states; `waiting`: the game is in WAIT_HUMAN."""
        if not waiting:
            self._rest = None
            return
        arm = np.asarray(arm)
        if self._rest is None:
            self._rest = arm
        self.moved = max(self.moved, float(np.abs(arm - self._rest).max()))


def row(number, seed, human, robot_first, state, why, truth, moved_rad, duration_s) -> dict:
    """A game's CSV row, FIELDS in order."""
    success, cause = outcome(state, why, truth, moved_rad)
    return {
        "game": number,
        "seed": seed,
        "human": human,
        "first": "robot" if robot_first else "human",
        "final_state": state["state"],
        "result": state["result"],
        "plies": state["moves"],
        "missed": state["missed"],
        "lost": state["lost"],
        "duration_s": round(duration_s, 1),
        "success": success,
        "cause": cause,
    }


class GameBench(evaluate.Bench):
    """The game bench's ROS side: the client and the simulated hand, the true grid, the arm's joints."""

    def __init__(self, node, human):
        self.node, self.human = node, human
        self.client = GameClient(node, SimHand(node))
        self.truth, self.still = None, Still()  # the last true cells; the arm during the person's turns
        node.create_subscription(Connect4Board, "/sim/ground_truth/connect4_board", self._on_truth, 10)
        node.create_subscription(JointState, "/joint_states", self._on_joints, 10)

    def _on_truth(self, msg):
        self.truth = [int(v) for v in msg.cells]

    def _on_joints(self, msg):
        state = self.client.state
        arm = [q for name, q in zip(msg.name, msg.position, strict=True) if name != "gripper"]
        self.still.update(state is not None and state["state"] == "WAIT_HUMAN", arm)

    def missing(self, timeout_s):
        """What a game cannot start without, as a message; None when all is there."""
        if not self.spin_until(lambda: self.truth is not None, timeout_s):
            return "evaluate: no /sim/ground_truth/connect4_board: game mode, expose_ground_truth:=true"
        if not self.client.wait(lambda state: state["ready"], timeout_s):
            return f"evaluate: the game node is not ready: {self.client.state}"
        return None

    def game(self, number, seed, timeout_s) -> dict:
        rng = random.Random(1000 * seed + number)
        robot_first, start = number % 2 == 0, time.monotonic()
        self.still.reset()
        state, why = play(self.client, lambda s: choose(self.human, s, rng), robot_first, timeout_s)
        duration = time.monotonic() - start
        self.spin_until(lambda: time.monotonic() > start + duration + 0.3, 1.0)  # the truth after the game
        moved = self.still.moved
        return row(number, seed, self.human, robot_first, state, why, self.truth, moved, duration)


def run(args, bench) -> int:
    seed = bench.driver_seed(evaluate.STARTUP_S)
    if args.seed is not None and args.seed != seed:
        print(f"evaluate: the driver runs seed {seed}, not {args.seed}", file=sys.stderr)
        return 2
    if missing := bench.missing(evaluate.STARTUP_S):
        print(missing, file=sys.stderr)
        return 2
    return evaluate.loop(args, bench, "game", args.games, seed, FIELDS, BAG_TOPICS, summary)


def start(args, node) -> int:
    return run(args, GameBench(node, args.human))
