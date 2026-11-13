"""robustness_game - the robustness demo's cases on a Connect Four game (#243, docs/project/06 §10), run by
robustness_demo for the cases g1 to g6:

    ros2 launch so101_bringup robustness_demo.launch.py case:=<g1..g6> [lighting:=dim]

A case (so101_bringup.robustness.GAME_CASES) is a list of steps on a game that waits for the person's first
move: an action, then the /connect4/state or the reply that must follow, and whether the arm may move
meanwhile. The faults go through the simulated person's hand, the game's and the brain's services, or the
demo's own board reader, stopped and started again; never through the ground truth (C-06).
"""

import random
import signal
import subprocess
import time

from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState

from so101_bringup import games
from so101_bringup.robustness import GAME_CASES, NOT_HANDLED, WAITS_PERSON, game_verdict, matches
from so101_bringup.sim_hand import SimHand
from so101_connect4.client import GameClient, call
from so101_interfaces.msg import Connect4Board
from so101_interfaces.srv import Drop

READER = b"lib/so101_connect4/board_reader"  # in the command line of the demo stack's board reader
OUT_OF_REACH = (0.6, 0.0, 0.3)  # m, base_link: no drop pose reaches it
COLUMN, FULL = 3, 0  # the person's column; the column filled for g4
STARTUP_S = 60.0
STEP_TIMEOUT_S = 120.0  # a move lasts 19 s at most [MES-69]
GAME_TIMEOUT_S = 600.0


def label(state) -> str:
    return f"{state['state']}:{state['reason']}" if state["reason"] else state["state"]


def run(case, demo) -> list:
    """Each step of the case on the demo, up to the first one that is not reached. For game_verdict:
    `reached`, `got` (what came) and `moved` (rad, the arm since the step's action)."""
    results = []
    for step in case.steps:
        before = demo.state
        demo.rest()
        reply = demo.act(step.action)
        if step.state is None:
            reached, got = matches(step.reply, reply), str(reply)
        else:  # a state published after the action: the one before it may already look the same

            def follows(state, before=before, expected=step.state):
                return state is not before and matches(expected, state)

            reached = demo.wait(follows, STEP_TIMEOUT_S)
            got = label(demo.state)
        results.append({"reached": reached, "got": got, "moved": demo.moved()})
        if not reached:
            break
    return results


class Demo:
    """The demo's ROS side: the game's client with the simulated hand, the arm's joints, the board reader."""

    def __init__(self, node, stop_nodes, t0):
        self.node, self._stop_nodes, self.t0 = node, stop_nodes, t0  # t0: the clock of the demo's lines
        self.client = GameClient(node, SimHand(node))
        self.hand, self.wait = self.client.hand, self.client.wait
        self.still, self.boards, self.reader, self.rng = games.Still(), 0, None, random.Random(0)
        self._said, self._spin_once = None, self.client.spin_once
        self.client.spin_once = self._spin_and_say
        node.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        node.create_subscription(Connect4Board, "/connect4/board", self._on_board, QoSProfile(depth=1))
        self._drop = node.create_client(Drop, "/brain/drop")
        self._actions = {
            "cheat": lambda: self._turn(COLUMN, COLUMN + 1),
            "turn": lambda: self.client.play(COLUMN),
            "turn_without_token": lambda: self._turn(COLUMN, give=False),
            "give_token": self.client.play,
            "human_done": self.client.human_done,
            "new_game": self.client.new_game,
            "stop": self.client.stop,
            "stop_reader": self._stop_reader,
            "start_reader": self._start_reader,
            "drop_out_of_reach": self._drop_out_of_reach,
            "fill_column": self._fill_column,
            "drop_token": lambda: self.hand.drop_token(FULL),
            "game": self._game,
            "wait_5_s": lambda: self._spin_for(5.0),
            "wait_2_s": lambda: self._spin_for(2.0),
        }

    @property
    def state(self):
        return self.client.state

    def say(self, text):
        print(f"{time.monotonic() - self.t0:7.2f} s  {text}", flush=True)

    def _on_joints(self, msg):
        self.still.update(
            True, [q for name, q in zip(msg.name, msg.position, strict=True) if name != "gripper"]
        )

    def _on_board(self, msg):
        self.boards += 1

    def _spin_and_say(self):
        """The client's spin, then each new state of the game once, with its time."""
        self._spin_once()
        if self.state is not None and label(self.state) != self._said:
            self._said = label(self.state)
            self.say(f"game: {self._said}")

    def rest(self):
        self.still = games.Still()  # the arm's place now: what moved() is measured from

    def moved(self) -> float:
        return self.still.moved

    def act(self, action) -> dict:
        self.say(f">>> {action}")
        success, message = self._actions[action]()
        self.say(f"    {'done' if success else 'refused'}{f': {message}' if message else ''}")
        return {"success": success, "message": message}

    def _spin_for(self, seconds):
        self.wait(lambda state: False, seconds)
        return True, ""

    def _turn(self, *columns, give=True):
        """The person's tokens, the robot's or not, then human_done."""
        if give:
            self.hand.give_token()
        for column in columns:
            self.hand.drop_token(column)
        return self.client.human_done()

    def _stop_reader(self):
        stopped = self._stop_nodes(READER)
        return bool(stopped), f"board reader stopped: pid {', '.join(map(str, stopped)) or 'none found'}"

    def _start_reader(self):
        seen = self.boards
        self.reader = subprocess.Popen(
            ["ros2", "launch", "so101_connect4", "board_reader.launch.py"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        self.wait(lambda state: self.boards > seen, STARTUP_S)
        return self.boards > seen, "the board reader publishes again"

    def _drop_out_of_reach(self):
        request = Drop.Request()
        request.target.x, request.target.y, request.target.z = OUT_OF_REACH
        return call(self.client.spin_once, self._drop, request)

    def _fill_column(self):
        for _ in range(games.game.ROWS):
            reply = self.hand.drop_token(FULL)
            self._spin_for(1.0)  # the token falls before the next one
        return reply

    def _game(self):
        state, why = games.play(self.client, lambda s: self.rng.choice(s["legal"]), False, GAME_TIMEOUT_S)
        return not why, why or f"{state['moves']} tokens, {state['result']} wins"

    def close(self):
        if self.reader is not None:
            self.reader.send_signal(signal.SIGINT)
            self.reader.wait(timeout=15)


def main(node, name, stop_nodes, t0) -> tuple[str, str]:
    """The verdict of one case on the demo's stack: a new game up to the person's turn, then its steps."""
    demo = Demo(node, stop_nodes, t0)
    try:
        if not demo.wait(lambda state: state["ready"], STARTUP_S):
            return NOT_HANDLED, "the game node is not ready"
        before = demo.state
        demo.act("new_game")
        if not demo.wait(lambda s: s is not before and matches(WAITS_PERSON, s), STEP_TIMEOUT_S):
            return NOT_HANDLED, f"no new game: {label(demo.state)}"
        return game_verdict(GAME_CASES[name], run(GAME_CASES[name], demo))
    finally:
        demo.close()
