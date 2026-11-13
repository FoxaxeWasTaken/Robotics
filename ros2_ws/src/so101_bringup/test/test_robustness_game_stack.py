"""The game's robustness demo on a fake stack in the same process (#243, #252), as test_robustness_demo.py
does for the pick & place: robustness_game.main with its real client, simulated hand and arm watch, against a
node that serves the game's and the sim's services and publishes /connect4/state and /joint_states. Case g4
needs no move: a drop out of reach is refused, a column is filled, one more token is refused."""

import json
import threading
import time

import pytest
import rclpy
import rclpy.node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from so101_bringup import robustness_game
from so101_interfaces.srv import Drop, DropToken

ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


class FakeGameStack:
    """`ready`: what /connect4/state says before a game; `starts`: a new game reaches the person's turn;
    `jump`: rad the arm moves when /brain/drop is called."""

    def __init__(self, ready=True, starts=True, jump=0.0):
        self.starts, self.jump, self.pan, self.tokens, self.dropped = starts, jump, 0.0, 0, []
        self.context = rclpy.Context()
        rclpy.init(context=self.context)
        node = self.node = rclpy.create_node("fake_game", context=self.context)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state = node.create_publisher(String, "/connect4/state", latched)
        self.joints = node.create_publisher(JointState, "/joint_states", 10)
        node.create_service(SetBool, "/connect4/new_game", self._new_game)
        node.create_service(Trigger, "/sim/connect4/reset_board", self._ok)
        node.create_service(DropToken, "/sim/connect4/drop_token", self._drop_token)
        node.create_service(Drop, "/brain/drop", self._drop)
        node.create_timer(0.05, self._tick)
        self.ready, self.said_ready = ready, False
        self.say("IDLE", ready=False)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def _tick(self):
        """The joints at 20 Hz; `ready` once the demo's clients are discovered, as on a stack long up: a
        reply sent before that is lost."""
        self.joints.publish(JointState(name=ARM, position=[self.pan] + [0.0] * 5))
        if self.ready and not self.said_ready:
            try:
                clients = self.node.get_client_names_and_types_by_node("robustness_demo", "/")
            except rclpy.node.NodeNameNonExistentError:
                return
            if {"/connect4/new_game", "/brain/drop", "/sim/connect4/drop_token"} <= {
                name for name, _ in clients
            }:
                self.said_ready = True
                self.say("IDLE")

    def say(self, state, **fields):
        base = {"ready": True, "state": state, "reason": "", "turn": "human", "moves": 0}
        self.state.publish(String(data=json.dumps(base | fields)))

    def _ok(self, request, response):
        response.success = True
        return response

    def _new_game(self, request, response):
        if self.starts:
            self.say("WAIT_HUMAN")
        return self._ok(request, response)

    def _drop_token(self, request, response):
        self.dropped.append(request.column)
        response.success = len(self.dropped) <= 6  # then the column is full
        response.message = "" if response.success else "column full"
        return response

    def _drop(self, request, response):
        self.pan += self.jump
        response.success, response.message = False, "unreachable"
        return response

    def close(self):
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)
        self.thread.join(timeout=5.0)


@pytest.fixture
def demo_node():
    rclpy.init()
    node = rclpy.create_node("robustness_demo")
    yield node
    node.destroy_node()
    rclpy.shutdown()


def run(case, demo_node, **stack):
    fake = FakeGameStack(**stack)
    try:
        return robustness_game.main(demo_node, case, lambda marker: [], time.monotonic()), fake
    finally:
        fake.close()


def test_case_g4_is_handled_a_drop_out_of_reach_refused_then_a_full_column(demo_node, capsys):
    verdict, fake = run("g4", demo_node)
    assert verdict == ("handled", "")
    assert fake.dropped == [robustness_game.FULL] * 7  # six fill the column, the seventh is refused
    said = capsys.readouterr().out
    for line in ("game: WAIT_HUMAN", ">>> drop_out_of_reach", "refused: unreachable", "refused: column full"):
        assert line in said


def test_an_arm_that_moves_on_a_refused_drop_is_not_handled(demo_node):
    verdict, _ = run("g4", demo_node, jump=0.2)
    assert verdict[0] == "not handled"
    assert "drop_out_of_reach: the arm moved" in verdict[1]


def test_a_game_node_that_is_not_ready_is_not_handled(demo_node, monkeypatch):
    monkeypatch.setattr(robustness_game, "STARTUP_S", 1.0)
    verdict, _ = run("g4", demo_node, ready=False)
    assert verdict == ("not handled", "the game node is not ready")


def test_a_new_game_that_does_not_reach_the_persons_turn_is_not_handled(demo_node, monkeypatch):
    monkeypatch.setattr(robustness_game, "STEP_TIMEOUT_S", 1.0)
    verdict, _ = run("g4", demo_node, starts=False)
    assert verdict == ("not handled", "no new game: IDLE")
