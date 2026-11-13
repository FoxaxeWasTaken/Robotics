"""client.py - a game seen from outside (docs/project/phase2/P2-02 §6): /connect4/state and the game node's
services, shared by the console (play.py) and the bench, so the two cannot drift.

No rule: what is legal and whose turn it is come from the state. No sim service either: the person's hand
is an object the caller gives, with drop_token(column), give_token() and reset_board(), each returning
(success, message). In sim it is so101_bringup's sim_hand.
"""

import json
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def call(spin_once, client, request=None, timeout_s=10.0) -> tuple:
    """(success, message) of a service's reply; RuntimeError when it is not served or does not reply."""
    if not client.wait_for_service(timeout_sec=timeout_s):
        raise RuntimeError(f"{client.srv_name}: no such service")
    future, end = client.call_async(request or client.srv_type.Request()), time.monotonic() + timeout_s
    while not future.done():
        if time.monotonic() > end:
            raise RuntimeError(f"{client.srv_name}: no answer in {timeout_s:g} s")
        spin_once()
    return future.result().success, future.result().message


class GameClient:
    def __init__(self, node, hand, spin_once=None):
        self.hand, self.state = hand, None  # the last /connect4/state, a dict
        self.spin_once = spin_once or (lambda: rclpy.spin_once(node, timeout_sec=0.05))
        node.create_subscription(String, "/connect4/state", self._on_state, LATCHED)
        self._new_game = node.create_client(SetBool, "/connect4/new_game")
        self._human_done = node.create_client(Trigger, "/connect4/human_done")
        self._stop = node.create_client(Trigger, "/connect4/stop")

    def _on_state(self, msg):
        self.state = json.loads(msg.data)

    def wait(self, condition, timeout_s) -> bool:
        """Spins until condition(state) holds; False at the timeout."""
        end = time.monotonic() + timeout_s
        while not (met := self.state is not None and condition(self.state)) and time.monotonic() < end:
            self.spin_once()
        return met

    def new_game(self, robot_first=False) -> tuple:
        """A new game, then the grid cleared: the game node waits for the empty board, and takes a board not
        empty yet as "not yet" (turns.py). A refused game leaves the grid as it is."""
        reply = call(self.spin_once, self._new_game, SetBool.Request(data=robot_first))
        return self.hand.reset_board() if reply[0] else reply

    def play(self, column=None) -> tuple:
        """The person's turn: the robot's token between its jaws, the person's own in `column` (None when the
        robot is to play), then human_done. The first step that fails ends it, with its message."""
        steps = [self.hand.give_token]
        if column is not None:
            steps.append(lambda: self.hand.drop_token(column))
        for step in [*steps, self.human_done]:
            reply = step()
            if not reply[0]:
                break
        return reply

    def human_done(self) -> tuple:
        return call(self.spin_once, self._human_done)

    def stop(self) -> tuple:
        return call(self.spin_once, self._stop)
