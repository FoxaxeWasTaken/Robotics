"""client.py - a game seen from outside (docs/project/phase2/P2-02 §6): /connect4/state and the game node's
services, shared by the console (play.py) and the bench, so the two cannot drift.

No rule: what is legal and whose turn it is come from the state. No sim service either: the person's hand
is an object the caller gives, with drop_token(column), give_token() and reset_board(), each returning
(success, message). In sim it is so101_bringup's sim_hand.
"""

import json
import time

import rclpy
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def last_move(last) -> str:
    """A state's `last` in words, shared by the console and the debug image: who played which column, as
    its key numbers it; the robot's: the depth its engine reached and, once the game is proven, how many
    moves the winner still has to play (wins_in and loses_in are plies, the robot's own move included). A
    token that fell in another column than the one meant says so: the engine's reason was for the other."""
    if not last:
        return ""
    text = f"{last['who']}: column {last['column'] + 1}"
    if last["planned"] != last["column"]:
        return f"{text} (it meant column {last['planned'] + 1})"
    if "depth" not in last:
        return text
    end = next((end for end in ("loses", "wins") if f"{end}_in" in last), "")
    if not end:
        return f"{text} (depth {last['depth']})"
    moves = last[f"{end}_in"] // 2  # the winner's moves after this one
    left = {0: "", 1: " in 1 move"}.get(moves, f" in {moves} moves")
    return f"{text} (depth {last['depth']}, it {end}{left})"


def answer(spin_once, client, request=None, timeout_s=10.0):
    """A service's reply; RuntimeError when it is not served or does not reply."""
    if not client.wait_for_service(timeout_sec=timeout_s):
        raise RuntimeError(f"{client.srv_name}: no such service")
    future, end = client.call_async(request or client.srv_type.Request()), time.monotonic() + timeout_s
    while not future.done():
        if time.monotonic() > end:
            raise RuntimeError(f"{client.srv_name}: no answer in {timeout_s:g} s")
        spin_once()
    return future.result()


def call(spin_once, client, request=None, timeout_s=10.0) -> tuple:
    """(success, message) of a service's reply; RuntimeError as answer()."""
    reply = answer(spin_once, client, request, timeout_s)
    return reply.success, reply.message


class GameClient:
    def __init__(self, node, hand, spin_once=None):
        self.hand, self.state = hand, None  # the last /connect4/state, a dict
        self.spin_once = spin_once or (lambda: rclpy.spin_once(node, timeout_sec=0.05))
        node.create_subscription(String, "/connect4/state", self._on_state, LATCHED)
        self._new_game = node.create_client(SetBool, "/connect4/new_game")
        self._human_done = node.create_client(Trigger, "/connect4/human_done")
        self._stop = node.create_client(Trigger, "/connect4/stop")
        self._set = node.create_client(SetParameters, "/game_node/set_parameters")

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

    def set_depth(self, depth) -> tuple:
        """The level: the game node's engine.max_depth (game.LEVELS), read by the engine at its next move."""
        depth = Parameter("engine.max_depth", value=depth).to_parameter_msg()
        (result,) = answer(self.spin_once, self._set, SetParameters.Request(parameters=[depth])).results
        return result.successful, result.reason
