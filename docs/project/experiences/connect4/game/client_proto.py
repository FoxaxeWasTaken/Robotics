"""The shared client (console and bench) for ticket #87: /connect4/state plus the services, nothing else, no rule.

GameClient is what so101_connect4/client.py would hold; render() what `tools/so101 play` draws. main() plays N games
with a random human, as the bench would. Usage: client_proto.py GAMES
"""

import json
import random
import sys
import time

import rclpy
from lifecycle_msgs.srv import ChangeState
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def render(state) -> str:
    """The grid as text, top row first; columns numbered for the keys."""
    glyph = {0: ".", 1: "R", 2: "Y"}
    rows = [
        "| " + " ".join(glyph[state["board"][r * 7 + c]] for c in range(7)) + " |" for r in range(5, -1, -1)
    ]
    last = state["last"]
    moved = f" | last: {last['who']} column {last['column'] + 1}" if last else ""
    return "\n".join(
        ["  1 2 3 4 5 6 7", *rows, "+---------------+", f"{state['state']} {state['reason']} | turn: {state['turn']}{moved}"]
    )


class GameClient:
    def __init__(self, node):
        self.node, self.state, self.seen = node, None, []
        node.create_subscription(String, "/connect4/state", self._on_state, LATCHED)
        self.new_game_client = node.create_client(SetBool, "/connect4/new_game")
        self.human_done_client = node.create_client(Trigger, "/connect4/human_done")
        self.stop_client = node.create_client(Trigger, "/connect4/stop")
        self.drop_client = node.create_client(ChangeState, "/sim/connect4/drop_token")
        self.give_client = node.create_client(Trigger, "/sim/connect4/give_token")
        self.reset_client = node.create_client(Trigger, "/sim/connect4/reset")

    def _on_state(self, msg):
        self.state = json.loads(msg.data)
        self.seen.append(self.state["state"])

    def spin_until(self, condition, timeout_s):
        end = time.monotonic() + timeout_s
        while not condition() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return condition()

    def call(self, client, request, timeout_s=30.0):
        if not client.wait_for_service(timeout_sec=timeout_s):
            raise RuntimeError(f"{client.srv_name}: no such service")
        future = client.call_async(request)
        if not self.spin_until(future.done, timeout_s):
            raise RuntimeError(f"{client.srv_name}: no answer")
        return future.result()

    def new_game(self, robot_first=False):
        """Clears the grid and starts: the arm comes home, the board is read, then WAIT_HUMAN."""
        self.call(self.reset_client, Trigger.Request())
        return self.call(self.new_game_client, SetBool.Request(data=robot_first))

    def human_turn(self, column=None):
        """One key: the human's token (none when the robot is to play), the robot's token, "done"."""
        if column is not None:
            request = ChangeState.Request()
            request.transition.id = column
            self.call(self.drop_client, request)
        self.call(self.give_client, Trigger.Request())
        return self.call(self.human_done_client, Trigger.Request())


def main():
    rclpy.init()
    node = rclpy.create_node("client")
    client = GameClient(node)
    assert client.spin_until(lambda: client.state is not None, 30.0), "no /connect4/state"
    start = time.monotonic()
    print(f"first state: ready={client.state['ready']}, state {client.state['state']}", flush=True)
    refused = client.call(client.new_game_client, SetBool.Request(data=False))
    print(f"new_game before the first pose: success={refused.success}, {refused.message!r}", flush=True)
    assert client.spin_until(lambda: client.state["ready"], 30.0)
    print(f"ready after {time.monotonic() - start:.1f} s", flush=True)
    for number in range(int(sys.argv[1])):
        rng = random.Random(number)
        begin = time.monotonic()
        reply = client.new_game(robot_first=number % 2 == 1)
        assert reply.success, reply.message
        client.spin_until(lambda: client.state["state"] == "MOVE" or client.state["state"] == "READ_BOARD", 5.0)
        while True:
            assert client.spin_until(
                lambda: client.state["state"] in ("WAIT_HUMAN", "GAME_OVER", "FIX_BOARD", "ERROR"), 60.0
            ), client.state
            state = client.state
            if state["state"] != "WAIT_HUMAN":
                break
            column = rng.choice(state["legal"]) if state["turn"] == "human" else None
            assert client.human_turn(column).success
            client.spin_until(lambda: client.state["state"] != "WAIT_HUMAN", 5.0)
        print(render(state), flush=True)
        print(f"game {number + 1}: {state['state']} {state['result']}, {state['moves']} tokens, "
              f"{time.monotonic() - begin:.1f} s", flush=True)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
