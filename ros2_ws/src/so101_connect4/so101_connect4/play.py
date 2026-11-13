"""play.py - the console of `tools/so101 play` (docs/project/phase2/P2-02 §6): a person plays against the
robot from a terminal.

It draws /connect4/state and turns keys into calls of client.GameClient. No rule here: whose turn it is and
the legal columns are read in the state. The person's hand comes from the caller (client.py): this package
names no sim service, so101_bringup's connect4_play gives the simulated one.
"""

import select
import sys
import termios
import tty

import rclpy

from so101_connect4.client import GameClient

KEYS = "keys: 1 to 7 play a column, n new game, r new game with the robot first, s stop, q quit"
TOKENS = {0: ".", 1: "R", 2: "Y"}  # the person's red, the robot's yellow
COLORS = {1: "\x1b[31mR\x1b[0m", 2: "\x1b[33mY\x1b[0m"}
RESULTS = {"robot": "the robot wins", "human": "you win", "draw": "a draw"}


def status(state) -> str:
    kind, reason = state["state"], state["reason"]
    if kind == "WAIT_HUMAN" and state["turn"] == "human":
        return "your turn (R): 1 to 7"
    if kind == "WAIT_HUMAN":
        return f"the robot's turn{f' ({reason})' if reason else ''}: a token is given to it"
    if kind == "GAME_OVER":
        return f"{RESULTS[state['result']]}; n: new game, r: the robot first"
    if kind == "IDLE":
        return "n: new game, r: the robot first" if state["ready"] else "waiting for the stack"
    waits = {"MOVE": "the robot plays", "THINK": "the robot thinks", "READ_BOARD": "the grid is read"}
    return waits.get(kind) or f"{kind}: {reason}; n: new game"


def render(state, color=False) -> str:
    """The grid as text, top row first, its columns numbered as their keys; the last move, what to do."""
    tokens = TOKENS | COLORS if color else TOKENS
    rows = [
        "| " + " ".join(tokens[state["board"][row * 7 + column]] for column in range(7)) + " |"
        for row in range(5, -1, -1)
    ]
    last = state["last"]
    moved = f"{last['who']}: column {last['column'] + 1} | " if last else ""
    return "\n".join(["  1 2 3 4 5 6 7", *rows, "+---------------+", moved + status(state)])


def show(text):
    print(text, flush=True)  # through a pipe stdout is block-buffered: the drawing would wait


class Console:
    """One pass at a time, nothing waits: step() draws the state when it changed, gives the robot its token
    when the turn is its own, and acts on a key."""

    def __init__(self, game, write=show, color=False):
        self.game, self.write, self.color = game, write, color
        self._drawn = self._played = (
            None  # the states drawn and played: the client's state is a new dict each time
        )

    def step(self, key=None) -> bool:
        """False to quit."""
        state = self.game.state
        if key == "q":
            return False
        if state is None:
            return True
        if state is not self._drawn:
            self._drawn = state
            self.write(render(state, self.color))
        try:
            self._act(state, key)
        except RuntimeError as error:  # a service that is gone
            self.write(str(error))
        return True

    def _act(self, state, key):
        mine = state["state"] == "WAIT_HUMAN" and state is not self._played
        if mine and state["turn"] == "robot":  # it starts, or its token was lost: no key for that
            self._played, mine = state, False
            self._say(self.game.play())
        if key in ("n", "r"):
            self._say(self.game.new_game(robot_first=key == "r"))
        elif key == "s":
            self._say(self.game.stop())
        elif key in tuple("1234567"):
            if not (mine and state["turn"] == "human"):  # a token dropped now would end in FIX_BOARD
                self.write(f"not your turn: {state['state']}")
            elif int(key) - 1 not in state["legal"]:
                self.write(f"column {key} is full")
            else:
                self._played = state
                self._say(self.game.play(int(key) - 1))
        elif key and not key.isspace():
            self.write(KEYS)

    def _say(self, reply):
        if not reply[0]:
            self.write(reply[1])


def read_key():
    """A key typed since the last call, else None; "q" at the end of the input."""
    if not select.select([sys.stdin], [], [], 0)[0]:
        return None
    return sys.stdin.read(1) or "q"


def main(make_hand, args=None):
    """The console, with make_hand(node) as the person's hand."""
    rclpy.init(args=args)
    node = rclpy.create_node("connect4_play")
    console = Console(GameClient(node, make_hand(node)), color=sys.stdout.isatty())
    show(KEYS)
    saved = termios.tcgetattr(sys.stdin) if sys.stdin.isatty() else None
    try:
        if saved:
            tty.setcbreak(sys.stdin)  # a key at a time, no Enter
        while console.step(read_key()):
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        if saved:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, saved)
        node.destroy_node()
        rclpy.try_shutdown()
