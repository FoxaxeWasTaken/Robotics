"""play.py - the console of `tools/so101 play` (docs/project/phase2/P2-02 §6): a person plays against the
robot from a terminal.

It draws /connect4/state, each side as the letter of its tokens' color (layout.colors), and turns keys into
calls of client.GameClient. No rule here: whose turn it is and
the legal columns are read in the state. The person's hand comes from the caller (client.py): this package
names no sim service, so101_bringup's connect4_play gives the simulated one.

The engine is used twice, in this process and never through the game node: the hint for the person, a search
of HINT_DEPTH (34 ms [MES-66]), and the cells of the winning four.
"""

import select
import sys
import termios
import tty

import rclpy

from so101_connect4 import game as engine
from so101_connect4 import layout
from so101_connect4.client import GameClient, last_move

KEYS = (
    "keys: 1 to 7 play a column, ? a hint, n new game, r new game with the robot first, "
    "e, m, h the level (easy, medium, hard), s stop, q quit"
)
LEVEL_KEYS = {name[0]: depth for name, depth in engine.LEVELS.items()}  # e, m, h
HINT_DEPTH = 8
LETTERS = {"red": "R", "yellow": "Y"}  # a token on the screen, by its color
ANSI = {"red": "\x1b[31mR\x1b[0m", "yellow": "\x1b[33mY\x1b[0m"}
RESULTS = {"robot": "the robot wins", "human": "you win", "draw": "a draw"}


def status(state, mine="R") -> str:
    """What to do; `mine`: the letter of the person's tokens."""
    kind, reason = state["state"], state["reason"]
    if kind == "WAIT_HUMAN" and state["turn"] == "human":
        return f"your turn ({mine}): 1 to 7"
    if kind == "WAIT_HUMAN":
        does = "a token is given to it"
        if state.get("token_source") == "dispenser":
            does = {"dispenser_empty": "the dispenser is filled", "": "it takes a token"}.get(
                reason, "it takes a token again"
            )
        return f"the robot's turn{f' ({reason})' if reason else ''}: {does}"
    if kind == "GAME_OVER":
        return f"{RESULTS[state['result']]}; n: new game, r: the robot first"
    if kind == "IDLE":
        return "n: new game, r: the robot first" if state["ready"] else f"waiting: {state['not_ready']}"
    waits = {"MOVE": "the robot plays", "THINK": "the robot thinks", "READ_BOARD": "the grid is read"}
    return waits.get(kind) or f"{kind}: {reason}; n: new game"


def render(state, color=False, colors=None) -> str:
    """The grid as text, top row first, its columns numbered as their keys; the last move, what to do.
    `colors`: layout.colors, each side's token color; the installed layout's when not given."""
    colors = colors or layout.colors(layout.load())
    plain = {0: "."} | {side: LETTERS[name] for side, name in colors.items()}
    tokens = plain | {side: ANSI[name] for side, name in colors.items()} if color else plain
    cells = [tokens[cell] for cell in state["board"]]
    for i in engine.four(state["board"]):
        cells[i] = cells[i].replace(plain[state["board"][i]], "*")  # the winning four, in its color
    rows = ["| " + " ".join(cells[row * 7 : row * 7 + 7]) + " |" for row in range(5, -1, -1)]
    depth = state["max_depth"]
    level = next((name for name, d in engine.LEVELS.items() if d == depth), f"depth {depth}")
    told = " | ".join(filter(None, [f"level: {level}", last_move(state["last"])]))
    return "\n".join(["  1 2 3 4 5 6 7", *rows, "+---------------+", told, status(state, plain[1])])


def show(text):
    print(text, flush=True)  # through a pipe stdout is block-buffered: the drawing would wait


class Console:
    """One pass at a time, nothing waits: step() draws the state when it changed, gives the robot its token
    when the turn is its own, and acts on a key."""

    def __init__(self, game, write=show, color=False, colors=None):
        self.game, self.write, self.color = game, write, color
        self.colors = colors or layout.colors(layout.load())  # each side's token color (token.robot_color)
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
            self.write(render(state, self.color, self.colors))
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
        elif key in LEVEL_KEYS:
            self._say(self.game.set_depth(LEVEL_KEYS[key]))
        elif key == "?" or key in tuple("1234567"):
            if not (mine and state["turn"] == "human"):  # a token dropped now would end in FIX_BOARD
                self.write(f"not your turn: {state['state']}")
            else:
                self._turn(state, key)
        elif key and not key.isspace():
            self.write(KEYS)

    def _turn(self, state, key):
        """The person's turn: a hint, or their token in the column of `key`."""
        if key == "?":
            position = engine.from_cells(state["board"], engine.HUMAN)
            self.write(f"hint: column {engine.best_move(*position, max_depth=HINT_DEPTH).column + 1}")
        elif int(key) - 1 not in state["legal"]:
            self.write(f"column {key} is full")
        else:
            self._played = state
            self._say(self.game.play(int(key) - 1))

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
