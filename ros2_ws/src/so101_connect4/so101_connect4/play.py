"""play.py - the console of `tools/so101 play` (docs/project/phase2/P2-02 §6): a person plays against the
robot from a terminal.

It draws /connect4/state, each side as the letter of its tokens' color (layout.colors), and turns keys into
calls of client.GameClient. No rule here: whose turn it is and
the legal columns are read in the state. The person's hand comes from the caller (client.py): this package
names no sim service, so101_bringup's connect4_play gives the simulated one.

The engine is used twice, in this process and never through the game node: the hint for the person, a search
of HINT_DEPTH (34 ms [MES-66]), and the cells of the winning four.
"""

import os
import select
import sys
import termios
import tty

import rclpy
from rclpy.executors import ExternalShutdownException

from so101_connect4 import game as engine
from so101_connect4 import layout
from so101_connect4.client import GameClient, last_move

KEYS = (
    "keys: 1 to 7 play a column, ? a hint, a the computer plays your turns, n new game, "
    "r new game with the robot first, e, m, h the level (easy, medium, hard), s stop, q quit"
)
NO_STATE = "no /connect4/state yet: is the game's stack up, and built? (CONNECT4=true tools/so101 up)"
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


class Screen:
    """A terminal redrawn in place: the keys, the last board, the last message under it. Printed one after
    the other, a turn scrolled fifty lines."""

    def __init__(self, out=show):
        self._out, self._board, self._message = out, "", ""

    def draw(self, board):
        self._board = board
        self._paint()

    def write(self, message):
        self._message = message
        self._paint()

    def _paint(self):  # the cursor home, the screen cleared from there, then the whole screen
        self._out("\x1b[H\x1b[J" + "\n".join(part for part in (KEYS, self._board, self._message) if part))


class Console:
    """One pass at a time, nothing waits: step() draws the state when it changed, gives the robot its token
    when the turn is its own, and acts on a key."""

    def __init__(self, game, write=show, color=False, colors=None, draw=None):
        """`write` gets the messages and `draw` the boards; the same by default."""
        self.game, self.write, self.color = game, write, color
        self.draw = draw or write
        self.auto = False  # the automatic player plays the person's turns
        self.colors = colors or layout.colors(layout.load())  # each side's token color (token.robot_color)
        # The states drawn and played, and the one whose robot's turn failed: the client's state is a new
        # dict each time
        self._drawn = self._played = self._failed = None

    def step(self, key=None) -> bool:
        """False to quit."""
        state, key = self.game.state, key.lower() if key else key
        if key == "q":
            return False
        if state is None:
            if key and not key.isspace():
                self.write(NO_STATE)
            return True
        if state is not self._drawn:
            self._drawn = state
            self.draw(render(state, self.color, self.colors))
        try:
            self._act(state, key)
        except RuntimeError as error:  # a service that is gone
            self.write(str(error))
        return True

    def _act(self, state, key):
        mine = state["state"] == "WAIT_HUMAN" and state is not self._played
        if mine and state["turn"] == "robot":  # it starts, or its token was lost: no key for that
            mine = False
            if self._robot_turn(state, key):
                return
        mine = mine and state["turn"] == "human"
        if not self._automatic(state, key, mine):
            self._on_key(state, key, mine)

    def _robot_turn(self, state, key) -> bool:
        """The robot's turn asked for, whether it was. A try that failed waits for a key, any but the
        game's own: a service that is gone takes 10 s."""
        if state is self._failed and not (key and key not in ("n", "r", "s", *LEVEL_KEYS)):
            return False
        try:
            done, message = self.game.play()
        except RuntimeError as error:
            done, message = False, str(error)
        self._played, self._failed = (state, None) if done else (None, state)
        if not done:
            self.write(f"{message}; any key to try again")
        return True

    def _automatic(self, state, key, mine) -> bool:
        """The automatic player: switched by a key, then its move when the turn is the person's (`mine`).
        Whether this pass was its own."""
        if key and not key.isspace() and self.auto:  # any key stops it, and is not taken for a move
            self.auto = False
            self.write("automatic player: off")
            return True
        if key == "a":
            self.auto = True
            self.write("automatic player: on, any key stops it")
        if not self.auto:
            return False
        if mine:  # a whole game to watch: the engine plays the person's turns, at the hint's depth
            position = engine.from_cells(state["board"], engine.HUMAN)
            column = engine.best_move(*position, max_depth=HINT_DEPTH).column
            if not self._turn(state, str(column + 1)):
                self.auto = False  # refused: no try at each pass
        return True

    def _on_key(self, state, key, mine):
        """A key of the game; `mine`: the turn is the person's."""
        if key in ("n", "r"):
            self._say(self.game.new_game(robot_first=key == "r"))
        elif key == "s":
            self._say(self.game.stop())
        elif key in LEVEL_KEYS:
            self._say(self.game.set_depth(LEVEL_KEYS[key]))
        elif key == "?" or key in tuple("1234567"):
            if not mine:  # a token dropped now would end in FIX_BOARD
                self.write(f"not your turn: {state['state']}")
            else:
                self._turn(state, key)
        elif key and not key.isspace():
            self.write(KEYS)

    def _turn(self, state, key) -> bool:
        """The person's turn: a hint, or their token in the column of `key`; whether a token was played."""
        if key == "?":
            position = engine.from_cells(state["board"], engine.HUMAN)
            self.write(f"hint: column {engine.best_move(*position, max_depth=HINT_DEPTH).column + 1}")
        elif int(key) - 1 not in state["legal"]:
            self.write(f"column {key} is full")
        else:
            reply = self.game.play(int(key) - 1)
            if reply[0]:  # refused, the state stays as it is: the key must work again
                self._played = state
            self._say(reply)
            return reply[0]
        return False

    def _say(self, reply):
        if not reply[0]:
            self.write(reply[1])


def read_key():
    """A key typed since the last call, else None; "q" at the end of the input. Read from the descriptor, a
    byte at a time: Python's buffered input took every pending byte and gave one, so a key typed ahead
    stayed there, unseen by select, and came out at the next key press. A special key is nothing: its
    sequence holds digits (Delete is ESC [ 3 ~), which are columns."""
    stdin = sys.stdin.fileno()
    if not select.select([stdin], [], [], 0)[0]:
        return None
    byte = os.read(stdin, 1)
    if not byte:
        return "q"
    if byte == b"\x1b":
        while select.select([stdin], [], [], 0)[0]:
            if not os.read(stdin, 32):
                break
        return None
    return byte.decode(errors="ignore") or None


def main(make_hand, args=None):
    """The console, with make_hand(node) as the person's hand."""
    rclpy.init(args=args)
    node = rclpy.create_node("connect4_play")
    game = GameClient(node, make_hand(node))
    if sys.stdout.isatty():  # a terminal: the screen redrawn in place; a pipe: one line after the other
        screen = Screen()
        console = Console(game, screen.write, color=True, draw=screen.draw)
        screen.write("")
    else:
        console = Console(game)
        show(KEYS)
    saved = termios.tcgetattr(sys.stdin) if sys.stdin.isatty() else None
    try:
        if saved:
            tty.setcbreak(sys.stdin)  # a key at a time, no Enter
        while console.step(read_key()):
            rclpy.spin_once(node, timeout_sec=0.05)
    except (KeyboardInterrupt, ExternalShutdownException):  # Ctrl-C: rclpy's own handler may shut down first
        pass
    finally:
        if saved:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, saved)
        node.destroy_node()
        rclpy.try_shutdown()
