"""play.py - the console of `tools/so101 play` (docs/project/phase2/P2-02 §6): a person plays against the
robot from a terminal.

It draws /connect4/state, each side as the letter of its tokens' color (layout.colors), and turns keys into
calls of client.GameClient. On a terminal the screen is redrawn in place with the look of the pick & place
console (so101_common.terminal): picture() and Screen; through a pipe, render(), one drawing after the other.
No rule here: whose turn it is and the legal columns are read in the state. The person's hand comes from the
caller (client.py): this package names no sim service, so101_bringup's connect4_play gives the simulated one.

The engine is used twice, in this process and never through the game node: the hint for the person, a search
of HINT_DEPTH (34 ms [MES-66]), and the cells of the winning four.
"""

import sys

import rclpy
from rclpy.executors import ExternalShutdownException

from so101_common import terminal
from so101_common.terminal import (
    Pen,
    read_key,
    show,
)
from so101_connect4 import game as engine
from so101_connect4 import layout
from so101_connect4.client import GameClient, last_move

KEYS = (
    "keys: 1 to 7 play a column, ? a hint, a the computer plays your turns, n new game, "
    "r new game with the robot first, e, m, h the level (easy, medium, hard), s stop, q quit"
)
NO_STATE = "no /connect4/state yet: is the game's stack up, and built? (CONNECT4=true tools/so101 up)"
LEVEL_KEYS = {name[0]: depth for name, depth in engine.LEVELS.items()}  # e, m, h
GAME_KEYS = ("n", "r", "s", *LEVEL_KEYS)  # they act on the game whoever's turn it is
HINT_DEPTH = 8
LETTERS = {"red": "R", "yellow": "Y"}  # a token on the screen, by its color
DISCS = {"red": ("●", "1;31"), "yellow": ("●", "1;33")}  # on a terminal: a disc in its color
TITLE = "Connect Four"
KEY_BAR = [
    ("1-7", "play"),
    ("?", "hint"),
    ("a", "auto"),
    ("n", "new"),
    ("r", "robot first"),
    ("e/m/h", "level"),
    ("s", "stop"),
    ("q", "quit"),
]
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
    if color:  # a terminal
        return picture(state, Pen(pretty=True), colors)
    plain = {0: "."} | {side: LETTERS[name] for side, name in colors.items()}
    cells = [plain[cell] for cell in state["board"]]
    for i in engine.four(state["board"]):
        cells[i] = cells[i].replace(plain[state["board"][i]], "*")  # the winning four, in its color
    rows = ["| " + " ".join(cells[row * 7 : row * 7 + 7]) + " |" for row in range(5, -1, -1)]
    told = " | ".join(filter(None, [f"level: {level(state)}", last_move(state["last"])]))
    return "\n".join(["  1 2 3 4 5 6 7", *rows, "+---------------+", told, status(state, plain[1])])


def level(state) -> str:
    depth = state["max_depth"]
    return next((name for name, d in engine.LEVELS.items() if d == depth), f"depth {depth}")


def disc(pen, colors, side, won=False) -> str:
    """A side's token: a disc in its color on a terminal, highlighted in a winning four; else its letter."""
    glyph, code = DISCS[colors[side]]
    return pen.paint(code + ";7" * won, glyph) if pen.pretty else LETTERS[colors[side]]


def grid(board, pen, colors) -> list:
    """The grid in its box, top row first: 8 lines."""
    four, empty = engine.four(board), "·" if pen.pretty else "."

    def cell(i):
        if not board[i]:
            return f" {empty} "
        return f"[{disc(pen, colors, board[i], True)}]" if i in four else f" {disc(pen, colors, board[i])} "

    side = pen.paint("chrome", pen.glyph("side"))
    edges = ("┌┐", "└┘") if pen.pretty else ("++", "++")
    top, bottom = (pen.paint("chrome", f"  {left}{pen.glyph('rule') * 29}{right}") for left, right in edges)
    rows = [
        f"  {side} " + " ".join(cell(row * 7 + c) for c in range(7)) + f" {side}" for row in range(5, -1, -1)
    ]
    return [top, *rows, bottom]


def mood(state) -> str:
    """The status line's mark: go (the person acts), done, fail, or run (the robot acts, the game waits)."""
    kind = state["state"]
    if kind == "WAIT_HUMAN" and state["turn"] == "human":
        return "go"
    if kind == "GAME_OVER" or (kind == "IDLE" and state["ready"]):
        return "done"
    return "fail" if kind in ("FIX_BOARD", "ERROR") else "run"


def picture(state, pen, colors=None) -> str:
    """The terminal's screen, from its header to its status line: the grid in its box with the game's facts
    beside it, the last move, what to do. Screen puts the last message and the keys under it."""
    colors = colors or layout.colors(layout.load())
    facts = [
        f"level   {level(state)}",
        f"moves   {state['moves']}",
        "",
        f"you     {disc(pen, colors, 1)} {colors[1]}",
        f"robot   {disc(pen, colors, 2)} {colors[2]}",
        "",
        f"drops   {state['lost']} lost, {state['missed']} in a wrong column",  # the robot's
        "",
    ]
    mine = DISCS[colors[1]][0] if pen.pretty else disc(pen, colors, 1)  # no color inside the status line's
    tag = state["state"] + (f":{state['reason']}" if state["reason"] else "")
    header = pen.header(TITLE, tag)
    return "\n".join(
        [
            header if pen.pretty else header.rstrip(),
            pen.rule("game"),
            "     " + "   ".join("1234567"),
            *(
                f"{line}      {fact}".rstrip()
                for line, fact in zip(grid(state["board"], pen, colors), facts, strict=True)
            ),
            pen.rule("last move"),
            "   " + (last_move(state["last"]) or "none yet"),
            pen.rule(),
            pen.status(mood(state), status(state, mine)),
        ]
    )


class Screen:
    """A terminal redrawn in place: the last board, the last message under it, the keys. Printed one after
    the other, a turn scrolled fifty lines."""

    def __init__(self, out=show, pen=None):
        self._out, self._pen, self._message = out, pen or Pen(), ""
        waiting = [self._pen.header(TITLE, "no state").rstrip(), self._pen.rule()]
        self._board = "\n".join([*waiting, self._pen.status("run", "waiting for the game's state")])

    def draw(self, board):
        self._board = board
        self._paint()

    def write(self, message):
        self._message = "" if message == KEYS else message  # the keys are on the screen already
        self._paint()

    def _paint(self):  # the whole screen, over the one before
        message = "   " + self._message if self._message else ""
        self._out(terminal.frame("\n".join([self._board, message, self._pen.keys(KEY_BAR)])))


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
            if self._robot_turn(state, key) and key not in GAME_KEYS:
                return
        mine = mine and state["turn"] == "human"
        if not self._automatic(state, key, mine):
            self._on_key(state, key, mine)

    def _robot_turn(self, state, key) -> bool:
        """The robot's turn asked for, whether it was. A try that failed waits for a key, any but the
        game's own: a service that is gone takes 10 s."""
        if state is self._failed and not (key and key not in GAME_KEYS):
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
            self.auto = False  # a turn refused, or a service that is gone: no try at each pass
            self.auto = self._turn(state, str(column + 1))
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
            try:
                reply = self.game.play(int(key) - 1)
            finally:
                # Refused before the token went in, the state stays as it is: the key must work again. In
                # the grid, whatever followed: never a second token
                if self.game.dropped:
                    self._played = state
            self._say(reply)
            return reply[0]
        return False

    def _say(self, reply):
        if not reply[0]:
            self.write(reply[1])


def main(make_hand, args=None):
    """The console, with make_hand(node) as the person's hand."""
    rclpy.init(args=args)
    node = rclpy.create_node("connect4_play")
    game = GameClient(node, make_hand(node))
    live = sys.stdout.isatty()  # a terminal: the screen redrawn in place; a pipe: one line after the other
    if live:
        pretty = terminal.pretty()
        screen = Screen(pen=Pen(pretty))
        console = Console(game, screen.write, color=pretty, draw=screen.draw)
        screen.write("")
    else:
        console = Console(game)
        show(KEYS)
    try:
        with terminal.session():
            while console.step(read_key()):
                rclpy.spin_once(node, timeout_sec=0.05)
    except (KeyboardInterrupt, ExternalShutdownException):  # Ctrl-C: rclpy's own handler may shut down first
        pass
    finally:
        if live:
            show("")  # the shell's prompt under the last screen, not over it
        node.destroy_node()
        rclpy.try_shutdown()
