"""The turns of a Connect Four game (phase2/P2-02 §5). Pure, no ROS. Ported from the prototype of
docs/project/experiences/connect4/game/ [MES-66].

game_node calls step(now, inputs) at 10 Hz and does what it returns, as brain_node does with fsm.Fsm: `now` is
the node's steady clock, the engine and the brain run outside. One transition at most per step.

READ_BOARD waits for a board that fits what it expects and only then judges: a stale read (the same board as
before) is not an error, it is "not yet". So a read right after a drop needs no settle delay.

The arm moves only in MOVE (P2-05). MOVE is entered from THINK or from a new game, THINK only from
READ_BOARD, and READ_BOARD only from a new game's MOVE, from a robot's MOVE, or from /connect4/human_done. In
MOVE the node calls /brain/drop (a column) or /brain/handover (a new game: the arm to HANDOVER); both reply at
the end of the move (P2-02 §6), and the reply decides.

| Event | Seen by | Then |
| illegal column, full column | client, `legal` of /connect4/state; driver refuses too | nothing changes |
| human token never seen | READ_BOARD timeout | FIX_BOARD not_seen:human |
| two tokens, wrong color, not at the landing cell | READ_BOARD | FIX_BOARD not_one_*_token |
| board invalid | reader's valid = false | FIX_BOARD <its reason> |
| new game: tokens left, grid not read | READ_BOARD timeout, "not yet" before | FIX_BOARD board_not_empty |
| board never stable | reader silent, READ_BOARD timeout | FIX_BOARD not_seen:* |
| no token in the jaws | /brain/drop reply no_token, arm still | WAIT_HUMAN, turn robot: give a token |
| token lost | READ_BOARD timeout with a recent unchanged read | WAIT_HUMAN, turn robot: give a token |
| token in the wrong column | READ_BOARD | the camera wins, the game goes on, `missed` counts it |
| brain refuses, drop fails | /brain/drop or /brain/handover reply | ERROR |
| timeouts | READ_BOARD 5 s, THINK 5 s, MOVE 90 s (then /brain/stop); WAIT_HUMAN, FIX_BOARD none | as above |
| /connect4/stop; STOPPED or the reply `stopped` in MOVE | any state | ERROR stopped, /brain/stop called |
| human won, robot won, draw | READ_BOARD | GAME_OVER, `result` |
| new game | IDLE, WAIT_HUMAN, FIX_BOARD, GAME_OVER, ERROR; not MOVE, THINK, READ_BOARD | MOVE, read: empty |
| robot plays first | new_game(data=true) | WAIT_HUMAN, turn robot, then the same path as a lost token |

FIX_BOARD is a dead end in sim, no hand can fix the grid: `new_game` leaves it, the bench counts a failure.
"""

from dataclasses import dataclass, field
from enum import StrEnum

from so101_connect4 import game
from so101_connect4.game import CELLS, EMPTY, HUMAN, ROBOT


class State(StrEnum):
    IDLE = "IDLE"
    MOVE = "MOVE"  # /brain/drop, or /brain/handover for a new game
    READ_BOARD = "READ_BOARD"
    WAIT_HUMAN = "WAIT_HUMAN"
    THINK = "THINK"
    FIX_BOARD = "FIX_BOARD"
    GAME_OVER = "GAME_OVER"
    ERROR = "ERROR"


class Expect(StrEnum):
    EMPTY = "empty"  # a new game
    SAME = "same"  # nothing new: the robot plays first, or again after a lost token
    HUMAN = "human"  # one human token
    ROBOT = "robot"  # one robot token


NEW_GAME_FROM = {State.IDLE, State.WAIT_HUMAN, State.FIX_BOARD, State.GAME_OVER, State.ERROR}
HUMAN_DONE_FROM = {State.WAIT_HUMAN, State.FIX_BOARD}
TIMED = {State.MOVE: "move", State.READ_BOARD: "read", State.WAIT_HUMAN: "wait", State.THINK: "think"}
NO_TOKEN = "no_token"  # /brain/drop's refusal when the jaws closed on nothing (P2-02 §6)
STOPPED = "stopped"  # the reply of a move cut by /brain/stop


@dataclass(frozen=True)
class Params:
    read_timeout_s: float = 5.0  # the reader publishes only 5 identical reads: 0.5 s, a token falls in 0.3 s
    board_fresh_s: float = 1.0  # a read this recent proves the token did not land
    think_timeout_s: float = 5.0  # budget 1 s, plus one queued search after a stop
    move_timeout_s: float = 90.0  # a move lasts 12 to 17 s on the sim [MES-73]


@dataclass(frozen=True)
class Board:
    cells: tuple
    valid: bool
    reason: str
    received: float  # s, on the step clock


@dataclass(frozen=True)
class Inputs:
    ready: bool = True  # first /end_effector_pose, /connect4/grid_pose, /brain/drop found
    board: Board | None = None  # last /connect4/board
    new_game: bool = False
    robot_first: bool = False  # with new_game
    human_done: bool = False
    stop: bool = False
    engine: game.Choice | None = None  # the engine's answer, once it has one
    move_reply: tuple | None = None  # (success, message) of /brain/drop or /brain/handover, at the move's end
    brain_states: tuple = ()  # /brain_state messages since the last step: STOPPED cuts a move


@dataclass
class Outputs:
    think: tuple | None = None  # (me, mask): submit best_move
    drop: int | None = None  # column: call /brain/drop
    handover: bool = False  # call /brain/handover: the arm to HANDOVER, the gripper open
    stop_brain: bool = False
    transition: str | None = None


@dataclass
class Turns:
    p: Params = field(default_factory=Params)

    def __post_init__(self):
        self.state = State.IDLE
        self.reason = ""
        self.cells = (EMPTY,) * CELLS
        self.turn = "human"
        self.result = ""
        self.last = None  # {"who", "column", "planned"}; the robot's: and the engine's reason, _why
        self.lost = self.missed = self.moves = 0
        self.history = []  # the columns, in the order the tokens were seen
        self.seconds = dict.fromkeys(
            TIMED.values(), 0.0
        )  # of this game, by state; added when a state is left
        self._expect = Expect.EMPTY
        self._robot_first = False
        self._column = None
        self._why = {}  # of the robot's move: the engine's score and depth, wins_in or loses_in if proven
        self._since = None  # the first step's time: the node's clock does not start at 0

    @property
    def label(self) -> str:
        return f"{self.state}:{self.reason}" if self.reason else str(self.state)

    def accepts_new_game(self, ready: bool = True) -> bool:
        return ready and self.state in NEW_GAME_FROM

    @property
    def accepts_human_done(self) -> bool:
        return self.state in HUMAN_DONE_FROM

    def snapshot(self) -> dict:
        """/connect4/state, as JSON: what the console and the bench need, so neither holds a rule."""
        _, mask = game.from_cells(self.cells, ROBOT)
        return {
            "state": str(self.state),
            "reason": self.reason,
            "turn": self.turn,
            "board": list(self.cells),
            "legal": game.legal_columns(mask),
            "last": self.last,
            "result": self.result,
            "moves": self.moves,
            "first": "robot" if self._robot_first else "human",
            "history": list(self.history),  # with `first`, the board can be rebuilt
            "seconds": {state: round(spent, 2) for state, spent in self.seconds.items()},
            "lost": self.lost,
            "missed": self.missed,
        }

    def step(self, now, inputs) -> Outputs:
        self._now, self._in, self._out = now, inputs, Outputs()
        if self._since is None:
            self._since = now
        i, t = inputs, now - self._since
        stopped = self.state is State.MOVE and "STOPPED" in i.brain_states
        if i.stop or stopped:
            self._out.stop_brain = i.stop
            self._go(State.ERROR, "stopped")
            return self._out
        match self.state:
            case s if s in NEW_GAME_FROM and i.new_game and i.ready:
                self._new_game(i.robot_first)
            case State.WAIT_HUMAN | State.FIX_BOARD if i.human_done:
                self._go(State.READ_BOARD)
            case State.READ_BOARD:
                self._read(t)
            case State.THINK if i.engine is not None:
                self._column = i.engine.column
                self._why = {"score": i.engine.score, "depth": i.engine.depth}
                if plies := game.ends_in(i.engine.score, self.moves):
                    self._why["wins_in" if plies > 0 else "loses_in"] = abs(plies)
                self._go(State.MOVE)
            case State.THINK if t > self.p.think_timeout_s:
                self._go(State.ERROR, "timeout:THINK")
            case State.MOVE:
                self._move(t)
        return self._out

    def _new_game(self, robot_first):
        self.cells = (EMPTY,) * CELLS
        self.result, self.last = "", None
        self.lost = self.missed = self.moves = 0
        self._robot_first = robot_first
        self._expect = Expect.EMPTY
        self._column = None
        self.history = []
        self._go(State.MOVE)
        self.seconds = dict.fromkeys(TIMED.values(), 0.0)  # after _go: the last game's wait is not this one's

    def _go(self, state, reason=""):
        old, spent = self.label, self._now - self._since
        if self.state in TIMED:
            self.seconds[TIMED[self.state]] += spent
        self.state, self.reason, self._since = state, reason, self._now
        self._out.transition = f"{old} → {self.label} ({spent:.2f} s in {old})"
        match state:
            case State.MOVE:
                self._out.drop, self._out.handover = self._column, self._column is None
            case State.THINK:
                self._out.think = game.from_cells(self.cells, ROBOT)
            case State.WAIT_HUMAN:
                self._expect = Expect.HUMAN if self.turn == "human" else Expect.SAME

    def _read(self, t):
        board = self._in.board
        if board is not None and board.received > self._since:
            self._judge(board)
        if self.state is State.READ_BOARD and t > self.p.read_timeout_s:
            # lost needs evidence: a recent read with nothing new. No read at all says nothing of the token.
            seen = board is not None and self._now - board.received < self.p.board_fresh_s
            if self._expect is Expect.ROBOT and seen and board.cells == self.cells:
                self.lost += 1
                self.turn = "robot"
                self._go(State.WAIT_HUMAN, "token_lost")
            elif self._expect is Expect.EMPTY and board is not None and board.received > self._since:
                self._go(State.FIX_BOARD, "board_not_empty" if board.valid else board.reason)
            else:
                self._go(State.FIX_BOARD, f"not_seen:{self._expect}")

    def _judge(self, board):
        expect = self._expect
        if expect is Expect.EMPTY:
            # Right after the board's reset the reader may still show the last game: not empty is "not yet".
            if board.valid and not any(board.cells):
                self.turn = "robot" if self._robot_first else "human"
                self._go(State.WAIT_HUMAN)
            return
        if not board.valid:
            self._go(State.FIX_BOARD, board.reason)
            return
        new = [(i, v) for i, (a, v) in enumerate(zip(self.cells, board.cells, strict=True)) if a != v]
        if expect is Expect.SAME:
            if new:
                self._go(State.FIX_BOARD, "unexpected_token")
            else:
                self._go(State.THINK)
        elif new:  # no change is "not yet": _read's timeout decides
            self._one_token(board, new, HUMAN if expect is Expect.HUMAN else ROBOT)

    def _one_token(self, board, new, player):
        (cell, value), *others = new
        _, mask = game.from_cells(self.cells, player)
        column, row = cell % game.COLUMNS, cell // game.COLUMNS
        if others or value != player or self.cells[cell] != EMPTY or row != game.row_of(mask, column):
            self._go(State.FIX_BOARD, f"not_one_{self._expect}_token")
            return
        self.cells = tuple(board.cells)
        self.moves += 1
        self.history.append(column)
        who = "robot" if player == ROBOT else "human"
        planned = self._column if player == ROBOT else column
        self.last = {"who": who, "column": column, "planned": planned} | (
            self._why if player == ROBOT else {}
        )
        self.missed += column != planned  # the camera says where it went: the game goes on from there
        winner = game.winner(self.cells)
        if winner != EMPTY:
            self.result = "robot" if winner == ROBOT else "human"
            self._go(State.GAME_OVER)
        elif self.moves == CELLS:
            self.result = "draw"
            self._go(State.GAME_OVER)
        elif player == HUMAN:
            self.turn = "robot"
            self._go(State.THINK)
        else:
            self.turn = "human"
            self._go(State.WAIT_HUMAN)

    def _move(self, t):
        reply = self._in.move_reply
        if reply is None:
            if t > self.p.move_timeout_s:
                self._out.stop_brain = True
                self._go(State.ERROR, "timeout:MOVE")
            return
        success, message = reply
        if success:
            self._expect = Expect.EMPTY if self._column is None else Expect.ROBOT
            self._go(State.READ_BOARD)
        elif message == NO_TOKEN:  # the jaws closed on nothing: the arm has not moved
            self.lost += 1
            self.turn = "robot"
            self._go(State.WAIT_HUMAN, NO_TOKEN)
        elif message == STOPPED:  # /brain/stop cut the move: the same reason as its /brain_state
            self._go(State.ERROR, "stopped")
        else:
            self._go(State.ERROR, f"brain_refused:{message}")
