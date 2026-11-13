"""The Connect Four engine (phase2/P2-02 §4): pure Python, standard library only, no ROS. Ported from the
prototype of docs/project/experiences/connect4/game/ [MES-66].

Two integers: `me`, the stones of the side to move, and `mask`, all the stones. Each column takes 7 bits,
6 cells and a sentinel bit that keeps the shifts of one column out of the next: bit = column * 7 + row, row 0
at the bottom. Columns are 0-based (0 to 6). `cells` is the int8[42] of /connect4/board: row-major from the
bottom row, 0 empty, 1 human, 2 robot (P2-02 §6).

best_move() is deterministic for one (me, mask, max_depth, seed) when budget_s is None, or not reached: a seed
then replays the same game. With a time budget cut short, the depth reached depends on the machine.
Among moves of equal score the most central column is played; the seed only decides between two as central.

The level is the depth, LEVELS: depth alone orders this engine's strength only loosely, these three are the
ones measured in order [MES-70].

The search: negamax with alpha-beta, center-first ordering, a transposition table, iterative deepening. At
each node a playable winning cell ends the search; one playable cell of the opponent's is forced, two lose; a
move just below an opponent's winning cell is never played. At the depth limit: the threats and the center.
"""

import random
import time
from typing import NamedTuple

COLUMNS = 7
ROWS = 6
EMPTY, HUMAN, ROBOT = 0, 1, 2
CELLS = COLUMNS * ROWS
_H = ROWS + 1
_BOTTOM = sum(1 << (c * _H) for c in range(COLUMNS))
_BOARD = _BOTTOM * ((1 << ROWS) - 1)
_COLUMN = [((1 << ROWS) - 1) << (c * _H) for c in range(COLUMNS)]
_ORDER = (3, 2, 4, 1, 5, 0, 6)  # center first
_CENTER = _COLUMN[3]
_WIN = (
    1000  # minus the stones on the board: the sooner the better, and the score depends on the position only
)
_IS_MATE = _WIN - CELLS - 1  # any score beyond it is a proven win or loss
_CHECK_EVERY = 4095  # nodes between two looks at the clock, mask of the counter
LEVELS = {"easy": 1, "medium": 4, "hard": 10}  # max_depth; each beats the one before [MES-70]


class Choice(NamedTuple):
    column: int
    score: int  # > 0 good for the side to move; beyond _IS_MATE a proven result
    depth: int  # last iteration completed
    nodes: int
    seconds: float


class _TimeoutError(Exception):
    pass


def _bit(column: int, row: int) -> int:
    return 1 << (column * _H + row)


def from_cells(cells, player: int) -> tuple[int, int]:
    """(me, mask) for `player` to move, from the 42 cells."""
    me = mask = 0
    for i, cell in enumerate(cells):
        if cell != EMPTY:
            bit = _bit(i % COLUMNS, i // COLUMNS)
            mask |= bit
            if cell == player:
                me |= bit
    return me, mask


def legal_columns(mask: int) -> list[int]:
    return [c for c in range(COLUMNS) if not mask & _bit(c, ROWS - 1)]


def row_of(mask: int, column: int) -> int:
    """The row a token dropped in `column` lands on; ROWS if the column is full."""
    return (mask & _COLUMN[column]).bit_count()


def play(me: int, mask: int, column: int) -> tuple[int, int]:
    """(me, mask) of the opponent, to move once `me` has dropped in `column`."""
    return me ^ mask, mask | (mask + _bit(column, 0))


def has_four(stones: int) -> bool:
    for shift in (1, _H, _H - 1, _H + 1):
        pairs = stones & (stones >> shift)
        if pairs & (pairs >> 2 * shift):
            return True
    return False


def winner(cells) -> int:
    """HUMAN, ROBOT or EMPTY."""
    for player in (HUMAN, ROBOT):
        me, _ = from_cells(cells, player)
        if has_four(me):
            return player
    return EMPTY


def four(cells) -> tuple:
    """The cells of the lines of four or more, as indices of `cells`; () while nobody has won."""
    for player in (HUMAN, ROBOT):
        stones, _ = from_cells(cells, player)
        won = 0
        for shift in (1, _H, _H - 1, _H + 1):
            pairs = stones & (stones >> shift)
            first = pairs & (pairs >> 2 * shift)  # the lowest stone of each four
            won |= first | first << shift | first << 2 * shift | first << 3 * shift
        if won:
            return tuple(i for i in range(CELLS) if won & _bit(i % COLUMNS, i // COLUMNS))
    return ()


def ends_in(score: int, stones: int) -> int:
    """Plies to the end of a proven game, from a Choice's score found with `stones` tokens on the board:
    > 0 the side to move wins in that many plies, its own included; < 0 it loses; 0 not proven."""
    if abs(score) <= _IS_MATE:
        return 0
    return (_WIN - abs(score) - stones) * (1 if score > 0 else -1)


def _threats(stones: int, mask: int) -> int:
    """The empty cells where `stones` would complete four (not only the playable ones)."""
    found = (stones << 1) & (stones << 2) & (stones << 3)
    for s in (_H, _H - 1, _H + 1):
        up = stones << s
        down = stones >> s
        left = up & (stones << 2 * s)
        right = down & (stones >> 2 * s)
        found |= left & (stones << 3 * s) | left & down | right & up | right & (stones >> 3 * s)
    return found & _BOARD & ~mask


class _Search:
    """One best_move's negamax: its deadline, its node count, its transposition table."""

    __slots__ = ("deadline", "nodes", "table")

    def __init__(self, deadline):
        self.deadline, self.nodes, self.table = deadline, 0, {}

    def score(self, me, mask, depth, alpha, beta) -> int:
        """The position's score for `me`, to play, within (alpha, beta)."""
        self.nodes += 1
        if self.deadline is not None and not self.nodes & _CHECK_EVERY and time.monotonic() > self.deadline:
            raise _TimeoutError
        stones = mask.bit_count()
        if stones == CELLS:
            return 0
        possible = (mask + _BOTTOM) & _BOARD
        mine = _threats(me, mask)
        if possible & mine:
            return _WIN - stones - 1
        theirs = _threats(me ^ mask, mask)
        forced = possible & theirs
        if forced:
            if forced & (forced - 1):  # two cells to block
                return stones + 2 - _WIN
            possible = forced
        possible &= ~(theirs >> 1)  # never play just below their winning cell
        if not possible:
            return stones + 2 - _WIN
        if depth <= 0:
            return 2 * (mine.bit_count() - theirs.bit_count()) + (me & _CENTER).bit_count()
        return self._deeper(me, mask, depth, alpha, beta, possible)

    def _deeper(self, me, mask, depth, alpha, beta, possible) -> int:
        """The best of the `possible` moves, through the table: what a search as deep already knows."""
        key = me + mask  # unique for a position: each column's stones plus its first empty cell
        hit = self.table.get(key)
        first = 0
        if hit is not None:
            entry_depth, kind, value, first = hit
            if entry_depth >= depth:
                if kind == 0:
                    return value
                if kind > 0:
                    alpha = max(alpha, value)
                else:
                    beta = min(beta, value)
                if alpha >= beta:
                    return value
        best, best_bit = self._best(me, mask, depth, alpha, beta, possible, first)
        # exact (0), or a bound: 1 when the search was cut (at least `best`), -1 when no move reached alpha
        kind = 0
        if best >= beta:
            kind = 1
        elif best <= alpha:
            kind = -1
        self.table[key] = (depth, kind, best, best_bit)
        return best

    def _best(self, me, mask, depth, alpha, beta, possible, first) -> tuple[int, int]:
        """(score, bit) of the best move: the table's move first, then the columns from the center."""
        order = [bit for column in _ORDER if (bit := _COLUMN[column] & possible) and bit != first]
        if first & possible:
            order.insert(0, first)
        best, best_bit = -_WIN - 1, 0
        for bit in order:
            score = -self.score(me ^ mask, mask | bit, depth - 1, -beta, -alpha)
            if score > best:
                best, best_bit = score, bit
                if score > alpha:
                    alpha = score
                    if alpha >= beta:
                        break
        return best, best_bit

    def root(self, me, mask, columns, depth) -> tuple[dict, int]:
        """({column: score}, the best score) of the root's moves at `depth`."""
        scores, best = {}, -_WIN - 1
        for column in columns:
            child_me, child_mask = play(me, mask, column)
            if has_four(me | (child_mask ^ mask)):
                score = _WIN - mask.bit_count() - 1
            else:
                # best - 1, not best: a move as good as the best gets its exact score, for the tie-break
                score = -self.score(child_me, child_mask, depth - 1, -_WIN - 1, -(best - 1))
            scores[column] = score
            best = max(best, score)
        return scores, best


def best_move(
    me: int, mask: int, budget_s: float | None = 1.0, max_depth: int = CELLS, seed: int = 0
) -> Choice:
    """Iterative deepening to max_depth, within budget_s. Among equal best moves at the root: the most central
    column, and `seed` between two as central."""
    start = time.monotonic()
    search = _Search(None if budget_s is None else start + budget_s)
    legal = legal_columns(mask)
    if not legal:
        raise ValueError("no legal move: the board is full")
    if len(legal) == 1:
        return Choice(legal[0], 0, 0, 0, time.monotonic() - start)
    rng = random.Random(seed ^ (me + mask))
    root = [c for c in _ORDER if c in legal]
    done = Choice(root[0], 0, 0, 0, 0.0)
    for depth in range(1, min(max_depth, CELLS - mask.bit_count()) + 1):
        try:
            scores, best = search.root(me, mask, root, depth)
        except _TimeoutError:
            break
        ties = [c for c, score in scores.items() if score == best]
        near = min(abs(c - COLUMNS // 2) for c in ties)
        central = [c for c in ties if abs(c - COLUMNS // 2) == near]
        done = Choice(rng.choice(central), best, depth, search.nodes, time.monotonic() - start)
        root.sort(key=scores.get, reverse=True)  # stable: center first among equals
        if abs(best) > _IS_MATE:
            break
    return done._replace(nodes=search.nodes, seconds=time.monotonic() - start)
