"""The Connect Four engine (phase2/P2-02 §4): pure Python, standard library only, no ROS. Ported from the
prototype of docs/project/experiences/connect4/game/ [MES-66].

Two integers: `me`, the stones of the side to move, and `mask`, all the stones. Each column takes 7 bits,
6 cells and a sentinel bit that keeps the shifts of one column out of the next: bit = column * 7 + row, row 0
at the bottom. Columns are 0-based (0 to 6). `cells` is the int8[42] of /connect4/board: row-major from the
bottom row, 0 empty, 1 human, 2 robot (P2-02 §6).

best_move() is deterministic for one (me, mask, max_depth, seed) when budget_s is None. With a time budget the
depth reached depends on the machine: a person plays against the 1 s budget, the benches and the tests give
max_depth and no budget, so a seed replays the same game.

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


def best_move(
    me: int, mask: int, budget_s: float | None = 1.0, max_depth: int = CELLS, seed: int = 0
) -> Choice:
    """Iterative deepening to max_depth, within budget_s. Equal best moves at the root are drawn by `seed`."""
    start = time.monotonic()
    deadline = None if budget_s is None else start + budget_s
    table = {}
    nodes = 0

    def search(me, mask, depth, alpha, beta):
        nonlocal nodes
        nodes += 1
        if deadline is not None and not nodes & _CHECK_EVERY and time.monotonic() > deadline:
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
        key = me + mask  # unique for a position: each column's stones plus its first empty cell
        hit = table.get(key)
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
        original_alpha = alpha
        best = -_WIN - 1
        best_bit = 0
        order = [_COLUMN[c] & possible for c in _ORDER]
        if first & possible:
            order.insert(0, first)
        for bit in order:
            if not bit or (bit == first and best != -_WIN - 1):
                continue
            score = -search(me ^ mask, mask | bit, depth - 1, -beta, -alpha)
            if score > best:
                best, best_bit = score, bit
                if score > alpha:
                    alpha = score
                    if alpha >= beta:
                        break
        kind = 1 if best >= beta else -1 if best <= original_alpha else 0
        table[key] = (depth, kind, best, best_bit)
        return best

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
            scores = {}
            best = -_WIN - 1
            for column in root:
                child_me, child_mask = play(me, mask, column)
                if has_four(me | (child_mask ^ mask)):
                    score = _WIN - mask.bit_count() - 1
                else:
                    # best - 1, not best: a move as good as the best gets its exact score, for the tie-break
                    score = -search(child_me, child_mask, depth - 1, -_WIN - 1, -(best - 1))
                scores[column] = score
                best = max(best, score)
        except _TimeoutError:
            break
        ties = [c for c, score in scores.items() if score == best]
        done = Choice(rng.choice(ties), best, depth, nodes, time.monotonic() - start)
        root.sort(key=lambda c: -scores[c])  # stable: center first among equals
        if abs(best) > _IS_MATE:
            break
    return done._replace(nodes=nodes, seconds=time.monotonic() - start)
