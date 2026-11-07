"""Self-check of the prototype: brute-force comparison of every bit trick. Run: python3 test_game.py"""

import random

import game
from game import COLUMNS, EMPTY, HUMAN, ROBOT, ROWS


def brute_winner(cells):
    at = lambda c, r: cells[r * COLUMNS + c]  # noqa: E731
    for r in range(ROWS):
        for c in range(COLUMNS):
            v = at(c, r)
            if v == EMPTY:
                continue
            for dc, dr in ((1, 0), (0, 1), (1, 1), (1, -1)):
                if all(
                    0 <= c + k * dc < COLUMNS and 0 <= r + k * dr < ROWS and at(c + k * dc, r + k * dr) == v
                    for k in range(4)
                ):
                    return v
    return EMPTY


def brute_threats(cells, player):
    """Empty cells where player would make four (gravity ignored), as bits."""
    bits = 0
    for i, v in enumerate(cells):
        if v == EMPTY:
            trial = list(cells)
            trial[i] = player
            if _four_through(trial, i, player):  # through this cell: the board may already hold a four
                bits |= game._bit(i % COLUMNS, i // COLUMNS)
    return bits


def _four_through(cells, i, player):
    c0, r0 = i % COLUMNS, i // COLUMNS
    at = lambda c, r: cells[r * COLUMNS + c] if 0 <= c < COLUMNS and 0 <= r < ROWS else -1  # noqa: E731
    for dc, dr in ((1, 0), (0, 1), (1, 1), (1, -1)):
        for back in range(4):
            if all(at(c0 + (k - back) * dc, r0 + (k - back) * dr) == player for k in range(4)):
                return True
    return False


def random_cells(rng, stones):
    cells = [EMPTY] * 42
    heights = [0] * 7
    player = HUMAN
    placed = 0
    while placed < stones:
        c = rng.randrange(7)
        if heights[c] == 6:
            continue
        cells[heights[c] * 7 + c] = player
        heights[c] += 1
        placed += 1
        player = HUMAN + ROBOT - player
    return cells


def main():
    rng = random.Random(1)
    for trial in range(20000):
        cells = random_cells(rng, rng.randrange(0, 43))
        me, mask = game.from_cells(cells, ROBOT)
        opp = me ^ mask
        wins = {HUMAN: _any_four(cells, HUMAN), ROBOT: _any_four(cells, ROBOT)}
        assert game.has_four(me) == wins[ROBOT], cells
        assert game.has_four(opp) == wins[HUMAN], cells
        assert game._threats(me, mask) == brute_threats(cells, ROBOT), (trial, cells)
        assert game.legal_columns(mask) == [c for c in range(7) if cells[35 + c] == EMPTY]
        for c in game.legal_columns(mask):
            row = game.row_of(mask, c)
            assert cells[row * 7 + c] == EMPTY and (row == 0 or cells[(row - 1) * 7 + c] != EMPTY)
            opp2, mask2 = game.play(me, mask, c)
            assert opp2 == opp and mask2 == mask | game._bit(c, row)
    print("bit tricks: 20000 random boards agree with brute force")

    # the mirrored position gets the same score
    for trial in range(200):
        cells = random_cells(rng, rng.randrange(0, 24))
        if _any_four(cells, HUMAN) or _any_four(cells, ROBOT):
            continue
        mirror = [cells[r * 7 + (6 - c)] for r in range(6) for c in range(7)]
        a = game.best_move(*game.from_cells(cells, ROBOT), None, 4)
        b = game.best_move(*game.from_cells(mirror, ROBOT), None, 4)
        assert a.score == b.score, (cells, a, b)
    print("mirror: same score at depth 4")

    # wins at once, blocks, deterministic
    win = [EMPTY] * 42
    for c, v in ((0, ROBOT), (1, ROBOT), (2, ROBOT)):
        win[c] = v
    win[7] = HUMAN
    win[8] = HUMAN
    win[9] = HUMAN
    me, mask = game.from_cells(win, ROBOT)
    assert game.best_move(me, mask, None, 6).column == 3
    block = [EMPTY] * 42
    for c, v in ((0, HUMAN), (1, HUMAN), (2, HUMAN), (6, ROBOT), (5, ROBOT)):
        block[c] = v
    me, mask = game.from_cells(block, ROBOT)
    assert game.best_move(me, mask, None, 6).column == 3
    me, mask = game.from_cells([EMPTY] * 42, ROBOT)
    assert len({game.best_move(me, mask, None, 6, seed=7).column for _ in range(3)}) == 1
    print("wins at once, blocks, deterministic")


def _any_four(cells, player):
    for i, v in enumerate(cells):
        if v == player and _four_through(cells, i, player):
            return True
    return False


if __name__ == "__main__":
    main()
