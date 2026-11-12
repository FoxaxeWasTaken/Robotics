"""Engine measurements for ticket #78: depth in 1 s, time per depth, games against a random player.

  python3 bench_engine.py depth            depth reached in budget, empty and mid-game boards
  python3 bench_engine.py pertime          time per depth 4 to 10
  python3 bench_engine.py games LABEL N    N games against random; LABEL is d<depth> or b<budget_s>
"""

import random
import statistics
import sys
import time

sys.path.insert(0, ".")
import game
from game import ROBOT


def midgame(stones, seed):
    """A position after `stones` plies: a depth-2 player against a noisy one, nobody has won, robot to move."""
    rng = random.Random(seed)
    while True:
        me, mask = 0, 0
        for ply in range(stones):
            legal = game.legal_columns(mask)
            column = rng.choice(legal) if rng.random() < 0.5 else game.best_move(me, mask, None, 2, seed=seed).column
            mover = me
            me, mask = game.play(me, mask, column)
            if game.has_four(mover ^ me ^ mask ^ mask if False else (mask ^ me)):
                break  # the side that just played won
        else:
            return me, mask


def positions():
    yield "empty", 0, 0
    for stones in (8, 14, 20, 26):
        for seed in range(4):
            yield f"{stones} stones", *midgame(stones, 100 * stones + seed)


def depth():
    print("position      depth  nodes     s      kn/s   (budget 1 s)")
    rows = {}
    for label, me, mask in positions():
        c = game.best_move(me, mask, 1.0)
        rows.setdefault(label, []).append(c)
        print(f"{label:12s} {c.depth:4d} {c.nodes:8d} {c.seconds:6.3f} {c.nodes / max(c.seconds, 1e-6) / 1000:7.0f}", flush=True)
    print("label: depth min/median/max, longest s")
    for label, cs in rows.items():
        ds = [c.depth for c in cs]
        print(f"{label:12s} {min(ds)}/{statistics.median(ds)}/{max(ds)}  {max(c.seconds for c in cs):.3f}")


def pertime():
    print("position      depth  nodes     s")
    worst = {}
    for label, me, mask in positions():
        for d in range(4, 11):
            c = game.best_move(me, mask, None, d)
            worst[d] = max(worst.get(d, 0.0), c.seconds)
            if label == "empty" or d in (6, 8):
                print(f"{label:12s} {d:4d} {c.nodes:8d} {c.seconds:7.3f}", flush=True)
    print("depth: worst s over all positions:", {d: round(s, 3) for d, s in worst.items()})


def one_game(index, depth_cap, budget):
    """Seeded game. Odd index: the robot plays first. Returns (result, robot moves, plies, seconds in the engine)."""
    rng = random.Random(index)
    me, mask, ply = 0, 0, 0
    robot_turn = index % 2 == 1
    spent, moves, longest = 0.0, 0, 0.0
    while True:
        if robot_turn:
            choice = game.best_move(me, mask, budget, depth_cap, seed=index)
            spent += choice.seconds
            longest = max(longest, choice.seconds)
            moves += 1
            column = choice.column
        else:
            column = rng.choice(game.legal_columns(mask))
        assert column in game.legal_columns(mask)
        mover_after = me ^ mask  # stones of the player who moves next, before the drop: unchanged by it
        me, mask = game.play(me, mask, column)
        ply += 1
        if game.has_four(mask ^ me):  # the side that just played
            return ("robot" if robot_turn else "random"), moves, ply, spent, longest
        if ply == 42:
            return "draw", moves, ply, spent, longest
        robot_turn = not robot_turn


def games(label, n):
    depth_cap = int(label[1:]) if label[0] == "d" else game.CELLS
    budget = float(label[1:]) if label[0] == "b" else None
    start = time.monotonic()
    rows = [one_game(i, depth_cap, budget) for i in range(n)]
    wall = time.monotonic() - start
    wins = sum(r[0] == "robot" for r in rows)
    lost = sum(r[0] == "random" for r in rows)
    moves = [r[1] for r in rows]
    plies = [r[2] for r in rows]
    print(
        f"{label}: robot {wins}/{n}, lost {lost}, draws {n - wins - lost}; wall {wall:.1f} s; "
        f"robot moves mean {statistics.mean(moves):.1f} max {max(moves)}; plies mean {statistics.mean(plies):.1f} "
        f"max {max(plies)}; longest move {max(r[4] for r in rows):.3f} s",
        flush=True,
    )


if __name__ == "__main__":
    {"depth": depth, "pertime": pertime}[sys.argv[1]]() if sys.argv[1] != "games" else games(sys.argv[2], int(sys.argv[3]))
