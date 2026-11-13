"""Does a deeper search play better? The package's engine against itself, depth against depth (#244, MES-70).

Each pair plays GAMES games from random two-token openings (the seed is the game's number), each side first in
half of them, no time budget: the result only depends on the code. Prints the wins of each depth, the draws,
and the longest move of the deeper side.

  tools/so101 run 'python3 /repo/docs/project/experiences/connect4/game/levels.py 40 1,4 4,10 1,10 2,6 6,10 2,10'
"""

import random
import sys

from so101_connect4 import game


def play(depths, seed):
    """(the winner's index in `depths` or None for a draw, the longest move of each side in s)."""
    rng = random.Random(seed)
    me = mask = 0
    for _ in range(2):
        me, mask = game.play(me, mask, rng.randrange(game.COLUMNS))
    longest = [0.0, 0.0]
    for ply in range(game.CELLS - 2):
        side = ply % 2
        choice = game.best_move(me, mask, None, depths[side], seed)
        longest[side] = max(longest[side], choice.seconds)
        me, mask = game.play(me, mask, choice.column)
        if game.has_four(me ^ mask):
            return side, longest
    return None, longest


def main():
    games = int(sys.argv[1])
    for pair in sys.argv[2:]:
        a, b = (int(depth) for depth in pair.split(","))
        wins, slowest = {a: 0, b: 0, "draw": 0}, 0.0
        for seed in range(games):
            depths = (a, b) if seed % 2 == 0 else (b, a)
            winner, longest = play(depths, seed)
            wins["draw" if winner is None else depths[winner]] += 1
            slowest = max(slowest, longest[depths.index(b)])
        print(
            f"depth {a} against depth {b}, {games} games: {wins[a]} - {wins[b]}, {wins['draw']} draws; "
            f"longest move at depth {b}: {slowest:.2f} s",
            flush=True,
        )


if __name__ == "__main__":
    main()
