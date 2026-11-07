"""E9: robot moves per game against a random player: negamax depth 4 (scratch), robot second then first."""
import random, statistics
R, C = 6, 7
def drop(b, c, p):
    for r in range(R):
        if b[r][c] == 0:
            b[r][c] = p; return r
def undo(b, c):
    for r in range(R - 1, -1, -1):
        if b[r][c]:
            b[r][c] = 0; return
def wins(b, r, c, p):
    for dr, dc in ((0, 1), (1, 0), (1, 1), (1, -1)):
        n = 1
        for s in (1, -1):
            i, j = r + s * dr, c + s * dc
            while 0 <= i < R and 0 <= j < C and b[i][j] == p:
                n += 1; i += s * dr; j += s * dc
        if n >= 4: return True
    return False
def legal(b): return [c for c in (3, 2, 4, 1, 5, 0, 6) if b[R - 1][c] == 0]
def neg(b, p, d, a, bb):
    ms = legal(b)
    if not ms: return 0
    if d == 0: return 0
    best = -10**6
    for c in ms:
        r = drop(b, c, p)
        v = (1000 + d) if wins(b, r, c, p) else -neg(b, 3 - p, d - 1, -bb, -a)
        undo(b, c)
        best = max(best, v); a = max(a, v)
        if a >= bb: break
    return best
def choose(b, p, d, rng):
    scored = []
    for c in legal(b):
        r = drop(b, c, p)
        v = (1000 + d) if wins(b, r, c, p) else -neg(b, 3 - p, d - 1, -10**6, 10**6)
        undo(b, c); scored.append((v, c))
    m = max(v for v, _ in scored)
    return rng.choice([c for v, c in scored if v == m])
rng = random.Random(0)
res = []
for g in range(100):
    b = [[0] * C for _ in range(R)]
    p = 1 + g % 2  # 1 random, 2 robot
    robot, moves, win = 0, 0, None
    while legal(b):
        c = rng.choice(legal(b)) if p == 1 else choose(b, 2, 4, rng)
        r = drop(b, c, p); moves += 1; robot += p == 2
        if wins(b, r, c, p):
            win = p; break
        p = 3 - p
    res.append((robot, moves, win))
rm = [x[0] for x in res]
print("robot moves per game: mean", statistics.mean(rm), "median", statistics.median(rm), "max", max(rm), "wins", sum(x[2] == 2 for x in res), "/100; total tokens mean", statistics.mean(x[1] for x in res))
