"""E12: fill the whole grid, 42 tokens, tokens left in place: HANDOVER B, the chosen roll, tilt and offsets. Stack heights, jams."""
import numpy as np, sys, json, time
import e7_cycle
from e7_cycle import *
O2 = np.array([-0.00315, 0.00025, -0.00245])
H = np.array([0.9, -0.6, -0.2, 1.0, ROLL])
gap, bias = float(sys.argv[1]), float(sys.argv[2])
b = Bench(gap=gap, offset=OFFSET, tokens=42, **CONTACT)
rng = np.random.default_rng(11)
heights = [0] * 7
order = []
t_walls = time.time()
res = []
T = []
for k in range(42):
    legal = [c for c in range(7) if heights[c] < 6]
    i = int(rng.choice(legal))
    b.teleport_arm(H); b.wait(0.2)
    # the sim's hand: gravity off until the jaws hold
    dy, dz = rng.uniform(-3e-3, 3e-3), rng.uniform(-5e-3, 5e-3)
    bid = give(b, dy, dz, k)
    t0 = b.t
    b.goal["gripper"] = 0.0; b.wait(0.8)
    b.d.xfrc_applied[bid, 2] = 0.0
    held = b.effort() > 0.06
    q = rel_pose(i, 0.008, o=O2, across=bias)
    r1 = b.move(q)
    b.goal["gripper"] = 100.0; b.wait(0.5)
    r2 = b.move(H); b.wait(0.3)
    T.append(b.t - t0)
    p, R = b.token_pose(k)
    col = int(round((p[0] - PAN_AXIS_X - R0) / PITCH))
    res.append((k, i, heights[i], col, round(p[2] * 1e3, 1), round((p[1] - OFFSET) * 1e3, 1), held, r1[0] is not None, r2[0] is not None))
    heights[i] += 1
print("gap", gap, "bias", bias, "cycle time per move, s (from the close command to HANDOVER + 0.3): min", round(min(T), 2), "max", round(max(T), 2), "wall s", round(time.time() - t_walls))
bad = [r for r in res if r[1] != r[3] or r[4] > 300 or r[4] < 30]
print("in the right column:", 42 - len(bad), "/42; held", sum(r[6] for r in res), "reached", sum(r[7] and r[8] for r in res))
for r in bad: print("  bad", r)
# final state of every token
rows = {}
for k, (idx, i, row, *_ ) in enumerate(res):
    p, R = b.token_pose(k)
    rows.setdefault(row, []).append(p[2] * 1e3)
print("settled z of the token centre per stack level, mm (cell centre for a 37 mm row pitch in brackets):")
for row in sorted(rows):
    print(f"  level {row}: {np.round(np.median(rows[row]),1)} (min {min(rows[row]):.1f} max {max(rows[row]):.1f}) [cell {TOP*1e3 - 222 + 18.5 + row*37:.1f}]")
tilt = [np.degrees(np.arccos(min(1, abs(b.token_pose(k)[1][:3, 2] @ [0, 1.0, 0])))) for k in range(42)]
print("settled token tilt from the grid normal, deg: median", round(float(np.median(tilt)), 1), "max", round(max(tilt), 1))
