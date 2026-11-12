"""E3d: close on a token that gravity leaves alone until the jaws hold it (xfrc_applied = the sim's hand)."""
import numpy as np, mujoco, sys
from c4 import *
kin = Kinematics(URDF)
ROLL = np.pi / 2 + 0.0487
Q = {"tool45": np.array([0.0, -0.5, 0.0, 1.2, ROLL]), "tool75": np.array([0, -0.2, 0.1, 0.4, ROLL]), "tool0": np.array([0, -0.2, 0.3, 1.5, ROLL])}
def in_tcp(b, i=0):
    T = fk_world(kin, b.q()); p, R = b.token_pose(i)
    return T[:3, :3].T @ (p - T[:3, 3]) * 1e3, np.degrees(np.arccos(min(1, abs(R[:3, 2] @ T[:3, 0]))))
def give(b, dx, dy=0.0, dz=0.0, i=0):
    T = fk_world(kin, b.q())
    p = T[:3, 3] + T[:3, :3] @ np.array([dx, dy, dz])
    R = np.column_stack([T[:3, 1], T[:3, 2], T[:3, 0]])
    b.set_token(p, R, i)
    bid = b.m.body(f"token_{i}").id
    b.d.xfrc_applied[bid, 2] = b.m.body_subtreemass[bid] * 9.81
    return bid
def run(q, close, dx=-0.0035, dy=0.0, dz=0.0, v=True, **kw):
    b = Bench(grid=False, **kw)
    b.teleport_arm(q); b.wait(0.3)
    bid = give(b, dx, dy, dz)
    b.goal["gripper"] = close
    t = 0; hist = []
    while t < 1.0:
        b.tick(); t += DT; hist.append((t, b.obs["gripper.pos"], b.effort()))
    b.d.xfrc_applied[bid, 2] = 0.0   # the hand lets go
    rel0, a0 = in_tcp(b)
    b.wait(1.0)
    rel, a = in_tcp(b)
    return b, hist, rel0, a0, rel, a
for name, q in Q.items():
    for close in (0.0, 2.0, 2.8):
        b, h, r0, a0, r, a = run(q, close)
        print(f"{name} close->{close}: at 1 s {h[-1][1]:.2f} % eff {h[-1][2]:.3f}; let go +1 s {b.obs['gripper.pos']:.2f} % eff {b.effort():.3f}; token xyz {np.round(r0,2).tolist()} -> {np.round(r,2).tolist()} mm, axis/jaw {a0:.2f}->{a:.2f} deg")
        del b
# 40 random handovers (dy +-3, dz +-5; dx mid-gap vs near the fixed jaw)
rng = np.random.default_rng(1)
for dx0, label in ((-0.0035, "against the fixed jaw"), (-0.016, "mid gap")):
    rows = []
    for k in range(30):
        dy, dz = rng.uniform(-3e-3, 3e-3), rng.uniform(-5e-3, 5e-3)
        b, h, r0, a0, r, a = run(Q["tool45"], 0.0, dx=dx0, dy=dy, dz=dz)
        rows.append((dy*1e3, dz*1e3, *r, a, b.obs["gripper.pos"], b.effort()))
        del b
    x = np.array(rows)
    held = np.abs(x[:, 3]) < 40
    print(f"{label}: held {held.sum()}/30; token x {np.round(np.percentile(x[held,2],[0,50,100]),2)} y-dy {np.round(np.percentile(x[held,3]-x[held,0],[0,50,100]),2)} z-dz {np.round(np.percentile(x[held,4]-x[held,1],[0,50,100]),2)} axis {np.round(np.percentile(x[held,5],[0,50,100]),2)} grip % {np.round(np.percentile(x[held,6],[0,50,100]),2)} effort {np.round(np.percentile(x[held,7],[0,50,100]),3)}")
