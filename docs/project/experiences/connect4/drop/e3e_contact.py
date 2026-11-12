"""E3e: which token contact parameters let the jaws stop on a 6.5 mm token and hold it?"""
import numpy as np, mujoco, itertools, sys
from c4 import *
kin = Kinematics(URDF)
ROLL = np.pi / 2 + 0.0487
Q = np.array([0.0, -0.5, 0.0, 1.2, ROLL])
def in_tcp(b, i=0):
    T = fk_world(kin, b.q()); p, R = b.token_pose(i)
    return T[:3, :3].T @ (p - T[:3, 3]) * 1e3, np.degrees(np.arccos(min(1, abs(R[:3, 2] @ T[:3, 0]))))
def give(b, dx=-0.0035, dy=0.0, dz=0.0, i=0):
    T = fk_world(kin, b.q())
    p = T[:3, 3] + T[:3, :3] @ np.array([dx, dy, dz])
    R = np.column_stack([T[:3, 1], T[:3, 2], T[:3, 0]])
    b.set_token(p, R, i)
    bid = b.m.body(f"token_{i}").id
    b.d.xfrc_applied[bid, 2] = b.m.body_subtreemass[bid] * 9.81
    return bid
cases = []
for mass in (0.003, 0.01):
    for sr in ((0.02, 1.0), (0.004, 1.0), (-2e5, -500), (-1e6, -1500)):
        for si in (None, (0.99, 0.999, 0.0005, 0.5, 2)):
            for multi, cd in ((False, 3), (True, 4)):
                cases.append(dict(token_mass=mass, solref=sr, solimp=si, multiccd=multi, condim=cd))
print(len(cases), "cases")
for c in cases:
    try:
        b = Bench(grid=False, **c)
        b.teleport_arm(Q); b.wait(0.3)
        bid = give(b)
        b.goal["gripper"] = 0.0
        tr = []
        for _ in range(50):
            b.tick(); tr.append(b.obs["gripper.pos"])
        b.d.xfrc_applied[bid, 2] = 0.0
        r0, a0 = in_tcp(b)
        g0, e0 = b.obs["gripper.pos"], b.effort()
        # shake: a 1.5 rad/s pan move while held, then rest
        b.move(np.array([0.8, -0.5, 0.0, 1.2, ROLL]))
        b.wait(0.5)
        r, a = in_tcp(b)
        vel = np.abs(b.d.qvel[b.token_adr()[1]:b.token_adr()[1]+6]).max()
        print(f"mass {c['token_mass']*1e3:.0f} g solref {c['solref']} solimp {'stiff' if c['solimp'] else 'dflt'} multi {c['multiccd']} condim {c['condim']}: closed {g0:.2f} % eff {e0:.3f}; token {np.round(r0,1).tolist()} tilt {a0:.1f}; after move {b.obs['gripper.pos']:.2f} % eff {b.effort():.3f} token {np.round(r,1).tolist()} tilt {a:.1f} |v|max {vel:.2f}", flush=True)
        del b
    except Exception as e:
        print("FAILED", c, e, flush=True)
