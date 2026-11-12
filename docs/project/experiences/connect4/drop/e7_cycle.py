"""E7: full closed-loop moves: HANDOVER -> release -> HANDOVER, driver tick + min-jerk, real contacts; clearance logged."""
import numpy as np, mujoco, time, sys, json
from c4 import *
from so101_brain.grasp_planner import tilted_down
ROLL = np.pi / 2 + 0.0487
OFFSET = -0.00485
O_TCP = np.array([-0.00315, 0.0, -0.001])
Z_SLOT = TOP - RISER
TILT = 70.0
CONTACT = dict(token_mass=0.01, solref=(0.004, 1.0), solimp=(0.99, 0.999, 0.0005, 0.5, 2))
HANDOVER = np.array([0.9, -0.7, 0.1, 0.9, ROLL])
kin = Kinematics(URDF)
R_NOM = None
def rel_pose(i, h, along=0.0, across=0.0, seed=None, o=O_TCP):
    """Token centre target -> joints: position IK with the roll locked gives the arm plane, then the tilt in it."""
    c = np.array([PAN_AXIS_X + R0 + i * PITCH + along, OFFSET + across, Z_SLOT + h + TOKEN_R])
    seed = seed if seed is not None else np.array([0.0, -0.4, 0.0, 1.0, ROLL])
    az = 0.0
    for _ in range(2):
        z = tilted_down(TILT, az); x = np.array([np.sin(az), -np.cos(az), 0.0]); y = np.cross(z, x)
        p = c - np.column_stack([x, y, z]) @ o
        q0 = kin.ik_pos(p, q_seed=seed, roll=ROLL)
        az = -q0[0]
    z = tilted_down(TILT, az)
    return kin.ik_point(p, z, q_seed=q0, roll=ROLL)
def give(b, dy=0.0, dz=0.0, i=0):
    T = fk_world(kin, b.q())
    p = T[:3, 3] + T[:3, :3] @ np.array([-0.0035, dy, dz])
    R = np.column_stack([T[:3, 1], T[:3, 2], T[:3, 0]])
    b.set_token(p, R, i)
    bid = b.m.body(f"token_{i}").id
    b.d.xfrc_applied[bid, 2] = b.m.body_subtreemass[bid] * 9.81
    return bid
class Log:
    def __init__(self, b):
        self.b, self.hits = b, {}
        self.grid = {b.m.geom(n).id for n in [f"rib_{i}" for i in range(COLS + 1)] + ["wall_p", "wall_m"]}
        self.tok = b.m.geom("token_0_geom").id
        self.phase = ""
    def __call__(self, b):
        for c in b.d.contact[: b.d.ncon]:
            g = {c.geom1, c.geom2}
            if g & self.grid:
                other = (g - self.grid).pop() if len(g - self.grid) else None
                if other is None: continue
                name = "token" if other == self.tok else b.m.body(b.m.geom_bodyid[other]).name
                self.hits[(self.phase, name)] = self.hits.get((self.phase, name), 0) + 1
def cycle(b, i, h, dy=0.0, dz=0.0, along=0.0, across=0.0, drop_seed=None, o=O_TCP, verbose=False, settle=0.0, q_rel=None):
    log = Log(b); b.on_tick = log
    b.teleport_arm(HANDOVER); b.wait(0.2)
    a, v = b.token_adr()
    log.phase = "close"
    bid = give(b, dy, dz)
    b.goal["gripper"] = 0.0
    b.wait(0.8)
    b.d.xfrc_applied[bid, 2] = 0.0
    held = b.effort() > 0.06
    grip = b.obs["gripper.pos"]; eff = b.effort()
    q = q_rel if q_rel is not None else rel_pose(i, h, along, across, o=o)
    log.phase = "to_column"
    t1, T1 = b.move(q)
    if settle: b.wait(settle)
    Tn = fk_world(kin, b.q()); p, R = b.token_pose()
    ref = np.array([PAN_AXIS_X + R0 + i * PITCH, OFFSET, TOP + TOKEN_R + h])
    err = p - ref
    tilt = np.degrees(np.arccos(min(1, abs(R[:3, 2] @ [0, 1.0, 0]))))
    rel = Tn[:3, :3].T @ (p - Tn[:3, 3]) * 1e3
    tcp_err = np.linalg.norm(b.d.site("gripperframe").xpos - fk_world(kin, q)[:3, 3]) * 1e3
    log.phase = "release"
    b.goal["gripper"] = 100.0
    b.wait(0.5)
    log.phase = "back"
    t2, T2 = b.move(HANDOVER)
    log.phase = "settle"
    b.wait(0.5)
    p2, R2 = b.token_pose()
    along_col = (p2[0] - PAN_AXIS_X - R0) / PITCH
    lands = int(round(along_col)) if p2[2] < TOP - 0.05 else None
    bottom = TOP - ROWS * PITCH + TOKEN_R
    return dict(col=i, h=h, held=held, grip=round(grip, 2), eff=round(eff, 3), T=(round(T1, 2), round(T2, 2)), t=(t1, t2), err_mm=np.round(err * 1e3, 2).tolist(), tilt=round(tilt, 2), rel=np.round(rel, 2).tolist(),
                tcp_err=round(tcp_err, 2), land=lands, z_final=round(p2[2], 3), bottom=round(bottom, 3), across_final=round((p2[1] - OFFSET) * 1e3, 2), hits={f"{k[0]}:{k[1]}": v for k, v in log.hits.items()})
def park(b):
    a, v = b.token_adr()
    b.d.qpos[a:a + 3] = [1.0, 1.0, 0.02]; b.d.qpos[a + 3:a + 7] = [1, 0, 0, 0]; b.d.qvel[v:v + 6] = 0
if __name__ == "__main__":
    print("HANDOVER self_collides", kin.self_collides(HANDOVER), "TCP", kin.fk(HANDOVER)[:3, 3].round(3))
    for gap in (0.0085, 0.0095, 0.0105):
        b = Bench(gap=gap, offset=OFFSET, tokens=1, **CONTACT)
        for h in (0.010, 0.005, 0.002, -0.003):
            ok = 0; rows = []
            for i in range(7):
                r = cycle(b, i, h)
                park(b)
                rows.append(r); ok += r["land"] == i and abs(r["z_final"] - r["bottom"]) < 0.01
            print(f"gap {gap*1e3:.1f} h {h*1e3:+.0f}: right column at the bottom {ok}/7", flush=True)
            for r in rows:
                print("   ", json.dumps(r), flush=True)
