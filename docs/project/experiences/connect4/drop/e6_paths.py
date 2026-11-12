"""E6: poses of the 7 columns, HANDOVER vs the camera, arm-to-grid clearance along the joint paths, durations."""
import numpy as np, mujoco, time, json
from c4 import *
from so101_brain.grasp_planner import tilted_down
from so101_brain.trajectory import PEAK_VELOCITY
ROLL = np.pi / 2 + 0.0487
OFFSET = -0.00485                       # grid plane from the pan axis, along +y (az = 0)
O_TCP = np.array([-0.00315, 0.0, -0.001])  # token centre in the TCP frame, held (E3e)
Z_SLOT = TOP - RISER                    # base_link
kin = Kinematics(URDF)
b = Bench(gap=0.0095, offset=OFFSET, tokens=1, token_mass=0.01, solref=(0.004, 1.0), solimp=(0.99, 0.999, 0.0005, 0.5, 2))
m, d = b.m, b.d
arm_bodies = ["shoulder", "upper_arm", "lower_arm", "wrist", "gripper", "moving_jaw_so101_v1"]
arm_geoms = [i for i in range(m.ngeom) if m.geom_contype[i] and m.body(m.geom_bodyid[i]).name in arm_bodies]
grid_geoms = [m.geom(n).id for n in ("wall_p", "wall_m")]
tok = m.geom("token_0_geom").id
ft = np.zeros(6)
def set_q(q, grip=None):
    for n, v in zip(ARM, q):
        d.qpos[m.joint(n).qposadr[0]] = v
    if grip is not None:
        g = m.joint("gripper").range
        d.qpos[m.joint("gripper").qposadr[0]] = g[0] + grip / 100 * (g[1] - g[0])
    mujoco.mj_kinematics(m, d)
    mujoco.mj_comPos(m, d) if False else None
def attach_token(q):
    T = fk_world(kin, q)
    p = T[:3, 3] + T[:3, :3] @ O_TCP
    R = np.column_stack([T[:3, 1], T[:3, 2], T[:3, 0]])
    qq = np.zeros(4); mujoco.mju_mat2Quat(qq, R.flatten())
    a, v = b.token_adr()
    d.qpos[a:a + 3] = p; d.qpos[a + 3:a + 7] = qq
    mujoco.mj_kinematics(m, d)
def park_token():
    a, v = b.token_adr()
    d.qpos[a:a + 3] = [1.0, 1.0, 0.02]; d.qpos[a + 3:a + 7] = [1, 0, 0, 0]
    mujoco.mj_kinematics(m, d)
def clearances(with_token):
    """(min distance arm-grid, its body, min distance token-grid)."""
    best, who = 1e9, None
    for a in arm_geoms:
        for g in grid_geoms:
            x = mujoco.mj_geomDistance(m, d, a, g, 0.5, ft)
            if x < best:
                best, who = x, m.body(m.geom_bodyid[a]).name
    tk = 1e9
    if with_token:
        for g in grid_geoms:
            tk = min(tk, mujoco.mj_geomDistance(m, d, tok, g, 0.5, ft))
        # top of the grid vs the bottom of the token, the ribs too
        for i in range(COLS + 1):
            tk = min(tk, mujoco.mj_geomDistance(m, d, tok, m.geom(f"rib_{i}").id, 0.5, ft))
    return best, who, tk
def tcp_for_token(c, tilt, seed):
    """TCP target so that the token centre is at c: R known from the tilt (x = -n, z = approach)."""
    z = tilted_down(tilt, 0.0); x = np.array([0, -1.0, 0]); y = np.cross(z, x)
    R = np.column_stack([x, y, z])
    p = c - R @ O_TCP
    return kin.ik_point(p, z, q_seed=seed, roll=ROLL), p
def column(i, h):
    r = R0 + i * PITCH
    return np.array([PAN_AXIS_X + r, OFFSET, Z_SLOT + h + TOKEN_R])
SEED = np.array([0.0, -0.4, 0.0, 1.0, ROLL])
print("== poses, token bottom h above the slot top; uniform tilt vs lowest tilt")
poses = {}
for tilt_name, tilts in (("70", [70] * 7), ("60", [60] * 7), ("low", [55, 45, 45, 45, 50, 55, 65])):
    for h in (0.005, 0.010, 0.040):
        for i in range(7):
            c = column(i, h)
            t0 = time.perf_counter(); q, p = tcp_for_token(c, tilts[i], SEED); dt = time.perf_counter() - t0
            poses[(tilt_name, h, i)] = q
            if q is None:
                print(f"tilt {tilt_name} h={h*1e3:.0f} col {i}: NONE ({dt*1e3:.0f} ms)")
                continue
            T = kin.fk(q)
            jaw = np.degrees(np.arccos(min(1, abs(T[:3, 0] @ [0, 1, 0]))))
            token = T[:3, 3] + T[:3, :3] @ O_TCP
            set_q(q, 3.0); attach_token(q)
            c1 = clearances(True)
            set_q(q, 100.0); park_token()
            c2 = clearances(False)
            print(f"tilt {tilt_name} h={h*1e3:3.0f} col {i}: q={np.round(q,3).tolist()} token err {np.linalg.norm(token-c)*1e3:.2f} mm jaw {jaw:.2f} tcp z {p[2]:.4f} | closed: arm-grid {c1[0]*1e3:6.1f} ({c1[1]}), token-grid {c1[2]*1e3:6.1f} | open: arm-grid {c2[0]*1e3:6.1f} ({c2[1]}) | {dt*1e3:.0f} ms", flush=True)
np.save("poses.npy", {str(k): v for k, v in poses.items()}, allow_pickle=True)
# camera: 45 cm in front of the grid face, 5 cm above its centre
def view(side):
    rc = PAN_AXIS_X + R0 + 3 * PITCH
    centre = np.array([rc, OFFSET, TOP - ROWS * PITCH / 2])
    pos = centre + np.array([0, side * 0.45, 0.05])
    f = centre - pos; f /= np.linalg.norm(f)
    x = np.cross(f, [0, 0, 1.0]); x /= np.linalg.norm(x); y = np.cross(x, f)
    return pos, x, y, f
def project(P, cam, w=320, h=240, fovy=52.0):
    pos, x, y, f = cam
    fp = (h / 2) / np.tan(np.radians(fovy) / 2)
    D = P - pos
    zc = D @ f
    return np.column_stack([w / 2 + fp * (D @ x) / zc, h / 2 - fp * (D @ y) / zc]), zc
corners = []
for s in (+1, -1):
    for ri in (R0 - PITCH / 2, R0 + 6.5 * PITCH):
        for zz in (TOP, TOP - ROWS * PITCH):
            corners.append([PAN_AXIS_X + ri, OFFSET + s * 0.0075, zz])
corners = np.array(corners)
def arm_points():
    pts = []
    for i in range(m.ngeom):
        if m.body(m.geom_bodyid[i]).name not in arm_bodies or m.geom_type[i] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mid = m.geom_dataid[i]; a = m.mesh_vertadr[mid]; n = m.mesh_vertnum[mid]
        V = m.mesh_vert[a:a + n]
        pts.append(d.geom_xpos[i] + (d.geom_xmat[i].reshape(3, 3) @ V.T).T)
    return np.vstack(pts)
def overlap(cam):
    uv, _ = project(corners, cam)
    lo, hi = uv.min(0), uv.max(0)
    P = arm_points()
    a, zc = project(P, cam)
    inside = (a[:, 0] > lo[0]) & (a[:, 0] < hi[0]) & (a[:, 1] > lo[1]) & (a[:, 1] < hi[1]) & (zc > 0)
    return inside.sum(), lo.round(0), hi.round(0), (a[:, 0].min(), a[:, 0].max()) if len(a) else None
print("\n== grid box in the image (320x240), camera on each side:")
for side in (+1, -1):
    uv, _ = project(corners, view(side))
    print(f" side {side:+d}: u {uv[:,0].min():.0f}..{uv[:,0].max():.0f}, v {uv[:,1].min():.0f}..{uv[:,1].max():.0f}")
print("\n== HANDOVER candidates: mesh vertices of the arm inside the grid box of the image; duration to the 7 columns (tilt 70, h=10 mm)")
targets = [poses[("70", 0.01, i)] for i in range(7)]
postures = {"mid": targets[3][1:4], "park": np.array([-1.7, 1.5, 0.7]), "bent": np.array([-0.9, 0.5, 1.0]), "tall": np.array([-0.6, -0.2, 1.0])}
for name, post in postures.items():
    for pan in (-1.9, -1.6, -1.3, -1.0, -0.8, -0.6, 0.6, 0.8, 1.0, 1.3, 1.6, 1.9):
        q = np.r_[pan, post, ROLL]
        set_q(q, 100.0); park_token()
        hits = {s: overlap(view(s))[0] for s in (+1, -1)}
        durs = [max(T_MIN, np.max(np.abs(t - q)) * PEAK_VELOCITY / V_MAX) for t in targets]
        print(f"{name:5s} pan {pan:+.1f}: vertices over the grid: cam+ {hits[1]:6d} cam- {hits[-1]:6d}; T to column {min(durs):.2f}..{max(durs):.2f} s; self_collides {kin.self_collides(q)}; TCP {np.round(kin.fk(q)[:3,3],3).tolist()}")
