"""E11: clearance along the joint paths (mj_geomDistance on the collision geoms), HANDOVER vs the camera, descent straightness."""
import numpy as np, mujoco
from e7_cycle import *
O2 = np.array([-0.00315, 0.00025, -0.00245])
b = Bench(gap=0.0095, offset=OFFSET, tokens=1, **CONTACT)
m, d = b.m, b.d
arm_bodies = ["shoulder", "upper_arm", "lower_arm", "wrist", "gripper", "moving_jaw_so101_v1"]
arm = [i for i in range(m.ngeom) if m.geom_contype[i] and m.body(m.geom_bodyid[i]).name in arm_bodies]
grid = [m.geom(n).id for n in ["wall_p", "wall_m"] + [f"rib_{i}" for i in range(COLS + 1)]]
tok = m.geom("token_0_geom").id
ft = np.zeros(6)
def set_q(q, grip):
    for n, v in zip(ARM, q):
        d.qpos[m.joint(n).qposadr[0]] = v
    g = m.joint("gripper").range
    d.qpos[m.joint("gripper").qposadr[0]] = g[0] + grip / 100 * (g[1] - g[0])
    a, _ = b.token_adr()
    T = fk_world(kin, q)
    if grip < 50:
        pos = T[:3, 3] + T[:3, :3] @ O2
        qq = np.zeros(4); mujoco.mju_mat2Quat(qq, np.column_stack([T[:3, 1], T[:3, 2], T[:3, 0]]).flatten())
        d.qpos[a:a + 3] = pos; d.qpos[a + 3:a + 7] = qq
    else:
        d.qpos[a:a + 3] = [1, 1, 0.02]; d.qpos[a + 3:a + 7] = [1, 0, 0, 0]
    mujoco.mj_kinematics(m, d)
def dist(with_token):
    best = (1e9, None)
    for x in arm:
        for g in grid:
            v = mujoco.mj_geomDistance(m, d, x, g, 0.5, ft)
            if v < best[0]:
                best = (v, m.body(m.geom_bodyid[x]).name)
    tk = 1e9
    if with_token:
        tk = min(mujoco.mj_geomDistance(m, d, tok, g, 0.5, ft) for g in grid)
    return best, tk
def path(q0, q1, grip, n=40):
    arm_min, who, tok_min = 1e9, None, 1e9
    for s in np.linspace(0, 1, n):
        set_q(q0 + s * (q1 - q0), grip)
        (a, w), t = dist(grip < 50)
        if a < arm_min: arm_min, who = a, w
        tok_min = min(tok_min, t)
    return arm_min, who, tok_min
for name, H in (("A", np.array([0.9, -0.7, 0.1, 0.9, ROLL])), ("B", np.array([0.9, -0.6, -0.2, 1.0, ROLL])), ("C", np.array([-0.9, -0.6, -0.2, 1.0, ROLL]))):
    print(f"HANDOVER {name} {np.round(H,3).tolist()}: self_collides {kin.self_collides(H)}, TCP base {np.round(kin.fk(H)[:3,3],3).tolist()}")
    for h in (0.008,):
        rows = []
        for i in range(7):
            q = rel_pose(i, h, o=O2)
            to = path(H, q, 3.0); back = path(q, H, 100.0)
            T = max(T_MIN, np.max(np.abs(q - H)) * 1.875 / V_MAX)
            rows.append(f"col {i}: T {T:.2f} s; in arm {to[0]*1e3:.1f} ({to[1]}) token {to[2]*1e3:.1f}; back arm {back[0]*1e3:.1f} ({back[1]})")
        print("  h=8 mm:\n   " + "\n   ".join(rows))
# descent straightness in joint interpolation
print("descent: joint-interpolated from h=+40 to h=+8 mm, TCP deviation from the straight line")
for i in range(7):
    q0, q1 = rel_pose(i, 0.040, o=O2), rel_pose(i, 0.008, o=O2)
    p0, p1 = kin.fk(q0)[:3, 3], kin.fk(q1)[:3, 3]
    dv = []
    for s in np.linspace(0, 1, 21):
        p = kin.fk(q0 + s * (q1 - q0))[:3, 3]
        t = np.dot(p - p0, p1 - p0) / np.dot(p1 - p0, p1 - p0)
        dv.append(np.linalg.norm(p - (p0 + t * (p1 - p0))))
    print(f"  col {i}: {max(dv)*1e3:.2f} mm, T {max(T_MIN, np.max(np.abs(q1-q0))*1.875/V_MAX):.2f} s, joint change {np.max(np.abs(q1-q0)):.3f} rad")
# camera
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
    D = P - pos; zc = D @ f
    return np.column_stack([w / 2 + fp * (D @ x) / zc, h / 2 - fp * (D @ y) / zc]), zc
corners = np.array([[PAN_AXIS_X + r, OFFSET + s * 0.0075, z] for s in (1, -1) for r in (R0 - PITCH / 2, R0 + 6.5 * PITCH) for z in (TOP, TOP - ROWS * PITCH)])
def pts():
    out = []
    for i in range(m.ngeom):
        if m.body(m.geom_bodyid[i]).name not in arm_bodies or m.geom_type[i] != mujoco.mjtGeom.mjGEOM_MESH: continue
        mid = m.geom_dataid[i]; a = m.mesh_vertadr[mid]; n = m.mesh_vertnum[mid]
        out.append(d.geom_xpos[i] + (d.geom_xmat[i].reshape(3, 3) @ m.mesh_vert[a:a + n].T).T)
    return np.vstack(out)
print("camera 45 cm in front of a face, 5 cm above the centre, fovy 52, 320x240: px margin between the arm and the grid box (negative: overlap)")
for pan in (-1.3, -1.1, -0.9, -0.7, 0.7, 0.9, 1.1, 1.3):
    for post in ((-0.7, 0.1, 0.9), (-0.6, -0.2, 1.0), (-0.8, -0.2, 1.2)):
        q = np.r_[pan, post, ROLL]; set_q(q, 100.0)
        P = pts(); line = []
        for s in (+1, -1):
            uv, _ = project(corners, view(s)); lo, hi = uv.min(0), uv.max(0)
            a, zc = project(P, view(s))
            dx = np.maximum(lo[0] - a[:, 0], a[:, 0] - hi[0]); dy = np.maximum(lo[1] - a[:, 1], a[:, 1] - hi[1])
            margin = np.where((dx < 0) & (dy < 0), np.maximum(dx, dy), np.hypot(np.maximum(dx, 0), np.maximum(dy, 0))).min()
            line.append(f"cam {s:+d}: {margin:6.1f}")
        print(f"  pan {pan:+.1f} post {post}: {'  '.join(line)}  T to cols {max(max(T_MIN, np.max(np.abs(rel_pose(i,0.008,o=O2)-q))*1.875) for i in range(7)):.2f} s max")
