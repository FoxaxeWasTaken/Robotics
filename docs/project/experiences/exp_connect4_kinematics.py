"""Phase 2 (Connect Four): can the 5-DoF arm drop a token into a vertical grid? Where, how, how fast?

The token must stay in the grid plane, so the jaw closing axis (x of gripper_frame_link) must be
normal to the grid. Kinematics only (URDF + ikpy), no sim needed. Feeds MES-27 to MES-29, MES-31, MES-32.
"""
import time, warnings, numpy as np
warnings.filterwarnings("ignore")
from ikpy.chain import Chain

URDF = "/ros2_ws/src/so101_description/urdf/so101.urdf"
chain = Chain.from_urdf_file(URDF, base_elements=["base_link"], active_links_mask=[False] + [True] * 5 + [False])
locked = Chain.from_urdf_file(URDF, base_elements=["base_link"], active_links_mask=[False] + [True] * 4 + [False, False])  # wrist_roll fixed
assert chain.links[-1].name == "gripper_frame_joint"  # ikpy follows the FIRST child joint in URDF order
fk = lambda q: chain.forward_kinematics(np.r_[0, q, 0])
PAN_X = 0.0388  # shoulder_pan axis position in base_link [MES-06]

def jaw_err_deg(T, n):
    return np.degrees(np.arccos(min(1.0, abs(T[:3, 0] @ n))))

def seeds(p):
    az = np.arctan2(p[1], p[0] - PAN_X)
    return [[-az, -0.3, 0.3, 1.2], [-az, 0.5, -0.5, 1.5], [-az, -1.0, 1.0, 0.5], [-az, 0.0, 0.5, -0.5], [-az, -1.5, 1.5, 1.0]]

def drop_ik(p, n):
    """Chosen method: wrist_roll locked at +-90 deg (jaws normal to the arm plane), position IK on 4 joints.
    Returns (q, jaw error in deg, all joint-frame transforms) of the solution with the smallest jaw error."""
    best = None
    for roll in (np.pi / 2, -np.pi / 2):
        for s in seeds(p):
            q = locked.inverse_kinematics(p, initial_position=np.r_[0, s, roll, 0])[1:6]
            F = chain.forward_kinematics(np.r_[0, q, 0], full_kinematics=True)
            if np.linalg.norm(F[-1][:3, 3] - p) < 1e-3:
                e = jaw_err_deg(F[-1], n)
                if best is None or e < best[1]:
                    best = (q, e, F)
    return best

print("== MES-29: jaw-axis residual and ikpy 'X' mode seed sensitivity")
for roll in (np.pi / 2, -np.pi / 2):
    T = fk(np.array([0, -0.3, 0.3, 1.2, roll]))
    print(f"  roll {np.degrees(roll):+.0f} deg: jaw axis {T[:3, 0].round(3)} -> {jaw_err_deg(T, np.array([0, 1.0, 0])):.1f} deg from the arm-plane normal")
p, n = np.array([0.25, 0.0, 0.09]), np.array([0, 1.0, 0])
for roll_seed in (0.0, -1.5):
    q = chain.inverse_kinematics(p, n, orientation_mode="X", initial_position=np.r_[0, 0, -0.3, 0.3, 1.2, roll_seed, 0])[1:6]
    T = fk(q)
    print(f"  ikpy orientation_mode='X' at {p}, roll seed {roll_seed:+.1f}: position error {np.linalg.norm(T[:3, 3] - p) * 1e3:.1f} mm, jaw error {jaw_err_deg(T, n):.1f} deg")

print("\n== MES-27 A: grid RADIAL (plane contains the pan axis), method drop_ik; jaw error in deg, '-' = unreachable")
R = np.round(np.arange(0.08, 0.37, 0.02), 2); Z = np.round(np.arange(0.34, 0.05, -0.04), 2)
print("    z\\r " + " ".join(f"{r:.2f}".rjust(5) for r in R))
tms = []
for z in Z:
    row = []
    for r in R:
        t0 = time.perf_counter(); b = drop_ik(np.array([r, 0.0, z]), np.array([0, 1.0, 0])); tms.append(time.perf_counter() - t0)
        row.append("-" if b is None else f"{b[1]:.0f}")
    print(f"   {z:.2f} " + " ".join(c.rjust(5) for c in row))
print(f"  time per drop pose (2 rolls x 5 seeds): median {np.median(tms) * 1e3:.0f} ms, max {max(tms) * 1e3:.0f} ms")

print("\n== MES-29: ikpy orientation_mode='X' (roll free) vs drop_ik on the same radial targets (success = < 1 mm and < 5 deg)")
n, ok_x, ok_l, t_x = np.array([0, 1.0, 0]), [], [], []
for z in Z:
    for r in R:
        p = np.array([r, 0.0, z]); b = drop_ik(p, n); ok_l.append(b is not None and b[1] < 5)
        t0 = time.perf_counter(); hit = False
        for s in seeds(p):
            for roll in (0.0, np.pi / 2, -np.pi / 2):
                q = chain.inverse_kinematics(p, n, orientation_mode="X", initial_position=np.r_[0, s, roll, 0])[1:6]
                T = fk(q)
                if np.linalg.norm(T[:3, 3] - p) < 1e-3 and jaw_err_deg(T, n) < 5:
                    hit = True; break
            if hit: break
        t_x.append(time.perf_counter() - t0); ok_x.append(hit)
print(f"  {len(ok_l)} targets: drop_ik {np.mean(ok_l) * 100:.0f} % | ikpy 'X' mode, up to 15 seeds {np.mean(ok_x) * 100:.0f} % (median {np.median(t_x) * 1e3:.0f} ms)")

print("\n== MES-27 B: grid FACING the robot at x = 0.22 m; solver-independent: wrist_roll scanned every 10 deg,")
print("   position IK on the 4 other joints, smallest jaw error kept ('X' mode is unreliable, see above); '-' = unreachable")
Y = np.round(np.arange(-0.12, 0.13, 0.06), 2); Z2 = (0.30, 0.22, 0.14, 0.10, 0.06); n = np.array([1.0, 0, 0])
print("    z\\y " + " ".join(f"{y:+.2f}".rjust(5) for y in Y))
for z in Z2:
    row = []
    for y in Y:
        p = np.array([0.22, y, z]); best = None
        for roll in np.radians(np.arange(-180, 180, 10)):
            for s in seeds(p)[:3]:
                q = locked.inverse_kinematics(p, initial_position=np.r_[0, s, roll, 0])[1:6]
                T = fk(q)
                if np.linalg.norm(T[:3, 3] - p) < 1e-3:
                    best = jaw_err_deg(T, n) if best is None else min(best, jaw_err_deg(T, n))
        row.append("-" if best is None else f"{best:.0f}")
    print(f"   {z:.2f} " + " ".join(c.rjust(5) for c in row))

print("\n== MES-28: arm clearance above a radial grid (segments between joint origins, link thickness NOT included)")
for name, z_drop, cols in [("small grid, slot entries at ~15 cm", 0.18, np.linspace(0.16, 0.30, 7)),
                           ("standard grid, slot entries at ~25 cm", 0.28, np.linspace(0.12, 0.34, 7)),
                           ("MES-32: standard grid, arm on an 8 cm riser, columns 2 cm further", 0.20, np.linspace(0.14, 0.36, 7))]:
    top = z_drop - 0.03  # the TCP holds the token ~3 cm above the slot entry
    print(f"  {name}: TCP z = {z_drop} m, grid top = {top:.2f} m")
    for r in cols:
        best = None
        for roll in (np.pi / 2, -np.pi / 2):
            for s in seeds(np.array([r, 0.0, z_drop])):
                q = locked.inverse_kinematics(np.array([r, 0.0, z_drop]), initial_position=np.r_[0, s, roll, 0])[1:6]
                F = chain.forward_kinematics(np.r_[0, q, 0], full_kinematics=True)
                if np.linalg.norm(F[-1][:3, 3] - [r, 0.0, z_drop]) > 1e-3:
                    continue
                pts = np.array([f[:3, 3] for f in F[2:-1]])  # shoulder_lift .. wrist_roll joint origins
                seg = np.vstack([a + t * (b - a) for a, b in zip(pts[:-1], pts[1:]) for t in np.linspace(0, 1, 20)])
                over = seg[(seg[:, 0] > cols[0] - 0.02) & (seg[:, 0] < cols[-1] + 0.02)]
                clear = over[:, 2].min() - top if len(over) else np.inf
                if best is None or clear > best:
                    best = clear
        print(f"    column r = {r:.3f} m: " + ("unreachable" if best is None else ("arm never above the grid" if np.isinf(best) else f"clearance {best * 100:+.1f} cm")))

print("\n== MES-31: size of a grid cell in the image (pinhole projection, MuJoCo camera convention: looks along -z)")
AZ, PITCH = np.radians(-45), 0.024  # grid azimuth seen from the pan axis (faces the default camera), cell pitch
u_r = np.array([np.cos(AZ), np.sin(AZ), 0.0])  # along the columns
normal = np.array([-np.sin(AZ), np.cos(AZ), 0.0])  # towards the camera side
cells = [np.array([PAN_X, 0, 0]) + (0.16 + c * PITCH) * u_r + np.array([0, 0, 0.012 + k * PITCH]) for c in range(7) for k in range(6)]
center = np.mean(cells, axis=0)

def cell_px(pos, x_axis, y_axis, fovy_deg, w, h):
    x_axis, y_axis = np.asarray(x_axis) / np.linalg.norm(x_axis), np.asarray(y_axis) / np.linalg.norm(y_axis)
    z_axis = np.cross(x_axis, y_axis); f = (h / 2) / np.tan(np.radians(fovy_deg) / 2)
    uv = []
    for p in cells:
        d = p - pos; xc, yc, zc = d @ x_axis, d @ y_axis, d @ z_axis
        uv.append((w / 2 + f * xc / -zc, h / 2 - f * yc / -zc))
    uv = np.array(uv).reshape(7, 6, 2)
    inside = (uv[..., 0].min() >= 0) and (uv[..., 0].max() <= w) and (uv[..., 1].min() >= 0) and (uv[..., 1].max() <= h)
    return np.linalg.norm(np.diff(uv, axis=0), axis=-1).mean(), np.linalg.norm(np.diff(uv, axis=1), axis=-1).mean(), inside

look = lambda pos: (np.cross(center - pos, [0, 0, 1.0]), np.cross(np.cross(center - pos, [0, 0, 1.0]), center - pos))
for name, pos, axes in [("default external_cam (0.45, 0.45, 0.55)", np.array([0.45, 0.45, 0.55]), ([-0.707, 0.707, 0], [-0.354, -0.354, 0.866])),
                        ("proposed: 0.45 m in front of the grid face, 5 cm above its center", center + 0.45 * normal + [0, 0, 0.05], None)]:
    for w, h in [(320, 240), (640, 480)]:
        ax = axes or look(pos)
        cw, ch, inside = cell_px(pos, ax[0], ax[1], 52, w, h)
        print(f"  {name}, {w}x{h}: cell {cw:.1f} x {ch:.1f} px, whole grid in view: {inside}")
