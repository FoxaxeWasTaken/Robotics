"""Q4: /connect4/grid_pose against the scene (scoring only). Grid body moved +-5 mm, +-1.5 deg yaw around its
nominal pose, random boards; marker corners -> SQPNP + LM (phase-1 solver) -> base_link through the extrinsics."""
import numpy as np, cv2
from scipy.optimize import least_squares
from common import *

def face_cam(g, d, e):
    c = g.t + g.R @ np.array([3 * PX, 0, 2.5 * g.pz]); a = np.radians(e)
    pos = c + d * np.cos(a) * g.y + d * np.sin(a) * g.z
    x, y = look_at(pos, c)
    return {"pos": pos.tolist(), "xyaxes": [*x.tolist(), *y.tolist()]}

def rigid_err(T_true):  # same size as the calibration error MES-47, for a camera with no calibration file yet
    d = np.eye(4); rv = np.radians(0.024) * np.array([0.6, 0.6, 0.53]); d[:3, :3] = cv2.Rodrigues(rv)[0]; d[:3, 3] = [0.0003, -0.0003, 0.0003]
    return T_true @ d

def constrained(found, g, K, T_bc, T0):
    """(x, y, yaw) only: the grid stands on a known table, vertical. Reprojection LSQ."""
    ids = [i for i in sorted(found) if i in g.markers()]
    local = np.vstack([g.marker_corners(i) for i in ids]); im = np.vstack([found[i] for i in ids])
    def T_of(p):
        c, s = np.cos(p[2]), np.sin(p[2]); T = T0.copy()
        T[:3, :3] = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ T0[:3, :3]; T[:2, 3] += p[:2]
        return T
    f = lambda p: (project(local, T_of(p), T_bc, K)[0] - im).ravel()
    return T_of(least_squares(f, np.zeros(3)).x)

def run(label, g, cam, res, cal_file, n=30, seed=3, **kw):
    w = World(g, *res, cam=cam, **kw); w.arm(PARK); rng = np.random.default_rng(seed)
    T_bc = T_FILE if cal_file else rigid_err(w.T_true())
    tops = np.array([[c * PX, 0, g.top - g.bottom - TOKEN_D / 2] for c in range(COLS)])
    E, C, miss, rep = [], [], 0, []
    for k in range(n):
        d = rng.uniform([-0.005, -0.005, -1.5], [0.005, 0.005, 1.5])
        c, s = np.cos(np.radians(d[2])), np.sin(np.radians(d[2]))
        T = g.T.copy(); T[:3, :3] = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ g.R; T[:2, 3] += d[:2]
        w.set_grid(T); w.board(random_board(rng, int(rng.integers(0, 43))), T=T, rng=rng)
        found = detect(w.render()); r = pnp(found, g, w.K)
        if r is None or len(r[2]) < 4:
            miss += 1; continue
        T_est = T_bc @ r[0]; rep.append(r[1])
        for store, Te in ((E, T_est), (C, constrained(found, g, w.K, T_bc, g.T))):
            dp = (np.linalg.inv(T) @ Te @ np.c_[tops, np.ones(7)].T).T[:, :3] - tops  # slot entries, error in the true grid frame
            dR = T[:3, :3].T @ Te[:3, :3]; rv = cv2.Rodrigues(dR)[0].ravel()
            store.append((np.abs(dp[:, 0]).max(), np.abs(dp[:, 1]).max(), np.abs(dp[:, 2]).max(), np.degrees(np.linalg.norm(rv))))
    w.close()
    def line(tag, A):
        A = np.array(A) * [1e3, 1e3, 1e3, 1]
        return f"{tag}: along {A[:,0].mean():.2f}/{A[:,0].max():.2f} mm, ACROSS {A[:,1].mean():.2f}/{A[:,1].max():.2f} mm, z {A[:,2].mean():.2f}/{A[:,2].max():.2f} mm, rot {A[:,3].mean():.3f}/{A[:,3].max():.3f} deg"
    print(f"{label} {res[0]}x{res[1]} marker {g.marker*1e3:.0f} az {g.az}: {n-miss}/{n} frames with 4 markers, reproj {np.mean(rep) if rep else float('nan'):.2f} px")
    if E:
        print("   6-DoF PnP  (mean/max at the 7 slot entries) " + line("", E)); print("   3-DoF fit  (table, vertical known)           " + line("", C))

if __name__ == "__main__":
  P1 = P1_CAM
  for az, mk in ((-51, .03), (-51, .04), (-40, .03), (-40, .04)):
      g = Grid(az=az, marker=mk, margin=max(.045, mk * 1.3 + .006)); run("P1 cam", g, P1, (640, 480), True)
  g = Grid(az=-51, marker=.06, margin=.084); run("P1 cam", g, P1, (320, 240), True)
  g = Grid(az=-51, marker=.04, margin=.058); cam = face_cam(g, .50, 20)
  run("moved d=0.5 e=20", g, cam, (320, 240), False); run("moved d=0.5 e=20", g, cam, (640, 480), False)
