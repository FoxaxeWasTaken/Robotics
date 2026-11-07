"""Q3: reader without markers: the cells projected from the KNOWN placement while the grid really is elsewhere.
How far can it be before the first wrong cell? And which token hues does the render really give (disc 0.2 x cell)?"""
import numpy as np, cv2
from common import *
from occl import hold_token
from classify import read, HANDOVER, mask, RED_CUBE, YELLOW

def run(g, res, arm=HANDOVER):
    w = World(g, *res, cam=P1_CAM); rng = np.random.default_rng(11)
    T_cfg = np.linalg.inv(T_FILE) @ g.T
    print(f"az {g.az} {res[0]}x{res[1]}: (shift along, across, height mm, yaw deg) -> wrong cells / 420, token frac min, empty frac max")
    for dx, dy, dz, yaw in ((2, 2, 0, 0), (5, 5, 0, 1), (8, 0, 0, 0), (0, 8, 0, 0), (0, 0, 8, 0), (10, 10, 0, 2), (12, 0, 0, 0), (0, 0, 12, 0), (15, 15, 0, 3), (20, 0, 0, 0), (0, 0, 20, 0)):
        c, s = np.cos(np.radians(yaw)), np.sin(np.radians(yaw))
        T = g.T.copy(); T[:3, :3] = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ g.R; T[:3, 3] += g.R @ (np.array([dx, dy, dz]) * 1e-3)
        w.set_grid(T); bad, occ, emp = 0, [], []
        for k in range(10):
            b = random_board(rng, int(rng.integers(10, 43)))
            w.board(b, T=T, rng=rng); w.arm(arm); hold_token(w)
            out, fr, fy, *_ = read(w.render(), g, w.K, T_cfg)
            bad += (out != b).sum()
            occ.append(np.nanmin(np.where(b == 1, fr, np.where(b == 2, fy, np.nan)))); emp.append(np.nanmax(np.where(b == 0, np.maximum(fr, fy), np.nan)) if (b == 0).any() else 0)
        print(f"  ({dx:2d},{dy:2d},{dz:2d},{yaw}) -> {bad:3d}, {min(occ):.2f}, {max(emp):.2f}", flush=True)
    w.close()

def hues(lighting, name):
    g = Grid(az=-51, marker=0.03); w = World(g, 640, 480, cam=P1_CAM, lighting=lighting)
    rng = np.random.default_rng(5); b = random_board(rng, 42); w.board(b, rng=rng); w.arm(HANDOVER)
    rgb = w.render(); T = np.linalg.inv(T_FILE) @ g.T
    uv = project(np.array([[c * PX, TOKEN_T / 2, r * g.pz] for r in range(ROWS) for c in range(COLS)]), T, np.eye(4), w.K)[0].reshape(ROWS, COLS, 2)
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    for v, n in ((1, "red"), (2, "yellow")):
        A = np.vstack([hsv[m].reshape(-1, 3) for m in disc_masks(uv[b == v], 5, rgb.shape)])
        H = (A[:, 0].astype(int) + 90) % 180 - 90
        print(f"{name} {n}: H {np.percentile(H,1):.0f}..{np.median(H):.0f}..{np.percentile(H,99):.0f}  S {A[:,1].min()}..{np.median(A[:,1]):.0f}  V {A[:,2].min()}..{np.median(A[:,2]):.0f}")
    w.close()

hues(None, "default"); hues(dict(scale=0.4, tint=[1.0, 0.8, 0.55]), "dim    ")
run(Grid(az=-51, marker=0.03), (320, 240))
