"""Q3: random valid boards, rendered with the prototype scene, simplest classifier: a disc per cell, fraction of
pixels in the color's HSV ranges (rgb8 -> COLOR_RGB2HSV). The ranges reuse so101_perception.detection.hsv_ranges'
format; the code is copied here because the image has no built workspace."""
import sys, json, numpy as np, cv2
from common import *
from occl import hold_token

def hsv_ranges(h_min, h_max, s_min, v_min, v_max=255):
    if h_min <= h_max:
        return (((h_min, s_min, v_min), (h_max, 255, v_max)),)
    return (((h_min, s_min, v_min), (180, 255, v_max)), ((0, s_min, v_min), (h_max, 255, v_max)))

RED_CUBE = hsv_ranges(160, 8, 150, 30)          # perception.yaml's cube ranges, unchanged
YELLOW = hsv_ranges(18, 40, 120, 30)
HANDOVER = [-1.0, -0.6, 0.6, 1.1, 1.57]

def mask(hsv, ranges):
    m = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in ranges:
        m |= cv2.inRange(hsv, lo, hi)
    return m > 0

def read(rgb, g, K, T_cam_grid, rad_scale=0.3, thr=0.5, red=RED_CUBE, yel=YELLOW):
    pts = np.array([[c * PX, TOKEN_T / 2, r * g.pz] for r in range(ROWS) for c in range(COLS)])
    uv, z = project(pts, T_cam_grid, np.eye(4), K)
    rad = rad_scale * PX * K[0, 0] / z.mean()
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV); R, Y = mask(hsv, red), mask(hsv, yel)
    fr, fy = np.zeros(42), np.zeros(42)
    for i, m in enumerate(disc_masks(uv, rad, rgb.shape)):
        fr[i], fy[i] = R[m].mean(), Y[m].mean()
    out = np.where(fr >= thr, 1, np.where(fy >= thr, 2, 0)).reshape(ROWS, COLS)
    return out, fr.reshape(ROWS, COLS), fy.reshape(ROWS, COLS), hsv, uv.reshape(ROWS, COLS, 2), rad

def experiment(label, g, res, n=30, seed=7, source="markers", lighting=None, pz_stack=None, cam=P1_CAM, arm=HANDOVER, save=None, jitter=True):
    w = World(g, *res, cam=cam, lighting=lighting); w.arm(arm); rng = np.random.default_rng(seed)
    T_cfg = np.linalg.inv(T_FILE) @ g.T   # known placement + our extrinsics
    wrong, cells, bad_boards, nomk = 0, 0, 0, 0
    margins = dict(occ=[], emp=[]); hs = {1: [], 2: []}
    for k in range(n):
        b = random_board(rng, 42 if k == 0 else int(rng.integers(4, 43)))
        w.board(b, rng=rng if jitter else None, pz_stack=pz_stack); w.arm(arm); hold_token(w)
        rgb = w.render()
        if save and k == 0:
            cv2.imwrite(save, rgb[..., ::-1])
        T = T_cfg
        if source == "markers":
            r = pnp(detect(rgb), g, w.K)
            if r is None or len(r[2]) < 4:
                nomk += 1; continue
            T = r[0]
        out, fr, fy, hsv, uv, rad = read(rgb, g, w.K, T)
        e = (out != b).sum(); wrong += e; cells += 42; bad_boards += e > 0
        occ = np.where(b == 1, fr, np.where(b == 2, fy, np.nan)); emp = np.where(b == 0, np.maximum(fr, fy), np.nan)
        margins["occ"].append(np.nanmin(occ)); margins["emp"].append(np.nanmax(emp) if (b == 0).any() else 0)
        for v in (1, 2):
            for ms in disc_masks(uv[b == v], rad, rgb.shape):
                hs[v].append(hsv[ms].reshape(-1, 3))
    w.close()
    txt = f"{label:44s} {res[0]}x{res[1]} src={source}: {cells-wrong}/{cells} cells, {bad_boards} bad boards of {n-nomk} (no 4 markers: {nomk}); token frac min {min(margins['occ']):.2f}, empty frac max {max(margins['emp']):.2f}"
    for v, name in ((1, "red"), (2, "yel")):
        A = np.vstack(hs[v]); H = (A[:, 0].astype(int) + 90) % 180 - 90 if v == 1 else A[:, 0]
        txt += f"\n      {name}: H p1-p99 {np.percentile(H,1):.0f}..{np.percentile(H,99):.0f}, S p1 {np.percentile(A[:,1],1):.0f}, V p1 {np.percentile(A[:,2],1):.0f}, n={len(A)}"
    print(txt, flush=True)

if __name__ == "__main__":
    DIM = dict(scale=0.4, tint=[1.0, 0.8, 0.55])
    for az in (-51, -40):
        g = Grid(az=az, marker=0.03)
        experiment(f"az {az}, markers 30 mm", g, (640, 480), save=f"/out/board_az{-az}_640.png")
        experiment(f"az {az}, known placement", g, (320, 240), source="config", save=f"/out/board_az{-az}_320.png")
        experiment(f"az {az}, known placement", g, (640, 480), source="config")
        experiment(f"az {az}, dim light", g, (640, 480), lighting=DIM)
        experiment(f"az {az}, dim light, known placement", g, (320, 240), source="config", lighting=DIM)
    # the model's trap: holes every 37 mm vertically, tokens stacking every 34 mm
    g = Grid(az=-51, marker=0.03, pz=0.037)
    experiment("holes 37 mm, tokens stack 34 mm", g, (640, 480), pz_stack=0.034, save="/out/board_pz37_640.png")
