"""The recommended setup, end to end: phase-1 camera and extrinsics file untouched, lowered table, grid azimuth -40,
30 mm markers, 640x480, shadow map 1024, no floor reflection, tokens parked near the origin. 50 boards, arm at the
HANDOVER candidate with a token in the jaws; then the pose against the scene; the debug image the node would publish."""
import numpy as np, cv2, mujoco
from common import *
from occl import hold_token
from classify import read, HANDOVER, experiment
from pose_acc import run

KW = dict(shadow=1024, reflect=0.0)
g = Grid(az=-40, marker=0.03)

def exp(n=50, seed=21):
    w = World(g, 640, 480, cam=P1_CAM, **KW); rng = np.random.default_rng(seed)
    print("extent", round(w.m.stat.extent, 2), "base geoms x max?", flush=True)
    wrong = cells = 0; occ, emp = [], []; first = None
    for k in range(n):
        b = random_board(rng, int(rng.integers(0, 43)) if k else 23)
        w.board(b, rng=rng); w.arm(HANDOVER); hold_token(w)
        rgb = w.render(); r = pnp(detect(rgb), g, w.K); assert r and len(r[2]) == 4, k
        out, fr, fy, hsv, uv, rad = read(rgb, g, w.K, r[0], thr=0.35)
        wrong += (out != b).sum(); cells += 42
        if b.any():
            occ.append(np.nanmin(np.where(b == 1, fr, np.where(b == 2, fy, np.nan))))
        emp.append(np.nanmax(np.where(b == 0, np.maximum(fr, fy), np.nan)) if (b == 0).any() else 0)
        if first is None:
            first = (rgb.copy(), out.copy(), uv.copy(), rad, r[0].copy())
            cv2.imwrite("/out/recommended_640.png", rgb[..., ::-1]); np.save("/out/recommended_T.npy", r[0]); np.save("/out/recommended_K.npy", w.K)
            small = cv2.resize(rgb, (320, 240), interpolation=cv2.INTER_AREA); cv2.imwrite("/out/recommended_320.png", small[..., ::-1])
    print(f"recommended 640x480: {cells - wrong}/{cells} cells on {n} boards; token frac min {min(occ):.2f}, empty frac max {max(emp):.2f}", flush=True)
    rgb, out, uv, rad, T = first
    dbg = rgb.copy()
    cz = [-g.pz / 2 - g.bottom, g.top - g.bottom - g.pz / 2]; cx = [-PX / 2 - g.m, 6.5 * PX + g.m]
    y = GAP / 2 + WALL
    q, _ = project(np.array([[cx[0], y, cz[0]], [cx[1], y, cz[0]], [cx[1], y, cz[1]], [cx[0], y, cz[1]]]), T, np.eye(4), w.K)
    cv2.polylines(dbg, [q.astype(np.int32)], True, (0, 255, 0), 1, cv2.LINE_AA)
    for r_ in range(ROWS):
        for c in range(COLS):
            a = tuple(np.round(uv[r_, c]).astype(int)); v = out[r_, c]
            if v:
                cv2.circle(dbg, a, int(rad), {1: (255, 0, 0), 2: (255, 255, 0)}[v], -1, cv2.LINE_AA)
            cv2.circle(dbg, a, int(rad), (255, 255, 255), 1, cv2.LINE_AA)
    a = np.round(uv[ROWS - 1, 3]).astype(int)  # chosen column: an arrow above it
    cv2.arrowedLine(dbg, (a[0], a[1] - 32), (a[0], a[1] - 12), (0, 255, 0), 2, cv2.LINE_AA, tipLength=0.4)
    for col, th in (((0, 0, 0), 3), ((0, 255, 0), 1)):
        cv2.putText(dbg, "board VALID 5/5  human to play  robot: col 3", (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, th, cv2.LINE_AA)
    cv2.imwrite("/out/recommended_debug_640.png", dbg[..., ::-1])
    w.close()

exp()
run("recommended", g, P1_CAM, (640, 480), True, **KW)
