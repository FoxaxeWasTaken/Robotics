"""Vision expert checks: FOV coverage, HSV detection, pixel->3D error, yaw estimate, ArUco extrinsic calibration."""
import shutil, time
import numpy as np, mujoco, cv2
from so101_sim import SO101Sim, ARM_JOINTS

np.set_printoptions(precision=4, suppress=True)
FLIP = np.diag([1.0, -1.0, -1.0])           # MuJoCo camera (x right, y up, looks -z) -> OpenCV (x right, y down, looks +z)
PARK = [1.9, -1.7, 1.6, 0.7, 0.0]           # folded, pan turned away from the workspace

def cam_cv(sim):
    T = sim.get_camera_extrinsics(); return T[:3, :3] @ FLIP, T[:3, 3]    # R_world_cam(cv), t

def project(K, R, t, P):
    pc = R.T @ (np.asarray(P) - t); uv = K @ pc; return uv[:2] / uv[2]

def ray_plane(K, R, t, u, v, z):
    d = R @ np.linalg.solve(K, [u, v, 1.0]); s = (z - t[2]) / d[2]; return t + s * d

def red_mask(rgb):
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    m = cv2.inRange(hsv, (160, 100, 60), (180, 255, 255)) | cv2.inRange(hsv, (0, 100, 60), (8, 255, 255))
    return cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

def brown_mask(rgb):
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    return cv2.morphologyEx(cv2.inRange(hsv, (5, 90, 40), (25, 255, 220)), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

def largest(mask):
    cs, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    return max(cs, key=cv2.contourArea) if cs else None

sim = SO101Sim(seed=0); sim.connect(); m, d = sim.model, sim.data
qadr = [m.joint(n).qposadr[0] for n in ARM_JOINTS]
K = sim.get_camera_intrinsics(); R, t = cam_cv(sim); W, H = sim.camera_width, sim.camera_height
print("[cam] position", t, "| optical axis (world)", R[:, 2])

# ---- 1. FOV coverage of the randomization region (cube corners) ----
rng = np.random.default_rng(0); inside = partial = 0; N = 2000
corners = np.array([[sx, sy, sz] for sx in (-.025, .025) for sy in (-.025, .025) for sz in (0, .05)])
for _ in range(N):
    r, a = rng.uniform(0.18, 0.30), np.radians(rng.uniform(-50, 60))
    c = np.array([r * np.cos(a), r * np.sin(a), 0.0])
    uv = np.array([project(K, R, t, c + k) for k in corners])
    ok = (uv[:, 0] >= 0) & (uv[:, 0] < W) & (uv[:, 1] >= 0) & (uv[:, 1] < H)
    inside += ok.all(); partial += ok.any() and not ok.all()
print(f"[fov] cube fully visible {inside/N*100:.0f}% | truncated {partial/N*100:.0f}% | out {100-(inside+partial)/N*100:.0f}%  (320x240)")

# ---- 2/3. detection + pixel->3D + yaw ----
errs, errs_c, yaw_errs, trunc, miss, times = [], [], [], 0, 0, []
box_errs = []
for ep in range(80):
    sim._rng = np.random.default_rng(1000 + ep); d.qpos[qadr] = PARK; d.ctrl[qadr] = PARK; sim.randomize_object()
    img = sim.render_camera(); gt = sim.get_object_position()
    qa = m.joint("pickup_object_free").qposadr[0]; gt_yaw = 2 * np.arctan2(d.qpos[qa + 6], d.qpos[qa + 3])
    t0 = time.perf_counter(); cnt = largest(red_mask(img))
    if cnt is None or cv2.contourArea(cnt) < 15: miss += 1; continue
    x, y, w, h = cv2.boundingRect(cnt)
    if x <= 0 or y <= 0 or x + w >= W or y + h >= H: trunc += 1; continue
    mo = cv2.moments(cnt); u, v = mo["m10"] / mo["m00"], mo["m01"] / mo["m00"]
    p = ray_plane(K, R, t, u, v, 0.025); times.append(time.perf_counter() - t0)
    errs.append(np.linalg.norm(p[:2] - gt[:2]))
    # bias-corrected: intersect at z where the visible-surface centroid lies (~upper part), estimated empirically below
    errs_c.append(p[:2] - gt[:2])
    # yaw: contour back-projected on the top-face plane, minAreaRect in metric XY
    pts = np.array([ray_plane(K, R, t, cu, cv, 0.05)[:2] for cu, cv in cnt[:, 0, :]], np.float32)
    (_, _), (_, _), ang = cv2.minAreaRect(pts)
    e = (np.radians(ang) - gt_yaw + np.pi / 4) % (np.pi / 2) - np.pi / 4; yaw_errs.append(abs(np.degrees(e)))
    bc = largest(brown_mask(img))
    if bc is not None:
        mb = cv2.moments(bc); pb = ray_plane(K, R, t, mb["m10"] / mb["m00"], mb["m01"] / mb["m00"], 0.025)
        box_errs.append(np.linalg.norm(pb[:2] - sim.get_drop_box_position()[:2]))
e = np.array(errs); bias = np.mean(errs_c, axis=0)
print(f"[detect] {len(errs)} ok, {trunc} truncated (touching border), {miss} missed | {np.median(times)*1e3:.2f} ms/frame")
print(f"[3d] ray-plane z=2.5cm, GT extrinsics: mean {e.mean()*1e3:.1f} mm, p95 {np.percentile(e,95)*1e3:.1f} mm, max {e.max()*1e3:.1f} mm | mean bias xy {bias*1e3} mm")
ec = np.linalg.norm(np.array(errs_c) - bias, axis=1)
print(f"[3d] after constant bias removal: mean {ec.mean()*1e3:.1f} mm, max {ec.max()*1e3:.1f} mm")
print(f"[yaw] minAreaRect on top plane: median err {np.median(yaw_errs):.1f} deg, p90 {np.percentile(yaw_errs,90):.1f} deg")
print(f"[box] brown blob centroid @z=2.5cm: mean {np.mean(box_errs)*1e3:.1f} mm, max {np.max(box_errs)*1e3:.1f} mm (n={len(box_errs)})")
img = sim.render_camera(); vis = img.copy(); vis[red_mask(img) > 0] = (0, 255, 0); vis[brown_mask(img) > 0] = (0, 0, 255)
cv2.imwrite("/out/percep_masks.png", cv2.cvtColor(np.hstack([img, vis]), cv2.COLOR_RGB2BGR))

# ---- 4. ArUco extrinsic calibration in a scene copy with 4 markers on the table ----
src = "/opt/so101/sim/so101_sim/assets/so101"; dst = "/tmp/so101_calib"; shutil.copytree(src, dst)
D = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
S_MARK, HALF = 0.075, 0.05                              # black square side, plane half-size (white margin around)
cands = []
for gx in np.arange(-0.6, 0.61, 0.05):
    for gy in np.arange(-0.6, 0.61, 0.05):
        c = np.array([gx, gy]); rr = np.hypot(gx, gy); aa = np.degrees(np.arctan2(gy, gx))
        if rr < 0.17 or np.linalg.norm(c - [0.05, -0.25]) < 0.13: continue
        if 0.12 < rr < 0.37 and -65 < aa < 75: continue           # keep the pick region free
        uv = np.array([project(K, R, t, [gx + sx, gy + sy, 0]) for sx in (-HALF, HALF) for sy in (-HALF, HALF)])
        if (uv[:, 0] > 8).all() and (uv[:, 0] < W - 8).all() and (uv[:, 1] > 8).all() and (uv[:, 1] < H - 8).all():
            cands.append(c)
sel = [cands[0]]
while len(sel) < 6:
    sel.append(max(cands, key=lambda c: min(np.linalg.norm(c - s_) for s_ in sel)))
MARKERS = {i: tuple(np.round(c, 3)) for i, c in enumerate(sel)}
print("[aruco] auto-selected marker centres (visible, spread):", MARKERS)
assets, bodies = [], []
for i, (mx, my) in MARKERS.items():
    px = 600; img_m = cv2.aruco.generateImageMarker(D, i, px)
    margin = int(px * (2 * HALF - S_MARK) / S_MARK / 2); img_m = cv2.copyMakeBorder(img_m, *(margin,) * 4, cv2.BORDER_CONSTANT, value=255)
    cv2.imwrite(f"{dst}/aruco{i}.png", cv2.cvtColor(img_m, cv2.COLOR_GRAY2BGR))
    assets.append(f'<texture name="aruco{i}" type="2d" file="aruco{i}.png"/><material name="aruco{i}" texture="aruco{i}"/>')
    bodies.append(f'<geom type="plane" size="{HALF} {HALF} 0.001" pos="{mx} {my} 0.0005" material="aruco{i}" contype="0" conaffinity="0"/>')
open(f"{dst}/scene_calib.xml", "w").write(f'<mujoco><include file="scene_tek5.xml"/><asset>{"".join(assets)}</asset><worldbody>{"".join(bodies)}</worldbody></mujoco>')
for res in [(320, 240), (640, 480)]:
    cs = SO101Sim(scene_path=f"{dst}/scene_calib.xml", camera_width=res[0], camera_height=res[1], seed=0); cs.connect()
    cs.data.qpos[[cs.model.joint(n).qposadr[0] for n in ARM_JOINTS]] = PARK; mujoco.mj_forward(cs.model, cs.data)
    im = cs.render_camera(); gray = cv2.cvtColor(im, cv2.COLOR_RGB2GRAY)
    det = cv2.aruco.ArucoDetector(D, cv2.aruco.DetectorParameters())
    corners_, ids, _ = det.detectMarkers(gray)
    Kc = cs.get_camera_intrinsics(); Rg, tg = cam_cv(cs)
    if ids is None: print(f"[aruco {res}] no marker detected"); continue
    # texture orientation on the plane: pick the corner ordering (8 symmetries) consistent with GT, once (convention check)
    h = S_MARK / 2; base = np.array([[-h, h], [h, h], [h, -h], [-h, -h]])
    def sym(k, flip): b = base[:, ::-1] * [1, 1] if flip else base; return np.roll(b, k, axis=0)
    best = None
    for k in range(4):
        for flip in (False, True):
            errp = []
            for c, i in zip(corners_, ids.ravel()):
                obj = np.c_[sym(k, flip) + MARKERS[i], np.zeros(4)]
                errp.append(np.linalg.norm(np.array([project(Kc, Rg, tg, P) for P in obj]) - c[0], axis=1).mean())
            if best is None or np.mean(errp) < best[0]: best = (np.mean(errp), k, flip)
    _, k, flip = best
    obj_pts = np.vstack([np.c_[sym(k, flip) + MARKERS[i], np.zeros(4)] for i in ids.ravel()]).astype(np.float64)
    img_pts = np.vstack([c[0] for c in corners_]).astype(np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, Kc, None, flags=cv2.SOLVEPNP_SQPNP)
    rvec, tvec = cv2.solvePnPRefineLM(obj_pts, img_pts, Kc, None, rvec, tvec)
    Rcw, _ = cv2.Rodrigues(rvec); R_est = Rcw.T; t_est = (-Rcw.T @ tvec).ravel()
    reproj = np.linalg.norm(cv2.projectPoints(obj_pts, rvec, tvec, Kc, None)[0][:, 0] - img_pts, axis=1).mean()
    dpos = np.linalg.norm(t_est - tg); drot = np.degrees(np.linalg.norm(cv2.Rodrigues(R_est.T @ Rg)[0]))
    # downstream impact: object localisation error using estimated vs GT extrinsics at 20 random points on the region
    imp = []
    for _ in range(200):
        r_, a_ = rng.uniform(0.18, 0.30), np.radians(rng.uniform(-50, 60)); P = np.array([r_ * np.cos(a_), r_ * np.sin(a_), 0.025])
        uv = project(Kc, Rg, tg, P); imp.append(np.linalg.norm(ray_plane(Kc, R_est, t_est, *uv, 0.025) - P))
    print(f"[aruco {res[0]}x{res[1]}] detected ids {sorted(ids.ravel().tolist())} | reproj {reproj:.2f}px | cam pose err {dpos*1e3:.1f} mm, {drot:.2f} deg | induced object err mean {np.mean(imp)*1e3:.1f} mm, max {np.max(imp)*1e3:.1f} mm")
    if res == (640, 480):
        vis = im.copy(); cv2.aruco.drawDetectedMarkers(vis, corners_, ids); cv2.imwrite("/out/aruco_view.png", cv2.cvtColor(vis, cv2.COLOR_RGB2BGR))
