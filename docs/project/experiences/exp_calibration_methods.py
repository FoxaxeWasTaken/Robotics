"""Extrinsic calibration: 1 ArUco (the demo method) vs several, with detection noise (640x480)."""
import shutil
import numpy as np, mujoco, cv2
from so101_sim import SO101Sim, ARM_JOINTS

FLIP = np.diag([1.0, -1.0, -1.0]); PARK = [1.9, -1.7, 1.6, 0.7, 0.0]
D = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
CONFIGS = {  # name: [(x, y, black_side_m), ...]
    "1 marker 7.5 cm (workspace)": [(0.24, 0.0, 0.075)],
    "1 marker 15 cm (workspace)": [(0.24, 0.0, 0.15)],
    "4 markers 7.5 cm (workspace corners)": [(0.17, 0.14, 0.075), (0.30, 0.10, 0.075), (0.30, -0.12, 0.075), (0.17, -0.16, 0.075)],
    "6 markers 7.5 cm (spread over the image)": [(-0.15, 0.35, 0.075), (0.25, -0.45, 0.075), (-0.6, 0.0, 0.075), (-0.2, -0.3, 0.075), (-0.05, -0.6, 0.075), (-0.6, -0.6, 0.075)],
}
rng = np.random.default_rng(0)
def project(K, R, t, P):
    pc = R.T @ (P - t); uv = K @ pc; return uv[:2] / uv[2]
def ray_plane(K, R, t, uv, z):
    d = R @ np.linalg.solve(K, [uv[0], uv[1], 1.0]); return t + (z - t[2]) / d[2] * d
src = "/opt/so101/sim/so101_sim/assets/so101"
for name, markers in CONFIGS.items():
    dst = "/tmp/calib_scene"; shutil.rmtree(dst, ignore_errors=True); shutil.copytree(src, dst)
    assets, geoms = [], []
    for i, (mx, my, side) in enumerate(markers):
        img = cv2.aruco.generateImageMarker(D, i, 600); m = int(600 * 0.15)  # 15 % white margin
        cv2.imwrite(f"{dst}/m{i}.png", cv2.cvtColor(cv2.copyMakeBorder(img, m, m, m, m, cv2.BORDER_CONSTANT, value=255), cv2.COLOR_GRAY2BGR))
        half = side / 2 * 1.3
        assets.append(f'<texture name="m{i}" type="2d" file="m{i}.png"/><material name="m{i}" texture="m{i}"/>')
        geoms.append(f'<geom type="plane" size="{half} {half} 0.001" pos="{mx} {my} 0.0005" material="m{i}" contype="0" conaffinity="0"/>')
    open(f"{dst}/scene_calib.xml", "w").write(f'<mujoco><include file="scene_tek5.xml"/><asset>{"".join(assets)}</asset><worldbody>{"".join(geoms)}</worldbody></mujoco>')
    sim = SO101Sim(scene_path=f"{dst}/scene_calib.xml", camera_width=640, camera_height=480, seed=3); sim.connect()
    qa = sim.model.joint("pickup_object_free").qposadr[0]; sim.data.qpos[qa:qa + 3] = [-0.5, 0.5, 0.025]  # cube out of the way
    sim.data.qpos[[sim.model.joint(n).qposadr[0] for n in ARM_JOINTS]] = PARK; mujoco.mj_forward(sim.model, sim.data)
    K = sim.get_camera_intrinsics(); T = sim.get_camera_extrinsics(); Rg, tg = T[:3, :3] @ FLIP, T[:3, 3]
    corners, ids, _ = cv2.aruco.ArucoDetector(D, cv2.aruco.DetectorParameters()).detectMarkers(cv2.cvtColor(sim.render_camera(), cv2.COLOR_RGB2GRAY))
    if ids is None: print(f"{name:45s} : no marker detected"); continue
    obj, img = [], []
    for c, i in zip(corners, ids.ravel()):
        mx, my, side = markers[i]; h = side / 2
        cand = np.array([[mx + sx, my + sy, 0.0] for sx, sy in [(-h, h), (h, h), (h, -h), (-h, -h)]])
        # corner order: keep the permutation consistent with ground truth (texture convention, fixed once)
        best = min((np.roll(cand[::s], k, axis=0) for s in (1, -1) for k in range(4)),
                   key=lambda P: np.linalg.norm(np.array([project(K, Rg, tg, p) for p in P]) - c[0]))
        obj.append(best); img.append(c[0])
    obj, img = np.vstack(obj), np.vstack(img).astype(np.float64)
    pose_e, rot_e, obj_e = [], [], []
    for trial in range(100):
        noisy = img + rng.normal(0, 0.5, img.shape)  # realistic detection noise (0.5 px)
        ok, rv, tv = cv2.solvePnP(obj, noisy, K, None, flags=cv2.SOLVEPNP_SQPNP)
        rv, tv = cv2.solvePnPRefineLM(obj, noisy, K, None, rv, tv)
        Rc, _ = cv2.Rodrigues(rv); Re, te = Rc.T, (-Rc.T @ tv).ravel()
        pose_e.append(np.linalg.norm(te - tg)); rot_e.append(np.degrees(np.linalg.norm(cv2.Rodrigues(Re.T @ Rg)[0])))
        for _ in range(20):
            r, a = rng.uniform(0.18, 0.30), np.radians(rng.uniform(-50, 60)); P = np.array([r * np.cos(a), r * np.sin(a), 0.025])
            obj_e.append(np.linalg.norm(ray_plane(K, Re, te, project(K, Rg, tg, P), 0.025) - P))
    print(f"{name:45s} : {len(ids)} detected | camera pose p95 {np.percentile(pose_e, 95)*1e3:6.1f} mm / {np.percentile(rot_e, 95):4.2f}° | object error mean {np.mean(obj_e)*1e3:5.1f} mm, p95 {np.percentile(obj_e, 95)*1e3:5.1f} mm", flush=True)
