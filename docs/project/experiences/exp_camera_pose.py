"""MES-45: where to put the sim camera so that every cube and box of /sim/randomize is in full view (#47).

The provided pose cuts 21 % of the cubes at the image edge [MES-12]. The teaching staff allows moving it
(2026-10-16), on a copy of the scene edited with mujoco.MjSpec, never in sim/ (ADR-036).

A. Projection: look-at poses on the work area, 30-40 degrees down (ADR-021), the provided fovy (52) at 320x240.
   2000 cubes (r 0.18-0.30 m, azimuth -50..60, any yaw) and 2000 boxes (r 0.18-0.30 m, azimuth -90..-10), all
   their corners. A pose passes when every corner is at least MARGIN px inside the image. Per sector around the
   work area (front, front-left, left, front-right, right), the best one shows the cube largest (accuracy grows
   with pixels per cm); front-left is also given at 35 and 40 degrees down.
B. Rendering of one pose (MuJoCo renders one model per process reliably: a later model in the same process
   gave blank detections): cubes cut by the image edge (any cube pixel on the border) over 300 /sim/randomize
   scenes; the cube and box pixels the parked arm hides over 50; and a cube lying in the box (16 boxes, 3
   offsets): where perception puts it, since VERIFY needs it inside the box footprint, or unseen.

In the control image:
    python3 exp_camera_pose.py           # A: every pose, by name
    python3 exp_camera_pose.py <name>    # B for that pose (provided, front, front-left, front-left-35, ...)
"""

import shutil
import sys
import tempfile
from pathlib import Path

import mujoco
import numpy as np
import yaml
from so101_sim import ARM_JOINTS, DEFAULT_SCENE, SO101Sim

for package in ("so101_common", "so101_driver", "so101_perception"):
    sys.path.insert(0, f"/ros2_ws/src/{package}")
from so101_driver.randomize import sample_box_xy  # noqa: E402
from so101_perception.detection import Thresholds, detect, hsv_ranges  # noqa: E402
from so101_perception.geometry import pixel_to_world  # noqa: E402

W, H, FOVY, MARGIN = 320, 240, 52.0, 10
PARK = [1.9, -1.7, 1.5, 0.7, 0.0]  # the brain's PARK (brain.yaml)
FLIP = np.diag([1.0, -1.0, -1.0])  # MuJoCo camera frame (y up, looks along -z) -> OpenCV optical frame
fy = (H / 2) / np.tan(np.radians(FOVY) / 2)
K = np.array([[fy, 0, (W - 1) / 2], [0, fy, (H - 1) / 2], [0, 0, 1]])  # OpenCV pixel centers [MES-41]
rng = np.random.default_rng(0)


def area(n, radius, azimuth_deg):
    r, a = rng.uniform(*radius, n), np.radians(rng.uniform(*azimuth_deg, n))
    return np.c_[r * np.cos(a), r * np.sin(a)]


def corners(centers, half, yaw):
    square = np.array([[1, 1], [1, -1], [-1, 1], [-1, -1]]) * half
    c, s = np.cos(yaw), np.sin(yaw)
    rot = np.stack([np.stack([c, -s], -1), np.stack([s, c], -1)], -2)  # (n, 2, 2)
    xy = centers[:, None, :] + np.einsum("nij,kj->nki", rot, square)
    return np.concatenate([np.c_[xy.reshape(-1, 2), np.full(xy.size // 2, z)] for z in (0.0, 0.05)])


cubes = corners(area(2000, (0.18, 0.30), (-50, 60)), 0.025, rng.uniform(0, np.pi / 2, 2000))
boxes = corners(area(2000, (0.18, 0.30), (-90, -10)), 0.05, np.zeros(2000))
points = np.r_[cubes, boxes]
target = (points.min(0) + points.max(0)) / 2


def look_at(position):
    """MuJoCo camera frame (x right, y up, looks along -z) and OpenCV rotation (x right, y down, looks +z)."""
    forward = (target - position) / np.linalg.norm(target - position)
    right = np.cross(forward, [0, 0, 1.0])
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    return np.c_[right, up, -forward], np.c_[right, -up, forward]


def margin_and_scale(position, R):
    pc = (points - position) @ R
    if (pc[:, 2] <= 0).any():
        return -np.inf, 0.0
    uv = (pc @ K.T)[:, :2] / pc[:, 2:]
    margin = min(uv[:, 0].min(), W - 1 - uv[:, 0].max(), uv[:, 1].min(), H - 1 - uv[:, 1].max())
    return margin, fy * 0.05 / np.mean(pc[: len(cubes), 2])  # mean cube edge, px


def closest(azimuth_deg, down_deg):
    """The closest pose of that direction that keeps every corner MARGIN px inside, or None."""
    a, d = np.radians(azimuth_deg), np.radians(down_deg)
    for distance in np.arange(0.35, 1.3, 0.01):
        position = target + distance * np.array([np.cos(d) * np.cos(a), np.cos(d) * np.sin(a), np.sin(d)])
        margin, scale = margin_and_scale(position, look_at(position)[1])
        if margin >= MARGIN:
            return scale, margin, position, distance
    return None


SECTORS = {"front": (-25, 25), "front-left": (25, 65), "left": (65, 115), "front-right": (-65, -25), "right": (-115, -65)}
poses = {}
for sector, (lo, hi) in SECTORS.items():
    candidates = [(closest(az, down), az, down) for az in range(lo, hi, 5) for down in (30, 32.5, 35, 37.5, 40)]
    poses[sector] = max((c for c in candidates if c[0] is not None), key=lambda c: c[0][0])
for down in (35, 40):
    poses[f"front-left-{down}"] = (closest(poses["front-left"][1], down), poses["front-left"][1], down)


def describe(name):
    (scale, margin, position, distance), azimuth, down = poses[name]
    R_mj = look_at(position)[0]
    xyaxes = " ".join(f"{v:.4f}" for v in np.r_[R_mj[:, 0], R_mj[:, 1]])
    return (f"{name}: pos {np.round(position, 3).tolist()} (azimuth {azimuth}, {down} down, {distance:.2f} m), "
            f"margin {margin:.1f} px, cube edge {scale:.1f} px; xyaxes {xyaxes}")


def scene(position, R_mj):
    folder = Path(tempfile.mkdtemp()) / "so101"
    shutil.copytree(Path(DEFAULT_SCENE).parent, folder)
    spec = mujoco.MjSpec.from_file(str(folder / Path(DEFAULT_SCENE).name))
    cam = spec.camera("external_cam")
    cam.pos = position
    cam.alt.type = mujoco.mjtOrientation.mjORIENTATION_XYAXES  # the provided camera is oriented by xyaxes,
    cam.alt.xyaxes = np.r_[R_mj[:, 0], R_mj[:, 1]]  # which wins over a quat set alone
    path = folder / "scene_camera.xml"
    path.write_text(spec.to_xml())
    return path


def thresholds():
    p = yaml.safe_load(Path("/ros2_ws/src/so101_perception/config/perception.yaml").read_text())
    p = p["so101_perception"]["ros__parameters"]
    return Thresholds(
        cube=hsv_ranges(*(p[f"hsv_cube_{k}"] for k in ("h_min", "h_max", "s_min", "v_min"))),
        box=hsv_ranges(*(p[f"hsv_box_{k}"] for k in ("h_min", "h_max", "s_min", "v_min", "v_max"))),
        min_area_px=p["min_area_px"],
        min_solidity=p["min_solidity"],
    )


def render(name):
    if name == "provided":
        sim = SO101Sim()
        sim.connect()
        T = sim.get_camera_extrinsics()  # spike only: the provided pose, to compare
        R_cv, t = T[:3, :3] @ FLIP, T[:3, 3]
        margin, scale = margin_and_scale(t, R_cv)
        print(f"provided: pos {np.round(t, 3).tolist()}, margin {margin:.1f} px, cube edge {scale:.1f} px")
    else:
        print(describe(name))
        t = poses[name][0][2]
        R_mj, R_cv = look_at(t)
        sim = SO101Sim(scene_path=scene(t, R_mj))
        sim.connect()
    m, d = sim.model, sim.data
    seg = mujoco.Renderer(m, height=H, width=W)
    seg.enable_segmentation_rendering()
    rgb = mujoco.Renderer(m, height=H, width=W)
    cube, box, shoulder = m.body("pickup_object").id, m.body("drop_box").id, m.body("shoulder").id
    cube_joint = m.joint("pickup_object_free").qposadr[0]

    def park():
        for joint, q in zip(ARM_JOINTS, PARK, strict=True):
            d.qpos[m.joint(joint).qposadr[0]] = q
        mujoco.mj_forward(m, d)

    def bodies():
        seg.update_scene(d, camera="external_cam")
        geom = seg.render()[..., 0]
        return np.where(geom >= 0, m.geom_bodyid[np.clip(geom, 0, None)], -1)

    sim._rng, box_rng = np.random.default_rng(0), np.random.default_rng(1)
    cut, hidden = 0, []
    for k in range(300):
        d.qpos[:] = 0.0
        m.body_pos[box, :2] = (2.0, 0.0)  # as /sim/randomize: the box away while the cube settles
        sim.randomize_object()
        m.body_pos[box, :2] = sample_box_xy(box_rng, d.body("pickup_object").xpos)
        park()
        seen = bodies()
        cube_px = seen == cube
        cut += bool(cube_px[0].any() or cube_px[-1].any() or cube_px[:, 0].any() or cube_px[:, -1].any())
        if k < 50:
            parked = np.array([cube_px.sum(), (seen == box).sum()])
            m.body_pos[shoulder, 2] += 10.0
            mujoco.mj_forward(m, d)
            lifted = bodies()
            hidden.append(parked / np.maximum([(lifted == cube).sum(), (lifted == box).sum()], 1))
            m.body_pos[shoulder, 2] -= 10.0
    h = np.array(hidden)
    print(f"  cube cut by the image edge in {cut}/300 scenes ({100 * cut / 300:.0f} %); in PARK, cube partly hidden "
          f"in {np.sum(h[:, 0] < 0.999)}/50 (worst {h[:, 0].min():.2f} visible), box in {np.sum(h[:, 1] < 0.999)}/50 "
          f"(worst {h[:, 1].min():.2f})")
    th, K_sim = thresholds(), sim.get_camera_intrinsics().copy()
    K_sim[:2, 2] -= 0.5  # OpenCV pixel centers [MES-41]
    box_rng = np.random.default_rng(3)
    radius, azimuth = box_rng.uniform(0.18, 0.30, 16), np.radians(box_rng.uniform(-90, -10, 16))
    verdict = {"inside": 0, "outside": 0, "unseen": 0}
    for bx, by in zip(radius * np.cos(azimuth), radius * np.sin(azimuth), strict=True):
        for dx, dy in ((0.0, 0.0), (0.015, 0.015), (-0.015, -0.015)):  # a cube lying in the box, off its center
            d.qpos[:] = 0.0
            d.qpos[cube_joint : cube_joint + 7] = [bx + dx, by + dy, 0.029, 1, 0, 0, 0]
            m.body_pos[box, :2] = (bx, by)
            park()
            rgb.update_scene(d, camera="external_cam")
            found = detect(rgb.render(), th).cube
            if found is None or found.truncated:
                verdict["unseen"] += 1
                continue
            p = pixel_to_world(found.u, found.v, K_sim, R_cv, t, 0.025)
            verdict["inside" if np.abs(p[:2] - [bx, by]).max() < 0.05 else "outside"] += 1
    print(f"  a cube lying in the box (48 cases), as VERIFY reads it: {verdict}")


if len(sys.argv) > 1:
    render(sys.argv[1])
else:
    print(f"A. target {np.round(target, 3)}, every corner >= {MARGIN} px inside the image:")
    for name in poses:
        print("  " + describe(name))
