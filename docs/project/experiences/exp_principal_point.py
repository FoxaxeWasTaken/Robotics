"""Sim camera: where is the principal point in OpenCV pixel coordinates? (MES-41)

The provided get_camera_intrinsics() puts cx = W/2, cy = H/2. OpenGL (MuJoCo) puts pixel edges on integers,
so (W/2, H/2) is the corner shared by the four central pixels; OpenCV and CameraInfo put pixel centers on
integers, where that corner is ((W - 1)/2, (H - 1)/2). Measured: the red blob of the cube against the exact
silhouette of the cube (convex hull of its 8 projected corners), for both K, over 50 scenes with the arm lifted
out of the picture; then the pixel -> 3D error of the blob centroid on the plane z = 2.5 cm (05 §4).
"""
import cv2, mujoco, numpy as np
from so101_sim import SO101Sim

FLIP = np.diag([1.0, -1.0, -1.0])                   # MuJoCo camera -> OpenCV optical (05 §3.1)
CORNERS = 0.025 * np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])

def project(K, R, t, P):
    c = (np.atleast_2d(P) - t) @ R; uv = c @ K.T; return uv[:, :2] / uv[:, 2:]

def ray_plane(K, R, t, u, v, z):
    d = R @ np.linalg.solve(K, [u, v, 1.0]); return t + (z - t[2]) / d[2] * d

sim = SO101Sim(seed=0); sim.connect(); m, d = sim.model, sim.data
m.body_pos[m.body("shoulder").id] += [0.0, 0.0, 10.0]   # the arm out of the picture: the whole silhouette shows
K_mj = sim.get_camera_intrinsics(); K_cv = K_mj.copy(); K_cv[:2, 2] -= 0.5
T = sim.get_camera_extrinsics(); R, t = T[:3, :3] @ FLIP, T[:3, 3]
W, H = sim.camera_width, sim.camera_height
offsets = {"W/2": [], "(W-1)/2": []}; errors = {"W/2": [], "(W-1)/2": []}
while len(offsets["W/2"]) < 50:
    sim.randomize_object(); mujoco.mj_forward(m, d)
    cube = d.body("pickup_object"); P = cube.xpos + CORNERS @ cube.xmat.reshape(3, 3).T
    hsv = cv2.cvtColor(sim.render_camera(), cv2.COLOR_RGB2HSV)
    red = cv2.inRange(hsv, (160, 100, 60), (180, 255, 255)) | cv2.inRange(hsv, (0, 100, 60), (8, 255, 255))
    rows, cols = np.nonzero(red)
    if not len(rows) or rows.min() == 0 or cols.min() == 0 or rows.max() == H - 1 or cols.max() == W - 1:
        continue                                        # truncated by the border: the silhouette is cut
    blob = np.array([cols.mean(), rows.mean()])
    for name, K in (("W/2", K_mj), ("(W-1)/2", K_cv)):
        hull = cv2.convexHull(project(K, R, t, P).astype(np.float32))
        mo = cv2.moments(hull); offsets[name].append(blob - [mo["m10"] / mo["m00"], mo["m01"] / mo["m00"]])
        errors[name].append(np.linalg.norm(ray_plane(K, R, t, *blob, 0.025)[:2] - cube.xpos[:2]))
for name in offsets:
    o, e = np.array(offsets[name]), np.array(errors[name]) * 1e3
    print(f"[K cx,cy = {name:8s}] blob - silhouette centroid: mean ({o[:, 0].mean():+.3f}, {o[:, 1].mean():+.3f}) px, "
          f"max |.| {np.abs(o).max():.3f} px | pixel -> 3D (blob, z = 2.5 cm): mean {e.mean():.2f} mm, "
          f"max {e.max():.2f} mm (n={len(o)})")
