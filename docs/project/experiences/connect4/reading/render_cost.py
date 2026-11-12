"""Q2: render time per frame, as the driver does it: update_scene + render on a copy of the state."""
import os, time, numpy as np, mujoco, cv2
from common import *
from so101_sim import DEFAULT_SCENE

def timeit(m, d, W, H, n=60):
    r = mujoco.Renderer(m, height=H, width=W); t = []
    for i in range(n + 10):
        t0 = time.perf_counter(); r.update_scene(d, camera="external_cam"); a = r.render(); t.append(time.perf_counter() - t0)
    r.close(); t = np.array(t[10:]) * 1e3
    return f"{W}x{H}: median {np.median(t):.1f} ms, p99 {np.percentile(t, 99):.1f} ms, max rate {1e3 / np.median(t):.0f} Hz"

print("LP_NUM_THREADS", os.environ["LP_NUM_THREADS"], "cpus", len(os.sched_getaffinity(0)))
spec = mujoco.MjSpec.from_file(str(DEFAULT_SCENE))
c = spec.camera("external_cam"); c.pos = P1_CAM["pos"]; c.alt.type = mujoco.mjtOrientation.mjORIENTATION_XYAXES; c.alt.xyaxes = P1_CAM["xyaxes"]
m = spec.compile(); d = mujoco.MjData(m); mujoco.mj_forward(m, d)
print("phase-1 scene ", timeit(m, d, 320, 240)); print("phase-1 scene ", timeit(m, d, 640, 480))
g = Grid(az=-51, marker=0.03); w = World(g, 320, 240, cam=P1_CAM)
rng = np.random.default_rng(1); w.board(random_board(rng, 42), rng=rng); w.arm([-1.0, -0.6, 0.6, 1.1, 1.57]); mujoco.mj_forward(w.m, w.d)
print("game scene 42t", timeit(w.m, w.d, 320, 240)); print("game scene 42t", timeit(w.m, w.d, 640, 480))
w.close()
