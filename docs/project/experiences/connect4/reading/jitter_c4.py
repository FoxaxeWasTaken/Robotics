"""The real DriverNode (exp_driver_jitter.py's method) on the game scene: control period and image rate.
In a throwaway tools/so101 run container, 20 s per config, one image subscriber in another process."""
import os, subprocess, sys, time
sys.path.insert(0, "/out")
import numpy as np, rclpy
from so101_driver.driver_node import DriverNode
from common import *

DURATION = 20.0
control_step = DriverNode._control_step

def run(name, scene, W, H, rate=10.0):
    ticks, frames = [], []
    def timed_step(self):
        ticks.append(time.monotonic()); control_step(self)
    DriverNode._control_step = timed_step
    rclpy.init(args=["--ros-args", "-p", f"scene_path:={scene}", "-p", f"camera_width:={W}", "-p", f"camera_height:={H}", "-p", f"camera_rate_hz:={rate}"])
    node = DriverNode(); publish = node._pub_image.publish
    node._pub_image.publish = lambda msg: (frames.append(time.monotonic()), publish(msg))
    sub = subprocess.Popen(["ros2", "topic", "hz", "/external_cam/image_raw"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    end = time.monotonic() + DURATION
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    sub.terminate(); node.destroy_node(); rclpy.shutdown()
    p = np.diff(ticks) * 1e3; s = [t for t in frames if t > ticks[0] + 5.0]
    hz = (len(s) - 1) / (s[-1] - s[0]) if len(s) > 1 else 0.0
    print(f"{name:44s} {W}x{H}: {len(p)} ticks, p50 {np.percentile(p, 50):.1f}, p99 {np.percentile(p, 99):.1f}, max {p.max():.1f} ms; image {hz:.1f} Hz; load {os.getloadavg()[0]:.1f}", flush=True)

rng = np.random.default_rng(1); board = random_board(rng, 42); g = Grid(az=-51, marker=0.03)
print("cores", os.cpu_count(), "LP_NUM_THREADS default half", flush=True)
p1 = "/tmp/p1"; os.makedirs(p1)
import mujoco, shutil
from so101_sim import DEFAULT_SCENE
shutil.copytree(os.path.dirname(DEFAULT_SCENE), p1 + "/so101")
spec = mujoco.MjSpec.from_file(p1 + "/so101/" + os.path.basename(DEFAULT_SCENE))
c = spec.camera("external_cam"); c.pos = P1_CAM["pos"]; c.alt.type = mujoco.mjtOrientation.mjORIENTATION_XYAXES; c.alt.xyaxes = P1_CAM["xyaxes"]
open(p1 + "/so101/scene_so101.xml", "w").write(spec.to_xml())
default = generate(g, cam=P1_CAM, static_board=board)
light = generate(g, cam=P1_CAM, static_board=board, shadow=1024, reflect=0.0)
run("phase-1 scene (baseline)", p1 + "/so101/scene_so101.xml", 320, 240)
run("game, full board, default quality", default, 320, 240)
run("game, full board, default quality", default, 640, 480)
run("game, full board, shadow 1024, no reflection", light, 640, 480)
run("game, full board, shadow 1024, no reflection, 5 Hz", light, 640, 480, 5.0)
run("phase-1 scene (baseline again)", p1 + "/so101/scene_so101.xml", 320, 240)
