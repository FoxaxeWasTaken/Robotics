"""Real DriverNode, in-process, on the game scene: control period and image rate. As exp_driver_jitter.py (MES-34).
tools/so101 run --out <this folder> 'python3 /out/drv.py'"""
import subprocess, sys, tempfile, time, json, os
sys.path.insert(0, "/out")
import numpy as np, mujoco, rclpy
from rclpy.executors import SingleThreadedExecutor
from so101_driver.driver_node import DriverNode
from exp_connect4_scene import build, GAME_OPTIONS, PARAMS, ASSETS, drop_height, bottom_z, PAN_X, park_xy
DURATION = float(os.environ.get("DUR", 40.0))
control_step = DriverNode._control_step

def run(label, over, game=True, camera=True):
    ticks, frames, locks = [], [], []
    def timed_step(self):
        ticks.append(time.monotonic())
        control_step(self)
    DriverNode._control_step = timed_step
    if game:
        path, p, tz = build(tempfile.mkdtemp(prefix="so101_"), **over)
    else:
        from so101_bringup.sim_scene import generate
        import yaml
        scene = yaml.safe_load(open("/repo/ros2_ws/src/so101_bringup/config/sim_scene.yaml"))
        path = generate(tempfile.mkdtemp(prefix="so101_"), camera=scene["camera"])
        p, tz = None, 0.0
    rclpy.init(args=["--ros-args", "-p", f"scene_path:={path}"])
    node = DriverNode()
    publish = node._pub_image.publish
    node._pub_image.publish = lambda msg: (frames.append(time.monotonic()), publish(msg))
    sub = subprocess.Popen(["ros2", "topic", "hz", "/external_cam/image_raw"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    ex = SingleThreadedExecutor(); ex.add_node(node)
    sim = node._robot
    state = {"t0": time.monotonic(), "n": 0}
    hold = []
    def play():
        t = time.monotonic() - state["t0"]
        node._goal["shoulder_pan"] = -1.2 + 0.6 * np.sin(1.0 * t)
        node._goal["wrist_flex"] = 0.4 * np.sin(1.3 * t)
        node._goal["shoulder_lift"] = -0.3
        node._goal["elbow_flex"] = -0.2
        if game and state["n"] < p["tokens"] and t > 5.0 + 0.75 * state["n"]:
            i = state["n"]; col = i % 7
            j = sim.model.joint(f"token_{i}_free")
            a = PAN_X + p["r0"] + p["pitch"] * col
            q0 = time.perf_counter()
            with node._backend_lock:  # the drop_token service
                sim.data.qpos[j.qposadr[0]:j.qposadr[0]+3] = [a, 0, tz + drop_height(p)]
                sim.data.qpos[j.qposadr[0]+3:j.qposadr[0]+7] = [0.7071, 0.7071, 0, 0]
                sim.data.qvel[j.dofadr[0]:j.dofadr[0]+6] = 0
            hold.append(time.perf_counter() - q0)
            state["n"] += 1
    node.create_timer(0.05, play, clock=node._steady)
    end = time.monotonic() + DURATION
    while time.monotonic() < end:
        ex.spin_once(timeout_sec=0.1)
    sub.terminate()
    cells = None
    if game:
        c = [(sim.data.body(f"token_{i}").xpos[0] - PAN_X - p["r0"]) / p["pitch"] for i in range(state["n"])]
        z = [(sim.data.body(f"token_{i}").xpos[2] - tz - bottom_z(p)) / p["d"] - 0.5 for i in range(state["n"])]
        cells = int(sum(abs(a - i % 7) < 0.25 and abs(b - (i // 7)) < 0.25 for i, (a, b) in enumerate(zip(c, z))))
    node.destroy_node(); rclpy.shutdown()
    periods = np.diff(ticks[50:]) * 1e3
    steady = [t for t in frames if t > ticks[0] + 5.0]
    hz = (len(steady) - 1) / (steady[-1] - steady[0]) if len(steady) > 1 else 0.0
    r = dict(label=label, periods=len(periods), p50=round(float(np.percentile(periods, 50)), 1), p99=round(float(np.percentile(periods, 99)), 1), max=round(float(periods.max()), 1),
             over25=int((periods > 25).sum()), over40=int((periods > 40).sum()), mean=round(float(periods.mean()), 2), image_hz=round(hz, 1), tokens=state["n"], in_cell=cells,
             lock_us=round(float(np.median(hold)) * 1e6, 1) if hold else None)
    print(r, flush=True)
    return r

out = []
out.append(run("phase-1 scene", {}, game=False))
out.append(run("game, sleep", dict(GAME_OPTIONS), game=True))
out.append(run("game, sleep, 2 render threads", dict(GAME_OPTIONS), game=True)) if False else None
print(os.cpu_count(), os.environ.get("LP_NUM_THREADS"))
json.dump(out, open("/out/drv.json", "w"), indent=1)
