"""Full pick & place cycles in sim with the ground-truth perception stub (steps 4.2b, 4.2c): success, cycle
time, retries, and the brain's tick periods while the worker plans (C-33). Feeds MES-40.

In a throwaway tek5/control container, the workspace built and sourced:
    ros2 launch /ros2_ws/src/so101_bringup/test/gt_stub.launch.py > /tmp/stack.log 2>&1 &
    python3 exp_gt_cycles.py 20 /tmp/stack.log
Each episode: /sim/randomize (driver seed 0), /brain/start, wait for DONE or ERROR (60 s), then the ground
truth: success = cube within the box footprint, z < 6 cm (08 §3).
"""

import re
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

episodes, log = int(sys.argv[1]), sys.argv[2]
rclpy.init()
node = rclpy.create_node("exp_gt_cycles")
states, truth = [], {}
latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(String, "/brain_state", lambda m: states.append(m.data), latched)
for name in ("object", "drop_box"):
    node.create_subscription(PoseStamped, f"/sim/ground_truth/{name}", lambda m, n=name: truth.__setitem__(n, m), 10)
randomize, start = (node.create_client(Trigger, s) for s in ("/sim/randomize", "/brain/start"))


def spin_until(condition, timeout):
    end = time.monotonic() + timeout
    while not condition() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    return condition()


def call(client):
    assert client.wait_for_service(timeout_sec=30.0), client.srv_name
    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    return future.result()


def xyz(msg):
    p = msg.pose.position
    return np.array([p.x, p.y, p.z])


rows = []
for i in range(episodes):
    call(randomize)
    spin_until(lambda: states and states[-1] == "IDLE", 10.0)
    begun, t0 = len(states), time.monotonic()
    call(start)
    spin_until(lambda: "WAIT_TARGET" in states[begun:], 5.0)
    spin_until(lambda: states[-1] in ("DONE", "IDLE") or states[-1].startswith("ERROR"), 60.0)
    seconds, cycle = time.monotonic() - t0, states[begun:]
    truth.clear()
    spin_until(lambda: len(truth) == 2, 2.0)
    cube, box = xyz(truth["object"]), xyz(truth["drop_box"])
    in_box = bool(np.all(np.abs(cube[:2] - box[:2]) < 0.05) and cube[2] < 0.06)
    end = "DONE" if "DONE" in cycle else cycle[-1]
    rows.append((in_box and end == "DONE", seconds, cycle.count("RETRY")))
    print(f"{i:2d}: {'ok  ' if rows[-1][0] else 'FAIL'} {end:<24} {seconds:5.1f} s, {rows[-1][2]} retries, "
          f"cube {np.round(cube, 3)}, box {np.round(box[:2], 3)}", flush=True)
    if states[-1].startswith("ERROR"):
        call(start)  # re-arm
        spin_until(lambda: states[-1] == "IDLE", 5.0)

ok, seconds, retries = (np.array(column) for column in zip(*rows, strict=True))
pattern = r"PLAN: planner answered in ([\d.]+) s; longest tick period ([\d.]+) ms over (\d+) ticks"
plans = np.array([[float(v) for v in m] for m in re.findall(pattern, open(log).read())])
print(f"success {ok.sum()}/{episodes}; cycle median {np.median(seconds):.1f} s, max {seconds.max():.1f} s; "
      f"{retries.sum()} retries")
print(f"{len(plans)} plans: planner median {np.median(plans[:, 0]):.2f} s, max {plans[:, 0].max():.2f} s; "
      f"longest tick period during PLAN: median {np.median(plans[:, 1]):.1f} ms, max {plans[:, 1].max():.1f} ms "
      f"({int(plans[:, 2].sum())} ticks, {int((plans[:, 1] > 25.0).sum())} plans with a tick over 25 ms)")
node.destroy_node()
rclpy.shutdown()
