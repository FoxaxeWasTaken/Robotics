"""MES-37: system-clock steps (WSL2) and their effect on the driver and the brain (#112).

Run next to a sim stack (driver + brain, `ros2 launch so101_bringup sim.launch.py`). For `duration` seconds:
- wall-clock steps: time.time() against time.monotonic(), sampled every 10 ms;
- /joint_states arrival gaps (monotonic);
- /go_to_target back and forth every 4 s: pauses of the stream in mid-motion, i.e. /joint_command gaps
  > 100 ms with no call inside (between motions the stream stops until the next call).

Usage: python3 exp_clock_steps.py [duration_s]
"""

import sys
import time
from itertools import pairwise

import numpy as np
import rclpy
from sensor_msgs.msg import JointState

from so101_interfaces.srv import GoToTarget

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 95.0
TARGETS = ((0.2, 0.0, 0.1), (0.25, 0.08, 0.12))
PERIOD = 4.0  # s between two /go_to_target calls

rclpy.init()
node = rclpy.create_node("exp_clock_steps")
arrivals, commands, calls = [], [], []  # monotonic times of /joint_states, /joint_command and calls
node.create_subscription(JointState, "/joint_states", lambda _: arrivals.append(time.monotonic()), 10)
node.create_subscription(JointState, "/joint_command", lambda _: commands.append(time.monotonic()), 100)
client = node.create_client(GoToTarget, "/go_to_target")
assert client.wait_for_service(timeout_sec=30.0), "/go_to_target not served"

steps, offset = [], time.time() - time.monotonic()
start = next_call = time.monotonic()
while time.monotonic() - start < DURATION:
    rclpy.spin_once(node, timeout_sec=0.01)
    o = time.time() - time.monotonic()
    if abs(o - offset) > 0.02:
        steps.append((time.monotonic() - start, o - offset))
    offset = o
    if time.monotonic() >= next_call:
        request = GoToTarget.Request()
        request.target_pose.header.frame_id = "base_link"
        p = request.target_pose.pose.position
        p.x, p.y, p.z = TARGETS[round((time.monotonic() - start) / PERIOD) % 2]
        calls.append(time.monotonic())
        client.call_async(request)
        next_call += PERIOD

gaps = np.diff(arrivals)
pauses = [b - a for a, b in pairwise(commands) if b - a > 0.1 and not any(a < c < b for c in calls)]
print(f"{DURATION:.0f} s: wall-clock steps (t s, step s): {[(round(t, 1), round(s, 3)) for t, s in steps]}")
late = [(round(arrivals[i] - start, 1), round(g, 3)) for i, g in enumerate(gaps) if g > 0.1]
print(f"/joint_states: {len(arrivals)} messages, largest gap {gaps.max() * 1000:.0f} ms, gaps > 100 ms (t s, gap s): {late}")
print(f"/go_to_target: {len(calls)} motions, {len(pauses)} pauses in mid-motion {[round(p, 2) for p in pauses]} s")
