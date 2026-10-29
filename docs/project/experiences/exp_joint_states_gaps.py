"""MES-51: the gaps of /joint_states on a running stack (#176): who stalls, the driver or the transport?

Subscribes to /joint_states for a while, then prints every gap over GAP_S between two messages: their
arrival gap (steady clock) and their stamp gap (the driver's read times). Equal gaps: the driver did not
read or publish (its executor held; its log then shows a "late" warning). An arrival gap with a short
stamp gap: the messages were late on the way (DDS, or this subscriber starved).
In the control image, a stack running on the same ROS domain:

Usage: python3 exp_joint_states_gaps.py [minutes]
"""

import sys
import time

import numpy as np
import rclpy
from sensor_msgs.msg import JointState

GAP_S = 0.1  # 3 periods at 30 Hz

minutes = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
rclpy.init()
node = rclpy.create_node("exp_joint_states_gaps")
arrivals, stamps = [], []


def received(msg):
    arrivals.append(time.monotonic())
    stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)


node.create_subscription(JointState, "/joint_states", received, 10)
end = time.monotonic() + minutes * 60
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.1)
node.destroy_node()
rclpy.shutdown()

arrival_gaps, stamp_gaps = np.diff(arrivals), np.diff(stamps)
for i in np.flatnonzero(arrival_gaps > GAP_S):
    print(
        f"gap at {arrivals[i] - arrivals[0]:.1f} s: arrival {arrival_gaps[i]:.3f} s, stamp {stamp_gaps[i]:.3f} s",
        flush=True,
    )
print(
    f"{len(arrivals)} messages in {minutes:g} min, {len(arrivals) / (minutes * 60):.1f} Hz; arrival gap "
    f"p50 {np.percentile(arrival_gaps, 50) * 1e3:.1f} ms, p99 {np.percentile(arrival_gaps, 99) * 1e3:.1f} ms, "
    f"max {arrival_gaps.max():.3f} s; {np.count_nonzero(arrival_gaps > GAP_S)} gaps over {GAP_S} s",
    flush=True,
)
