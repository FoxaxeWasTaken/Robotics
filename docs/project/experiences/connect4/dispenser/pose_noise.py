"""pose_noise.py [seconds]: on a running game stack, each /connect4/dispenser_pose and /connect4/grid_pose against
where the layout puts them: how far the three stations and the columns 0 and 6 are from the truth, image by
image, a game being played [MES-77]. A measurement script: it reads the layout's placement, as a test may."""

import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState

from so101_connect4 import board, layout

LAYOUT = layout.load()
TRUE_D, TRUE_G = layout.dispenser_pose(LAYOUT), layout.grid_pose(LAYOUT)
RAILS = LAYOUT["dispenser"]["rails"]
rclpy.init()
node = rclpy.create_node("pose_noise")
rows, grid_rows, arm = [], [], [None]
t0 = time.monotonic()


def T(msg):
    p, o = msg.pose.position, msg.pose.orientation
    return board.transform((p.x, p.y, p.z), (o.x, o.y, o.z, o.w))


def on_dispenser(msg):
    pose = T(msg)
    errors = [board.station_target(LAYOUT, pose, r) - board.station_target(LAYOUT, TRUE_D, r) for r in range(RAILS)]
    rows.append((time.monotonic() - t0, arm[0], *[e * 1e3 for e in errors]))


def on_grid(msg):
    pose = T(msg)
    errors = [board.column_target(LAYOUT, pose, c) - board.column_target(LAYOUT, TRUE_G, c) for c in (0, 6)]
    grid_rows.append((time.monotonic() - t0, *[e * 1e3 for e in errors]))


def on_joints(msg):
    arm[0] = round(float(msg.position[0]), 2)


latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(PoseStamped, "/connect4/dispenser_pose", on_dispenser, latched)
node.create_subscription(PoseStamped, "/connect4/grid_pose", on_grid, latched)
node.create_subscription(JointState, "/joint_states", on_joints, 10)
end = time.monotonic() + float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.1)
print(f"{len(rows)} dispenser poses, {len(grid_rows)} grid poses")
for rail in range(RAILS):
    norms = np.array([np.linalg.norm(row[2 + rail]) for row in rows])
    xyz = np.array([row[2 + rail] for row in rows])
    print(f"station {rail}: error mm p50 {np.percentile(norms, 50):.2f}, p95 {np.percentile(norms, 95):.2f}, "
          f"p99 {np.percentile(norms, 99):.2f}, max {norms.max():.2f}; mean xyz {np.round(xyz.mean(axis=0), 2)}, std {np.round(xyz.std(axis=0), 2)}")
at_handover = [row for row in rows if row[1] is not None and abs(row[1] + 0.8) < 0.02]
print(f"{len(at_handover)} of them with the pan at HANDOVER")
for rail in range(RAILS):
    norms = np.array([np.linalg.norm(row[2 + rail]) for row in at_handover])
    print(f"station {rail} at HANDOVER: p50 {np.percentile(norms, 50):.2f}, p99 {np.percentile(norms, 99):.2f}, max {norms.max():.2f}")
worst = sorted(rows, key=lambda row: -np.linalg.norm(row[2 + RAILS - 1]))[:8]
for row in worst:
    print(f"  t {row[0]:6.1f} s, pan {row[1]}: station {RAILS - 1} off by {np.round(row[2 + RAILS - 1], 1)} mm")
for i, c in enumerate((0, 6)):
    norms = np.array([np.linalg.norm(row[1 + i]) for row in grid_rows])
    print(f"column {c}: error mm p50 {np.percentile(norms, 50):.2f}, p99 {np.percentile(norms, 99):.2f}, max {norms.max():.2f}")
