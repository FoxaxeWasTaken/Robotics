#!/usr/bin/env python3
"""gt_perception_stub.py - DEVELOPMENT ONLY (step 4.2c): perception replaced by the sim's ground truth.

Republishes /sim/ground_truth/object and /sim/ground_truth/drop_box (driver with expose_ground_truth) as
/object_position and /drop_box_position, as perception publishes them (01 §3.1): base_link, 10 Hz (each
ground-truth message once), its stamp, the cube yaw only (90-degree-symmetric), Gaussian noise of noise_m on
x and y. Unlike perception it never loses the cube, even in the gripper: the brain reads detections only in
WAIT_TARGET and VERIFY.

Under test/, started only by gt_stub.launch.py there: the C-06 guard rejects it in any other launch file.
"""

import numpy as np
import rclpy
import rclpy.node
from geometry_msgs.msg import PoseStamped
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException

TOPICS = {"object": "/object_position", "drop_box": "/drop_box_position"}


class GtPerceptionStub(rclpy.node.Node):
    def __init__(self):
        super().__init__("gt_perception_stub")
        self._noise = self.declare_parameter("noise_m", 0.001).value  # standard deviation on x and y, m
        self._rng = np.random.default_rng(self.declare_parameter("seed", 0).value)
        self._latest = {}  # ground truth not republished yet, per body
        self._pubs = {name: self.create_publisher(PoseStamped, topic, 1) for name, topic in TOPICS.items()}
        for name in TOPICS:
            self.create_subscription(
                PoseStamped,
                f"/sim/ground_truth/{name}",
                lambda msg, n=name: self._latest.__setitem__(n, msg),
                10,
            )
        # Steady clock (WSL2 clock steps, MES-37); 20 Hz, above the 10 Hz ground truth.
        self.create_timer(0.05, self._publish, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().warning(
            "Ground-truth perception stub: development only, never in the pipeline (C-06)."
        )

    def _publish(self):
        for name, truth in self._latest.items():
            msg = PoseStamped(header=truth.header)
            p = truth.pose.position
            dx, dy = self._rng.normal(0.0, self._noise, 2)
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = p.x + dx, p.y + dy, p.z
            yaw = cube_yaw(truth.pose.orientation) if name == "object" else 0.0
            msg.pose.orientation.z, msg.pose.orientation.w = float(np.sin(yaw / 2)), float(np.cos(yaw / 2))
            self._pubs[name].publish(msg)
        self._latest.clear()


def cube_yaw(o) -> float:
    """Yaw about z in [0, pi/2): what perception measures of the 90-degree-symmetric cube (05 §1)."""
    yaw = np.arctan2(2.0 * (o.w * o.z + o.x * o.y), 1.0 - 2.0 * (o.y * o.y + o.z * o.z))
    return float(yaw % (np.pi / 2))


def main():
    rclpy.init()
    node = GtPerceptionStub()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
