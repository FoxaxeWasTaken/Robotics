#!/usr/bin/env python3
"""perception_node.py - finds the cube and the drop box in the external camera (docs/project/05).

Never talks to the backend (C-05): images and K come from the driver's topics only. Today (step 3.1) the
skeleton: the callbacks store the latest image and camera_info, a timer republishes each new image on
/perception/debug_image with its status. Detection comes at 3.2b, pixel -> 3D at 3.3a.
"""

import rclpy
import rclpy.node
from cv_bridge import CvBridge
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from so101_perception.debug import NO_DETECTION, annotate

# Only the latest image matters; Best effort is compatible with the driver's Reliable publisher, and the
# debug image is Best effort for RViz (docs/project/01 §3.1, ADR-004).
LATEST = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


class PerceptionNode(rclpy.node.Node):
    """Cube and drop-box detection. Today: republishes each image with its status."""

    def __init__(self):
        super().__init__("so101_perception")
        rate = self.declare_parameter(
            "process_rate_hz",
            20.0,
            ParameterDescriptor(
                description="New image check rate; above the camera's 10 Hz, each image is processed once",
                read_only=True,
                floating_point_range=[FloatingPointRange(from_value=1.0, to_value=100.0)],
            ),
        ).value
        self._bridge = CvBridge()
        self._image = None  # latest image not processed yet
        self._camera_info = None  # latest K, D, P (pixel -> 3D at 3.3a)

        self.create_subscription(Image, "/external_cam/image_raw", self._cb_image, LATEST)
        self.create_subscription(CameraInfo, "/external_cam/camera_info", self._cb_camera_info, LATEST)
        self._pub_debug = self.create_publisher(Image, "/perception/debug_image", LATEST)
        # Steady clock: a step back of the system clock (WSL2, MES-37) would pause a ROS-clock timer.
        self.create_timer(1.0 / rate, self._process, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info("Perception node ready.")

    def _cb_image(self, msg: Image):
        self._image = msg

    def _cb_camera_info(self, msg: CameraInfo):
        self._camera_info = msg

    def _process(self):
        """The latest image, once: annotated on /perception/debug_image, same header."""
        if self._image is None:
            return
        msg, self._image = self._image, None
        rgb = self._bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        debug = self._bridge.cv2_to_imgmsg(annotate(rgb, NO_DETECTION), encoding="rgb8")
        debug.header = msg.header
        self._pub_debug.publish(debug)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
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
