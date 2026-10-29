#!/usr/bin/env python3
"""perception_node.py - finds the cube and the drop box in the external camera (docs/project/05).

Never talks to the backend (C-05): images and K come from the driver's topics only, the camera pose from
the extrinsics file. A timer takes each new image once: HSV detection, then for a valid, whole blob the ray
on the plane z = 2.5 cm, the temporal filter, and a PoseStamped in base_link with the image's stamp
(01 §3.1, 05 §1); the cube's orientation is its yaw about z (05 §4). Nothing is published for an invalid
detection; /perception/debug_image always comes.
"""

from pathlib import Path

import numpy as np
import rclpy
import rclpy.node
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from so101_common.robot import BASE_FRAME, CAMERA_FRAME
from so101_perception.debug import annotate, status
from so101_perception.detection import Thresholds, detect, hsv_ranges
from so101_perception.filter import MedianFilter
from so101_perception.geometry import cube_area_ratio, cube_yaw, load_extrinsics, pixel_to_world, quaternion

# Only the latest image matters; Best effort is compatible with the driver's Reliable publisher, and the
# debug image is Best effort for RViz (docs/project/01 §3.1, ADR-004).
LATEST = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
PLANE_Z = 0.025  # m: cube center, and mid-height of the box walls (ADR-012)
# Thresholds (05 §2.2), read at each image so they can be tuned at runtime: name: (default, description,
# min, max). A hue interval wraps around 180 when h_min > h_max (red).
PARAMETERS = {
    "hsv_cube_h_min": (160, "Cube hue interval start", 0, 180),
    "hsv_cube_h_max": (8, "Cube hue interval end", 0, 180),
    "hsv_cube_s_min": (150, "Cube minimum saturation", 0, 255),
    "hsv_cube_v_min": (30, "Cube minimum value", 0, 255),
    "hsv_box_h_min": (5, "Box hue interval start", 0, 180),
    "hsv_box_h_max": (25, "Box hue interval end", 0, 180),
    "hsv_box_s_min": (140, "Box minimum saturation", 0, 255),
    "hsv_box_v_min": (20, "Box minimum value", 0, 255),
    "hsv_box_v_max": (220, "Box maximum value", 0, 255),
    "min_area_px": (15.0, "Smallest blob kept, px (05 §2.3)", 1.0, 10000.0),
    "min_solidity": (0.6, "Smallest area / convex hull area kept (05 §2.3)", 0.0, 1.0),
}
# Startup-only parameters, same format.
STARTUP = {
    "process_rate_hz": (
        20.0,
        "New image check rate; above the camera's 10 Hz, each image is processed once",
        1.0,
        100.0,
    ),
    "filter_size": (5, "Median filter window; a jump must also persist this many times (05 §7)", 1, 50),
    "filter_jump_m": (0.02, "A detection this far from the median is a jump, m (05 §7)", 0.001, 0.2),
    "min_cube_area_ratio": (
        0.85,
        "Cube blob area / a 5 cm face at its depth below which it is partly hidden, not published (05 §6)",
        0.0,
        2.0,
    ),
}


class PerceptionNode(rclpy.node.Node):
    """Cube and drop-box detection. Today: republishes each image with its status."""

    def __init__(self):
        super().__init__("so101_perception")
        for name, spec in PARAMETERS.items():
            self._declare(name, *spec)
        startup = {name: self._declare(name, *spec, read_only=True) for name, spec in STARTUP.items()}
        extrinsics = self.declare_parameter(
            "extrinsics_file",
            str(Path(get_package_share_directory("so101_perception")) / "config" / "camera_extrinsics.yaml"),
            ParameterDescriptor(description="Camera pose in base_link (05 §5.1)", read_only=True),
        ).value
        # ROS ignores a key no node declares: a typo in perception.yaml would keep the default (#57).
        if unknown := sorted(set(self._parameter_overrides) - set(self._parameters)):  # rclpy: no public API
            raise ValueError(f"Unknown parameters {unknown}: a typo in a config file? (05)")
        T = load_extrinsics(extrinsics).T_base_cam  # a wrong file stops the node here (ValueError)
        self._R, self._t = T[:3, :3], T[:3, 3]
        # the camera pose for tf2 and RViz (01 §4, 05 §5.1): static, from the same file
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id, tf.child_frame_id = BASE_FRAME, CAMERA_FRAME
        t, r = tf.transform.translation, tf.transform.rotation
        t.x, t.y, t.z = map(float, self._t)
        r.x, r.y, r.z, r.w = map(float, quaternion(self._R))
        self._tf = StaticTransformBroadcaster(self)
        self._tf.sendTransform(tf)
        self._bridge = CvBridge()
        self._image = None  # latest image not processed yet
        self._camera_info = None  # latest K
        self._min_area_ratio = startup["min_cube_area_ratio"]
        self._filters = {
            name: MedianFilter(startup["filter_size"], startup["filter_jump_m"]) for name in ("cube", "box")
        }

        self.create_subscription(Image, "/external_cam/image_raw", self._cb_image, LATEST)
        self.create_subscription(CameraInfo, "/external_cam/camera_info", self._cb_camera_info, LATEST)
        self._pub_debug = self.create_publisher(Image, "/perception/debug_image", LATEST)
        self._pubs = {
            "cube": self.create_publisher(PoseStamped, "/object_position", 1),  # Reliable, depth 1 (01 §3.1)
            "box": self.create_publisher(PoseStamped, "/drop_box_position", 1),
        }
        # Steady clock: a step back of the system clock (WSL2, MES-37) would pause a ROS-clock timer.
        self.create_timer(
            1.0 / startup["process_rate_hz"], self._process, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )
        self.get_logger().info("Perception node ready.")

    def _declare(self, name, default, description, low, high, read_only=False):
        """A parameter with its range: an int range for an int default, else a float one."""
        bounds = (
            {"integer_range": [IntegerRange(from_value=low, to_value=high)]}
            if isinstance(default, int)
            else {"floating_point_range": [FloatingPointRange(from_value=low, to_value=high)]}
        )
        descriptor = ParameterDescriptor(description=description, read_only=read_only, **bounds)
        return self.declare_parameter(name, default, descriptor).value

    def _cb_image(self, msg: Image):
        self._image = msg

    def _cb_camera_info(self, msg: CameraInfo):
        self._camera_info = msg

    def _thresholds(self) -> Thresholds:
        def value(name):
            return self.get_parameter(name).value

        return Thresholds(
            cube=hsv_ranges(*(value(f"hsv_cube_{k}") for k in ("h_min", "h_max", "s_min", "v_min"))),
            box=hsv_ranges(*(value(f"hsv_box_{k}") for k in ("h_min", "h_max", "s_min", "v_min", "v_max"))),
            min_area_px=value("min_area_px"),
            min_solidity=value("min_solidity"),
        )

    def _process(self):
        """The latest image, once: the poses of its valid detections, then the annotated debug image."""
        if self._image is None:
            return
        msg, self._image = self._image, None
        rgb = self._bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        found = detect(rgb, self._thresholds())
        occluded = False
        info = self._camera_info
        if info is not None and info.k[0] > 0.0:  # no K, no ray
            K = np.array(info.k).reshape(3, 3)
            for name, blob, z in (("cube", found.cube, None), ("box", found.box, 0.0)):  # box: its floor
                if blob is None or blob.truncated:
                    continue
                p = pixel_to_world(blob.u, blob.v, K, self._R, self._t, PLANE_Z)
                if p is None:
                    continue
                if (
                    name == "cube"
                    and cube_area_ratio(blob.contour, p, K, self._R, self._t) < self._min_area_ratio
                ):
                    occluded = True  # its center would be pulled towards its visible part (MES-46)
                    continue
                self._publish(name, blob, p, K, msg.header.stamp, z)
        debug = self._bridge.cv2_to_imgmsg(
            annotate(rgb, status(found.cube, occluded), found), encoding="rgb8"
        )
        debug.header = msg.header
        self._pub_debug.publish(debug)

    def _publish(self, name, blob, p, K, stamp, z):
        """The filtered point p of the blob on the plane, z replaced when given; nothing when rejected."""
        p = self._filters[name].update(p)
        if p is None:
            return
        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = stamp, BASE_FRAME
        pose.pose.position.x, pose.pose.position.y = float(p[0]), float(p[1])
        pose.pose.position.z = float(p[2] if z is None else z)
        yaw = cube_yaw(blob.contour, K, self._R, self._t) if name == "cube" else 0.0
        pose.pose.orientation.z, pose.pose.orientation.w = float(np.sin(yaw / 2)), float(np.cos(yaw / 2))
        self._pubs[name].publish(pose)


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
