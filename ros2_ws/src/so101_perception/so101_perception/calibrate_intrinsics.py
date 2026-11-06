#!/usr/bin/env python3
"""calibrate_intrinsics - K and distortion of the webcam from a ChArUco board (05 §3.2, ADR-014, 6.2b).

    ros2 run so101_perception calibrate_intrinsics --out <camera_real_intrinsics.yaml>

Show the board to the camera, at the driver's resolution: all over the image, tilted, near and far. Each view
counts once the board is held still. Exits 1 and writes nothing under the views needed or above the RMS gate.
"""

import argparse
import sys
import time
from pathlib import Path

import rclpy
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from so101_perception.calibration import CalibrationError
from so101_perception.intrinsics import BoardDetector, ViewPicker, calibrate, save_camera_info


def main(args=None):
    parser = argparse.ArgumentParser(description="K and distortion from a ChArUco board (05 §3.2).")
    parser.add_argument("--out", type=Path, required=True, help="calibration file to write")
    parser.add_argument("--views", type=int, default=25, help="views to collect (default 25)")
    parser.add_argument("--timeout", type=float, default=300.0, help="s to collect them (default 300)")
    opts = parser.parse_args(rclpy.utilities.remove_ros_args(args or sys.argv)[1:])

    rclpy.init(args=args)
    node = rclpy.create_node("calibrate_intrinsics")
    latest = []
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(Image, "/external_cam/image_raw", latest.append, qos)
    bridge, detector, picker = CvBridge(), BoardDetector(), ViewPicker()
    size = None
    deadline = time.monotonic() + opts.timeout
    try:
        while len(picker.views) < opts.views and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if latest:
                image = latest.pop()
                latest.clear()
                size = (image.width, image.height)
                if picker.offer(detector.corners(bridge.imgmsg_to_cv2(image, "rgb8"))):
                    print(f"view {len(picker.views)}/{opts.views}", flush=True)
        K, D, rms = calibrate(picker.views, size)
    except CalibrationError as e:
        node.get_logger().error(f"Calibration refused: {e}")
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    save_camera_info(opts.out, K, D, size, rms)
    print(
        f"{opts.out}: {len(picker.views)} views, RMS {rms:.3f} px, fx {K[0, 0]:.1f}, D {D.round(4).tolist()}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
