#!/usr/bin/env python3
"""calibrate_extrinsics - the camera pose from the ArUco markers of a layout file (05 §5.1, 3.4b).

    ros2 run so101_perception calibrate_extrinsics --markers <layout.yaml> --out <camera_extrinsics.yaml>

The arm in PARK, the camera still, at 640x480 (ADR-013: at 320x240 distant markers are not detected). Grabs
--frames images and the camera_info (the driver's K, OpenCV pixel centers [MES-41]), solves on all their
detections and writes the extrinsics file; exits 1 and writes nothing under 4 markers or above the layout's
reprojection gate.
"""

import argparse
import sys
import time
from datetime import date
from pathlib import Path

import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from so101_perception.calibration import CalibrationError, detect_markers, solve
from so101_perception.geometry import Extrinsics, save_extrinsics


def main(args=None):
    parser = argparse.ArgumentParser(description="The camera pose from ArUco markers (05 §5.1).")
    parser.add_argument("--markers", type=Path, required=True, help="layout file, e.g. markers_sim.yaml")
    parser.add_argument("--out", type=Path, required=True, help="extrinsics file to write")
    parser.add_argument("--frames", type=int, default=10, help="images to solve on (default 10)")
    parser.add_argument("--timeout", type=float, default=30.0, help="s to get them (default 30)")
    opts = parser.parse_args(rclpy.utilities.remove_ros_args(args or sys.argv)[1:])
    layout = yaml.safe_load(opts.markers.read_text())

    rclpy.init(args=args)
    node = rclpy.create_node("calibrate_extrinsics")
    images, info = [], []
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(Image, "/external_cam/image_raw", images.append, qos)
    node.create_subscription(CameraInfo, "/external_cam/camera_info", info.append, qos)
    deadline = time.monotonic() + opts.timeout
    try:
        while (len(images) < opts.frames or not info) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if len(images) < opts.frames or not info:
            raise CalibrationError(
                f"{len(images)} images and {len(info)} camera_info in {opts.timeout:.0f} s"
            )
        bridge, camera = CvBridge(), info[-1]
        frames = [
            detect_markers(bridge.imgmsg_to_cv2(m, "rgb8"), layout["dictionary"])
            for m in images[: opts.frames]
        ]
        dist = np.array(camera.d) if len(camera.d) else None
        T, error, ids = solve(frames, layout, np.array(camera.k).reshape(3, 3), dist)
    except CalibrationError as e:
        node.get_logger().error(f"Calibration refused: {e}")
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    method = (
        f"ArUco ({opts.markers.name}, markers {ids}, {opts.frames} frames at {camera.width}x{camera.height})"
    )
    save_extrinsics(opts.out, Extrinsics(T, method, date.today().isoformat(), round(error, 3)))
    print(
        f"{opts.out}: {len(ids)} markers, mean reprojection {error:.3f} px, "
        f"camera at {np.round(T[:3, 3], 4)} m"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
