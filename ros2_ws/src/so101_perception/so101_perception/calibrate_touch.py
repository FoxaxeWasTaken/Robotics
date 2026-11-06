#!/usr/bin/env python3
"""calibrate_touch - the camera pose from the ChArUco board and the arm's touches on it (05 §5.4, #68).

    ros2 run so101_perception calibrate_touch --touch 0 x y z --touch 5 x y z --touch 18 x y z \
        --touch 23 x y z --out <camera_extrinsics_real.yaml>

The board flat on the table, in the working area, not moved since the touches; the TCP (jaws closed) read from
/end_effector_pose on each board corner, in m. No ruler, and the pose is in the arm's own frame: a zero offset
the touches contain, the grasps contain too. The arm out of the way for the images. Exits 1 and writes nothing
when the touches disagree with the board (MAX_TOUCH_M) or the board reprojects badly (MAX_BOARD_PX).
"""

import argparse
import sys
import time
from datetime import date
from pathlib import Path

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from so101_perception.calibration import CalibrationError
from so101_perception.geometry import Extrinsics, save_extrinsics
from so101_perception.intrinsics import MIN_CORNERS, BoardDetector, board_pose, extrinsics_from_touches

MAX_TOUCH_M = 0.004
MAX_BOARD_PX = 1.0


def main(args=None):
    parser = argparse.ArgumentParser(description="The camera pose from the board and touches (05 §5.4).")
    parser.add_argument("--touch", nargs=4, action="append", required=True, metavar=("ID", "X", "Y", "Z"))
    parser.add_argument("--out", type=Path, required=True, help="extrinsics file to write")
    parser.add_argument("--timeout", type=float, default=30.0, help="s to get an image (default 30)")
    opts = parser.parse_args(rclpy.utilities.remove_ros_args(args or sys.argv)[1:])
    touches = {int(t[0]): np.array([float(v) for v in t[1:]]) for t in opts.touch}

    rclpy.init(args=args)
    node = rclpy.create_node("calibrate_touch")
    images, info = [], []
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(Image, "/external_cam/image_raw", images.append, qos)
    node.create_subscription(CameraInfo, "/external_cam/camera_info", info.append, qos)
    deadline = time.monotonic() + opts.timeout
    try:
        while (not images or not info) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not images or not info:
            raise CalibrationError(
                f"{len(images)} images and {len(info)} camera_info in {opts.timeout:.0f} s"
            )
        corners = BoardDetector().corners(CvBridge().imgmsg_to_cv2(images[-1], "rgb8"))
        if len(corners) < MIN_CORNERS:  # a touched corner need not be seen: the board's geometry gives it
            raise CalibrationError(f"{len(corners)} corners seen, {MIN_CORNERS} needed")
        K, D = np.array(info[-1].k).reshape(3, 3), np.array(info[-1].d)
        T_cam_board, error = board_pose(corners, K, D)
        T, residuals = extrinsics_from_touches(T_cam_board, touches)
        print(f"{len(corners)} corners, {error:.3f} px; touches off by {np.round(residuals * 1000, 1)} mm")
        if error > MAX_BOARD_PX or residuals.max() > MAX_TOUCH_M:
            raise CalibrationError(f"above {MAX_BOARD_PX} px or {MAX_TOUCH_M * 1000:.0f} mm")
    except CalibrationError as e:
        node.get_logger().error(f"Calibration refused: {e}")
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    method = f"ChArUco board, {len(corners)} corners, touched at {sorted(touches)}"
    save_extrinsics(opts.out, Extrinsics(T, method, date.today().isoformat(), round(error, 3)))
    print(f"{opts.out}: camera at {np.round(T[:3, 3], 4)} m")
    return 0


if __name__ == "__main__":
    sys.exit(main())
