#!/usr/bin/env python3
"""board_reader_node.py - reads the Connect Four grid in the external camera (docs/project/phase2/P2-02 §3).

Never talks to the backend and knows no ground truth (C-05, C-06): images and K come from the driver's topics,
the camera's pose from so101_perception's extrinsics file. A timer takes each new image once: the grid's
markers, its pose (board.grid_in_camera), the 42 cells (board.read), the board's validation.

- /connect4/board: only once 5 reads in a row agree, each on a new image; its stamp is the first image's of
  the streak, so the game can reject a read older than its last move. An unreadable or impossible grid is
  published too, `valid` false with its reason (markers:<n>, out_of_view, gravity:<column>, counts:<h>-<r>).
- /connect4/grid_pose and the TF base_link -> connect4_grid: at each image whose markers give a pose.
- /connect4/debug_image: at each image.
"""

import json
import signal
import threading
from pathlib import Path

import numpy as np
import rclpy
import rclpy.node
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from so101_common.robot import BASE_FRAME
from so101_connect4 import board, layout
from so101_interfaces.msg import Connect4Board
from so101_perception.calibration import detect_markers
from so101_perception.geometry import load_extrinsics, quaternion

GRID_FRAME = "connect4_grid"
# Reliable, the newest only. A 640x480 image is 0.9 MB: a best-effort subscriber gets 1 of the 10 Hz [MES-67].
NEWEST = QoSProfile(depth=1)
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class BoardReaderNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("board_reader")

        def declare(name, default, description):
            descriptor = ParameterDescriptor(description=description, read_only=True)
            return self.declare_parameter(name, default, descriptor).value

        rate = declare("process_rate_hz", 10.0, "How often a new image is looked for, Hz")
        reads = declare("stable_reads", 5, "Identical reads in a row before /connect4/board")
        self._threshold = declare("token_threshold", board.THRESHOLD, "Share of a cell's disc in a color")
        extrinsics = declare("extrinsics_file", "", "Camera pose; empty: so101_perception's")
        layout_file = declare("layout_file", "", "Grid layout; empty: the installed connect4_grid.yaml")
        if unknown := sorted(set(self._parameter_overrides) - set(self._parameters)):  # rclpy: no public API
            raise ValueError(f"Unknown parameters {unknown}: a typo in a config file?")
        perception = Path(get_package_share_directory("so101_perception")) / "config"
        self._T_base_cam = load_extrinsics(extrinsics or perception / "camera_extrinsics.yaml").T_base_cam
        self._layout = layout.load(layout_file or None)
        self._stable = board.Stable(reads)
        self._bridge = CvBridge()
        self._image = None  # latest image not processed yet
        self._camera_info = None
        self._game = {}  # the last /connect4/state, for the debug image

        self.create_subscription(Image, "/external_cam/image_raw", self._cb_image, NEWEST)
        self.create_subscription(CameraInfo, "/external_cam/camera_info", self._cb_camera_info, NEWEST)
        self.create_subscription(String, "/connect4/state", self._cb_state, LATCHED)
        self._pub_board = self.create_publisher(Connect4Board, "/connect4/board", NEWEST)
        self._pub_pose = self.create_publisher(PoseStamped, "/connect4/grid_pose", LATCHED)
        self._pub_debug = self.create_publisher(Image, "/connect4/debug_image", NEWEST)
        self._tf = TransformBroadcaster(self)
        # Steady clock: a step back of the system clock (WSL2, MES-37) would pause a ROS-clock timer.
        self.create_timer(1.0 / rate, self._process, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info("Board reader ready.")

    def _cb_image(self, msg: Image):
        self._image = msg

    def _cb_camera_info(self, msg: CameraInfo):
        self._camera_info = msg

    def _cb_state(self, msg: String):
        try:
            self._game = json.loads(msg.data)
        except ValueError:
            self._game = {}

    def _process(self):
        image, info = self._image, self._camera_info
        if image is None or info is None:
            return
        self._image = None
        rgb = self._bridge.imgmsg_to_cv2(image, desired_encoding="rgb8")
        K = np.array(info.k).reshape(3, 3)
        found = detect_markers(rgb, self._layout["markers"]["dictionary"])
        pose = board.grid_in_camera(found, self._layout, K)
        cells = None
        if pose is None:
            seen = sum(i in found for i in self._layout["markers"]["ids"])
            valid, reason = False, f"markers:{seen}"
        else:
            cells = board.read(rgb, self._layout, pose[0], K, threshold=self._threshold)
            valid, reason = (False, "out_of_view") if cells is None else board.validate(cells)
            self._publish_pose(image.header.stamp, self._T_base_cam @ pose[0])
        stamp = Time.from_msg(image.header.stamp).nanoseconds
        stable = self._stable.update(stamp, (cells or (board.EMPTY,) * 42, valid, reason))
        if stable:
            first, (stable_cells, stable_valid, stable_reason) = stable
            msg = Connect4Board(cells=stable_cells, valid=stable_valid, reason=stable_reason)
            msg.header.stamp, msg.header.frame_id = Time(nanoseconds=first).to_msg(), GRID_FRAME
            self._pub_board.publish(msg)
        self._publish_debug(image, rgb, None if pose is None else pose[0], K, cells, valid, reason, stable)

    def _publish_pose(self, stamp, T_base_grid):
        x, y, z, w = (float(v) for v in quaternion(T_base_grid[:3, :3]))
        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = stamp, BASE_FRAME
        position, orientation = pose.pose.position, pose.pose.orientation
        position.x, position.y, position.z = (float(v) for v in T_base_grid[:3, 3])
        orientation.x, orientation.y, orientation.z, orientation.w = x, y, z, w
        self._pub_pose.publish(pose)
        tf = TransformStamped()
        tf.header, tf.child_frame_id = pose.header, GRID_FRAME
        translation, rotation = tf.transform.translation, tf.transform.rotation
        translation.x, translation.y, translation.z = position.x, position.y, position.z
        rotation.x, rotation.y, rotation.z, rotation.w = x, y, z, w
        self._tf.sendTransform(tf)

    def _publish_debug(self, image, rgb, T_cam_grid, K, cells, valid, reason, stable):
        text = f"board {'VALID' if valid else reason}{'' if stable else ' (not stable yet)'}"
        if state := self._game.get("state"):
            text += f"  game {state}"
        out = rgb if T_cam_grid is None else board.draw(rgb, self._layout, T_cam_grid, K, cells, "")
        out = board.label(out, text)
        debug = self._bridge.cv2_to_imgmsg(out, encoding="rgb8")
        debug.header = image.header
        self._pub_debug.publish(debug)


def main(args=None):
    # SIGINT and SIGTERM only set an event, as the driver and perception (#158, #177): rclpy's handlers raise
    # at any bytecode.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())  # before the node: its "ready" log is the start
    node = BoardReaderNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while not stop.is_set():
            executor.spin_once(timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
