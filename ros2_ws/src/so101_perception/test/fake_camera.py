"""A fake driver for the tests of the calibration tools: its image and camera_info topics, from a node on its
own context and thread, and the view of the ChArUco board from a known pose."""

import threading
from contextlib import contextmanager

import cv2
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from so101_perception.intrinsics import board

W, H = 640, 480
K = np.array([[500.0, 0, (W - 1) / 2], [0, 500.0, (H - 1) / 2], [0, 0, 1]])  # OpenCV pixel centers [MES-41]


def board_view(T_cam_board):
    """The rgb8 image of the board on a white table, from T_cam_board."""
    side = 50  # px per square in the board's own image
    w, h = board().getChessboardSize()
    length = board().getSquareLength()
    image = board().generateImage((w * side, h * side), marginSize=0, borderBits=1)
    outline = np.array([[0, 0, 0], [w, 0, 0], [w, h, 0], [0, h, 0]], float) * length
    rvec = cv2.Rodrigues(T_cam_board[:3, :3])[0]
    uv = cv2.projectPoints(outline, rvec, T_cam_board[:3, 3], K, None)[0].reshape(-1, 2)
    edges = np.float32(
        [[-0.5, -0.5], [w * side - 0.5, -0.5], [w * side - 0.5, h * side - 0.5], [-0.5, h * side - 0.5]]
    )
    homography = cv2.getPerspectiveTransform(edges, uv.astype(np.float32))
    gray = cv2.warpPerspective(image, homography, (W, H), flags=cv2.INTER_LINEAR, borderValue=255)
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)


@contextmanager
def fake_camera(next_frame, period_s=0.05):
    """Publishes next_frame(), rgb8, with the camera_info of K until the block ends."""
    context = rclpy.Context()
    rclpy.init(context=context)
    node = rclpy.create_node("fake_camera", context=context)
    qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
    images = node.create_publisher(Image, "/external_cam/image_raw", qos)
    infos = node.create_publisher(CameraInfo, "/external_cam/camera_info", qos)

    def publish():
        data = next_frame().tobytes()
        images.publish(Image(height=H, width=W, encoding="rgb8", step=3 * W, data=data))
        infos.publish(CameraInfo(width=W, height=H, k=K.ravel().tolist(), d=[0.0] * 5))

    node.create_timer(period_s, publish)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        yield
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
        thread.join(timeout=5.0)
