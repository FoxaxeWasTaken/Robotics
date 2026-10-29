"""The calibrate_extrinsics entry point (3.4b, #45) against a fake camera in this process: synthetic images
of ArUco markers seen from a known pose, and the driver's camera_info."""

import threading

import cv2
import numpy as np
import pytest
import rclpy
import yaml
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from so101_common.markers import marker_corners
from so101_perception.calibrate_extrinsics import main
from so101_perception.geometry import load_extrinsics

W, H = 640, 480
K = np.array([[500.0, 0, (W - 1) / 2], [0, 500.0, (H - 1) / 2], [0, 0, 1]])  # OpenCV pixel centers [MES-41]
T_TRUE = np.eye(4)  # 0.6 m above the work area, looking down; the markers' up (+x) is the image's up
T_TRUE[:3, :3] = np.column_stack([[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
T_TRUE[:3, 3] = [0.2, -0.1, 0.6]
LAYOUT = {
    "dictionary": "DICT_4X4_50",
    "side_m": 0.075,
    "max_reprojection_px": 1.0,
    "markers": {0: [0.1, -0.2], 1: [0.3, -0.2], 2: [0.1, 0.0], 3: [0.3, 0.0], 4: [0.2, -0.1]},
}


def view(layout):
    """The rgb8 image of the layout's markers on a white table, from T_TRUE."""
    canvas = np.full((H, W), 255, np.uint8)
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, layout["dictionary"]))
    size = 120
    square = np.float32([[-0.5, -0.5], [size - 0.5, -0.5], [size - 0.5, size - 0.5], [-0.5, size - 0.5]])
    for i, center in layout["markers"].items():
        camera = (marker_corners(center, layout["side_m"]) - T_TRUE[:3, 3]) @ T_TRUE[:3, :3]
        uv = (camera @ K.T)[:, :2] / camera[:, 2:]
        homography = cv2.getPerspectiveTransform(square, uv.astype(np.float32))
        marker = cv2.aruco.generateImageMarker(dictionary, i, size)
        canvas = np.minimum(canvas, cv2.warpPerspective(marker, homography, (W, H), borderValue=255))
    return cv2.cvtColor(canvas, cv2.COLOR_GRAY2RGB)


@pytest.fixture
def camera():
    """The driver's image and camera_info topics, from a node on its own context and thread."""
    context = rclpy.Context()
    rclpy.init(context=context)
    node = rclpy.create_node("fake_camera", context=context)
    qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
    images = node.create_publisher(Image, "/external_cam/image_raw", qos)
    infos = node.create_publisher(CameraInfo, "/external_cam/camera_info", qos)
    frame = {"rgb": view(LAYOUT)}

    def publish():
        image = Image(height=H, width=W, encoding="rgb8", step=3 * W, data=frame["rgb"].tobytes())
        images.publish(image)
        infos.publish(CameraInfo(width=W, height=H, k=K.ravel().tolist()))

    node.create_timer(0.1, publish)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    yield frame
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown(context=context)
    thread.join(timeout=5.0)


def run(tmp_path, layout):
    markers = tmp_path / "markers.yaml"
    markers.write_text(yaml.safe_dump(layout))
    out = tmp_path / "extrinsics.yaml"
    return main(
        [
            "calibrate_extrinsics",
            "--markers",
            str(markers),
            "--out",
            str(out),
            "--frames",
            "3",
            "--timeout",
            "20",
        ]
    ), out


def test_the_tool_writes_the_camera_pose(camera, tmp_path):
    code, out = run(tmp_path, LAYOUT)
    assert code == 0
    extrinsics = load_extrinsics(out)
    assert np.linalg.norm(extrinsics.T_base_cam[:3, 3] - T_TRUE[:3, 3]) < 0.005
    angle = np.degrees(np.linalg.norm(cv2.Rodrigues(extrinsics.T_base_cam[:3, :3].T @ T_TRUE[:3, :3])[0]))
    assert angle < 0.3
    assert extrinsics.reprojection_error_px < 1.0
    assert "markers [0, 1, 2, 3, 4], 3 frames at 640x480" in extrinsics.method


def test_too_few_markers_exit_1_and_write_nothing(camera, tmp_path):
    camera["rgb"] = view({**LAYOUT, "markers": {0: [0.1, -0.2], 1: [0.3, -0.2]}})
    code, out = run(tmp_path, LAYOUT)
    assert code == 1
    assert not out.exists()


def test_no_camera_exits_1_after_the_timeout(tmp_path):
    markers = tmp_path / "markers.yaml"
    markers.write_text(yaml.safe_dump(LAYOUT))
    out = tmp_path / "extrinsics.yaml"
    assert main(["calibrate_extrinsics", "--markers", str(markers), "--out", str(out), "--timeout", "1"]) == 1
    assert not out.exists()
