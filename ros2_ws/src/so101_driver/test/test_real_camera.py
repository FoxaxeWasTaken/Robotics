"""Real camera in the driver (step 6.2a, ADR-007): a webcam (`camera_device`) feeds /external_cam/image_raw in
rgb8 at 10 Hz, with the camera_info of a ROS calibration file; a missing camera leaves the driver running.
The webcam itself is replaced by a stand-in with OpenCVCamera's read_latest()."""

import threading
import time

import numpy as np
import pytest
import rclpy
from sensor_msgs.msg import CameraInfo

from so101_driver.driver_node import DriverNode, camera_info_from_yaml

CALIBRATION = """\
image_width: 640
image_height: 480
camera_name: external_cam
camera_matrix: {rows: 3, cols: 3, data: [600.0, 0.0, 319.5, 0.0, 601.0, 239.5, 0.0, 0.0, 1.0]}
distortion_model: plumb_bob
distortion_coefficients: {rows: 1, cols: 5, data: [0.1, -0.2, 0.0, 0.0, 0.05]}
rectification_matrix: {rows: 3, cols: 3, data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]}
projection_matrix:
  rows: 3
  cols: 4
  data: [590.0, 0.0, 319.0, 0.0, 0.0, 595.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
"""
# No camera at that path; the startup pose as shutdown pose: no shutdown move.
ARGS = ["--ros-args", "-p", "camera_device:=/dev/no_camera", "-p", "shutdown_pose:=[0.0,0.0,0.0,0.0,0.0]"]


class Webcam:
    """read_latest() of OpenCVCamera: the newest RGB frame, or TimeoutError when it is too old."""

    def __init__(self, *frames):
        self.frames = list(frames)

    def read_latest(self):
        frame = self.frames.pop(0) if len(self.frames) > 1 else self.frames[0]
        if isinstance(frame, Exception):
            raise frame
        return frame


@pytest.fixture
def calibration(tmp_path):
    path = tmp_path / "external_cam.yaml"
    path.write_text(CALIBRATION)
    return path


@pytest.fixture
def driver():
    rclpy.init(args=ARGS)
    node = DriverNode()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_camera_info_comes_from_a_ros_calibration_file(calibration):
    info = camera_info_from_yaml(calibration, 640, 480)
    assert (info.width, info.height, info.distortion_model) == (640, 480, "plumb_bob")
    assert list(info.k) == [600.0, 0.0, 319.5, 0.0, 601.0, 239.5, 0.0, 0.0, 1.0]
    assert list(info.d) == [0.1, -0.2, 0.0, 0.0, 0.05]
    assert list(info.p)[:3] == [590.0, 0.0, 319.0]  # P as calibrated, not rebuilt from K


def test_a_calibration_of_another_image_size_is_refused(calibration):
    with pytest.raises(ValueError, match="640x480"):
        camera_info_from_yaml(calibration, 320, 240)


def test_a_missing_camera_leaves_the_driver_running_without_images(driver):
    assert driver._camera_thread is None  # no sim render either: camera_device asks for the webcam
    driver.published = []
    driver._pub_joint_states.publish = driver.published.append
    driver._control_step()
    driver._publish_joint_states()
    assert len(driver.published) == 1


def test_webcam_frames_go_out_in_rgb8_with_their_camera_info(driver):
    images, infos = [], []
    driver._pub_image.publish = images.append
    driver._pub_camera_info.publish = infos.append
    red = np.zeros((480, 640, 3), np.uint8)
    red[..., 0] = 255  # RGB, as OpenCVCamera gives it
    webcam = Webcam(TimeoutError("frame too old"), red)  # a stale frame first: skipped, the loop goes on
    loop = threading.Thread(target=driver._real_camera_loop, args=(webcam, CameraInfo(width=640, height=480)))
    loop.start()
    deadline = time.monotonic() + 5.0
    while len(images) < 2 and time.monotonic() < deadline:
        time.sleep(0.05)
    driver._stop_camera.set()
    loop.join()
    assert len(images) >= 2
    image = images[-1]
    assert (image.encoding, image.width, image.height) == ("rgb8", 640, 480)
    assert bytes(image.data[:3]) == b"\xff\x00\x00"  # red stays red: no BGR swap
    assert infos[-1].header == image.header  # same stamp and frame_id (C-13)
