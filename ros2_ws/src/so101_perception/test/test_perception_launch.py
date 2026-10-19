"""Component test (launch_testing): perception.launch.py alone, fed by a fake camera at 10 Hz (no driver,
no sim). Each image comes back once on /perception/debug_image, annotated, with the QoS of 01 §3.1."""

import time
import unittest
from pathlib import Path

import launch
import launch.actions
import launch.launch_description_sources
import launch_testing.actions
import numpy as np
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from so101_common.robot import CAMERA_FRAME

TIMEOUT = 20.0  # s
IMAGES = 30  # sent after the first debug image


@pytest.mark.launch_test
def generate_test_description():
    launch_file = Path(get_package_share_directory("so101_perception")) / "launch" / "perception.launch.py"
    perception = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(str(launch_file))
    )
    return launch.LaunchDescription([perception, launch_testing.actions.ReadyToTest()])


def key(stamp):
    return stamp.sec, stamp.nanosec


class TestPerception(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_perception")
        cls.bridge = CvBridge()
        cls.rgb = np.random.default_rng(0).integers(0, 256, size=(240, 320, 3), dtype=np.uint8)
        cls.sent, cls.debug = [], []
        # The driver's publishers: Reliable, depth 2 (01 §3.1).
        cls.pub_image = cls.node.create_publisher(Image, "/external_cam/image_raw", 2)
        cls.pub_info = cls.node.create_publisher(CameraInfo, "/external_cam/camera_info", 2)
        best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        cls.node.create_subscription(Image, "/perception/debug_image", cls.debug.append, best_effort)
        cls.node.create_timer(0.1, cls.publish_camera)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def publish_camera(cls):
        image = cls.bridge.cv2_to_imgmsg(cls.rgb, encoding="rgb8")
        image.header.stamp = cls.node.get_clock().now().to_msg()
        image.header.frame_id = CAMERA_FRAME
        info = CameraInfo(header=image.header, width=320, height=240)
        cls.pub_image.publish(image)
        cls.pub_info.publish(info)
        cls.sent.append(key(image.header.stamp))

    def spin_until(self, condition, what):
        end = time.monotonic() + TIMEOUT
        while not condition() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(condition(), what)

    def test_1_each_image_comes_back_once_annotated(self):
        self.spin_until(lambda: self.debug, "no /perception/debug_image")
        first = len(self.sent)
        self.spin_until(lambda: len(self.sent) >= first + IMAGES, "the fake camera stalled")
        sent = set(self.sent)
        stamps = [key(msg.header.stamp) for msg in self.debug]
        self.assertTrue(set(stamps) <= sent, "a debug image does not carry the stamp of an input image")
        self.assertEqual(len(stamps), len(set(stamps)), "an image was processed twice")
        later = set(self.sent[first:])
        back = sum(stamp in later for stamp in stamps)
        self.assertGreaterEqual(back, 0.8 * IMAGES, f"{back} of {IMAGES} images came back")

        msg = self.debug[-1]
        self.assertEqual(msg.header.frame_id, CAMERA_FRAME)
        self.assertEqual((msg.encoding, msg.width, msg.height), ("rgb8", 320, 240))
        out = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        self.assertTrue(np.any(out[:30, :160] != self.rgb[:30, :160]), "no status drawn")
        np.testing.assert_array_equal(out[30:], self.rgb[30:])

    def test_2_qos_of_01_section_3_1(self):
        self.spin_until(lambda: self.debug, "no /perception/debug_image")
        publishers = self.node.get_publishers_info_by_topic("/perception/debug_image")
        self.assertEqual([p.node_name for p in publishers], ["so101_perception"])
        self.assertEqual(publishers[0].qos_profile.reliability, ReliabilityPolicy.BEST_EFFORT)
        image_subs = [
            s
            for s in self.node.get_subscriptions_info_by_topic("/external_cam/image_raw")
            if s.node_name == "so101_perception"
        ]
        self.assertEqual(len(image_subs), 1)
        # Depth 1 is not checked: DDS discovery does not carry the history depth.
        self.assertEqual(image_subs[0].qos_profile.reliability, ReliabilityPolicy.BEST_EFFORT)
        info_subs = self.node.get_subscriptions_info_by_topic("/external_cam/camera_info")
        self.assertIn("so101_perception", [s.node_name for s in info_subs])
