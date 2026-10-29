"""Component test (launch_testing): perception.launch.py alone, fed by a fake camera at 10 Hz (no driver,
no sim) that replays reference images with their K. Each image comes back once on /perception/debug_image,
annotated; the cube and box poses match the labels, and a truncated or a partly hidden cube is never published
(01 §3.1, #167)."""

import json
import time
import unittest
from pathlib import Path

import cv2
import launch
import launch.actions
import launch.launch_description_sources
import launch_testing.actions
import numpy as np
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from test_geometry import rotation
from tf2_msgs.msg import TFMessage

from so101_common.robot import BASE_FRAME, CAMERA_FRAME
from so101_perception.geometry import load_extrinsics

TIMEOUT = 20.0  # s
IMAGES = 30  # sent after the first debug image
DATA = Path(__file__).parent / "data"
LABELS = json.loads((DATA / "labels.json").read_text())
# Whole cube A, truncated cube, a cube the arm partly hides (050.png, 37 % visible, the box whole), whole
# cube B (B then repeats): each far from the others, so the filter
# rejects the first 4 frames after each change (05 §7). Each image is shown until the node has processed
# that many frames of it, not sent: a loaded node starts late and drops frames (#157).
SCRIPT = [("000.png", 15), ("015.png", 15), ("050.png", 15), ("024.png", None)]


@pytest.mark.launch_test
def generate_test_description():
    launch_file = Path(get_package_share_directory("so101_perception")) / "launch" / "perception.launch.py"
    perception = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(str(launch_file)),
        # the camera that rendered the reference images, not the sim's moved one (#47)
        launch_arguments={"extrinsics_file": str(DATA / "camera_extrinsics.yaml")}.items(),
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
        labels = {image["file"]: image for image in LABELS["images"]}
        cls.script = [(labels[name], cls.rgb(name), count) for name, count in SCRIPT]
        cls.sent, cls.debug, cls.cubes, cls.boxes = [], [], [], []
        cls.shown = {}  # stamp -> label of the image sent with it
        # The driver's publishers: Reliable, depth 2 (01 §3.1).
        cls.pub_image = cls.node.create_publisher(Image, "/external_cam/image_raw", 2)
        cls.pub_info = cls.node.create_publisher(CameraInfo, "/external_cam/camera_info", 2)
        best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        cls.node.create_subscription(Image, "/perception/debug_image", cls.debug.append, best_effort)
        cls.node.create_subscription(PoseStamped, "/object_position", cls.cubes.append, 10)
        cls.node.create_subscription(PoseStamped, "/drop_box_position", cls.boxes.append, 10)
        cls.node.create_timer(0.1, cls.publish_camera)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @staticmethod
    def rgb(name):
        return cv2.cvtColor(cv2.imread(str(DATA / name)), cv2.COLOR_BGR2RGB)

    @classmethod
    def publish_camera(cls):
        label, rgb, count = cls.script[0]
        if count is not None and sum(cls.shown[key(m.header.stamp)] is label for m in cls.debug) >= count:
            cls.script.pop(0)
            label, rgb, count = cls.script[0]
        image = cls.bridge.cv2_to_imgmsg(rgb, encoding="rgb8")
        image.header.stamp = cls.node.get_clock().now().to_msg()
        image.header.frame_id = CAMERA_FRAME
        info = CameraInfo(header=image.header, width=320, height=240, distortion_model="plumb_bob")
        info.k = [float(v) for row in LABELS["K"] for v in row]
        cls.pub_image.publish(image)
        cls.pub_info.publish(info)
        cls.sent.append(key(image.header.stamp))
        cls.shown[key(image.header.stamp)] = label

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
        self.assertLessEqual(set(stamps), sent, "a debug image does not carry the stamp of an input image")
        self.assertEqual(len(stamps), len(set(stamps)), "an image was processed twice")
        later = set(self.sent[first:])
        back = sum(stamp in later for stamp in stamps)
        self.assertGreaterEqual(back, 0.8 * IMAGES, f"{back} of {IMAGES} images came back")

        msg = self.debug[-1]
        self.assertEqual(msg.header.frame_id, CAMERA_FRAME)
        self.assertEqual((msg.encoding, msg.width, msg.height), ("rgb8", 320, 240))
        out = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        sent = self.rgb(self.shown[key(msg.header.stamp)]["file"])
        self.assertTrue(np.any(out[:30, :160] != sent[:30, :160]), "no status drawn")
        self.assertGreater(np.sum(np.any(out[30:] != sent[30:], axis=2)), 100, "no mask painted")

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
        for topic in ("/object_position", "/drop_box_position"):
            publishers = self.node.get_publishers_info_by_topic(topic)
            self.assertEqual([p.node_name for p in publishers], ["so101_perception"], topic)
            self.assertEqual(publishers[0].qos_profile.reliability, ReliabilityPolicy.RELIABLE, topic)

    def test_3_poses_match_the_labels_and_a_truncated_cube_is_never_published(self):
        """05 §8 and 01 §3.1: within 5 mm of the labels (cube center, box floor center), the cube yaw within
        5°, in base_link, stamped with their image; nothing for the truncated cube (015.png) nor for the
        partly hidden one (050.png, #167)."""
        b_frames = lambda: sum(self.shown.get(k, {}).get("file") == "024.png" for k in self.sent)  # noqa: E731
        self.spin_until(lambda: b_frames() >= 10, "the fake camera did not reach image B")
        self.spin_until(
            lambda: any(self.shown[key(m.header.stamp)]["file"] == "024.png" for m in self.cubes),
            "no cube pose for image B",
        )
        seen = set()
        poses = [(m, "cube_center") for m in self.cubes] + [(m, "box_floor_center") for m in self.boxes]
        for msg, center in poses:
            label = self.shown[key(msg.header.stamp)]
            self.assertEqual(msg.header.frame_id, BASE_FRAME)
            p, q = msg.pose.position, msg.pose.orientation
            if center == "box_floor_center":
                self.assertEqual((q.x, q.y, q.z, q.w), (0.0, 0.0, 0.0, 1.0))
            else:  # the cube yaw about z, 90°-symmetric (#43)
                self.assertEqual((q.x, q.y), (0.0, 0.0))
                self.assertAlmostEqual(q.z**2 + q.w**2, 1.0)
                yaw = 2.0 * np.arctan2(q.z, q.w)
                error = (yaw - label["cube_yaw"] + np.pi / 4) % (np.pi / 2) - np.pi / 4
                self.assertLess(abs(np.degrees(error)), 5.0, f"cube yaw of {label['file']}")
            error = np.linalg.norm(np.array([p.x, p.y, p.z]) - label[center])
            # the box of 015.png and 050.png (8.1 mm, measured): images for the cube, not the box
            limit = (
                0.01 if center == "box_floor_center" and label["file"] in ("015.png", "050.png") else 0.005
            )
            self.assertLess(error, limit, f"{center} of {label['file']}: {error * 1e3:.1f} mm")
            seen.add((center, label["file"]))
        self.assertNotIn(("cube_center", "015.png"), seen, "a truncated cube was published")
        self.assertNotIn(("cube_center", "050.png"), seen, "a partly hidden cube was published")
        self.assertLessEqual({("cube_center", "000.png"), ("box_floor_center", "000.png")}, seen)

    def test_4_the_camera_tf_is_the_pose_of_the_extrinsics_file(self):
        """#46: perception publishes the static TF base_link -> camera of the file it loaded (01 §4)."""
        tfs = []
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(TFMessage, "/tf_static", tfs.append, latched)
        self.spin_until(lambda: tfs, "no /tf_static")
        tf = next(t for m in tfs for t in m.transforms if t.child_frame_id == CAMERA_FRAME)
        self.assertEqual(tf.header.frame_id, BASE_FRAME)
        T = load_extrinsics(DATA / "camera_extrinsics.yaml").T_base_cam
        t, r = tf.transform.translation, tf.transform.rotation
        np.testing.assert_allclose([t.x, t.y, t.z], T[:3, 3], atol=1e-9)
        np.testing.assert_allclose(rotation(r.x, r.y, r.z, r.w), T[:3, :3], atol=1e-9)
