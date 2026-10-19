"""Component test (launch_testing): brain_node alone. A fake arm in the test echoes /joint_command into
/joint_states, so no driver and no sim are needed (the sim-backed version is in so101_bringup)."""

import time
import unittest
from itertools import pairwise

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from so101_interfaces.srv import GoToTarget

STARTUP_TIMEOUT = 20.0  # s
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
V_MAX = 1.0  # rad/s: default of motion.v_max_rad_s


@pytest.mark.launch_test
def generate_test_description():
    brain = launch_ros.actions.Node(package="so101_brain", executable="brain", output="screen")
    return launch.LaunchDescription([brain, launch_testing.actions.ReadyToTest()]), {"brain": brain}


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class TestBrain(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_brain")
        cls.q = dict.fromkeys(JOINTS, 0.0)  # the fake arm: at q = 0, follows every command at once
        cls.commands, cls.poses, cls.sent = [], [], []
        cls.pub_states = cls.node.create_publisher(JointState, "/joint_states", 10)
        cls.node.create_subscription(JointState, "/joint_command", cls.on_command, 100)
        cls.node.create_subscription(PoseStamped, "/end_effector_pose", cls.poses.append, 10)
        cls.node.create_timer(1.0 / 30.0, cls.publish_states)
        cls.client = cls.node.create_client(GoToTarget, "/go_to_target")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def on_command(cls, msg):
        cls.commands.append(msg)
        cls.q.update(zip(msg.name, msg.position, strict=True))

    @classmethod
    def publish_states(cls):
        msg = JointState(name=JOINTS, position=[cls.q[n] for n in JOINTS])
        msg.header.stamp = cls.node.get_clock().now().to_msg()
        cls.sent.append(msg.header.stamp)
        cls.pub_states.publish(msg)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def wait_for_pose(self):
        end = time.monotonic() + STARTUP_TIMEOUT
        while not self.poses and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(self.poses, "no /end_effector_pose")

    def call(self, frame_id, xyz):
        self.assertTrue(self.client.wait_for_service(timeout_sec=STARTUP_TIMEOUT), "/go_to_target not served")
        request = GoToTarget.Request()
        request.target_pose.header.frame_id = frame_id
        p = request.target_pose.pose.position
        p.x, p.y, p.z = xyz
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.assertIsNotNone(future.result(), "no answer from /go_to_target")
        return future.result()

    def tcp(self):
        p = self.poses[-1].pose.position
        return np.array([p.x, p.y, p.z])

    def test_1_pose_is_the_fk_of_the_joint_states(self):
        self.wait_for_pose()
        pose = self.poses[-1]
        self.assertEqual(pose.header.frame_id, "base_link")
        self.assertIn(pose.header.stamp, self.sent)  # stamped with the joint states it comes from
        np.testing.assert_allclose(self.tcp(), [0.391, 0.0, 0.226], atol=1e-3)  # q = 0 (04 §1.1)

    def test_2_go_to_target_streams_until_reached(self):
        self.wait_for_pose()
        target = np.array([0.2, 0.0, 0.1])
        start = len(self.commands)
        response = self.call("base_link", target)
        self.assertTrue(response.success, response.message)
        end = time.monotonic() + 6.0
        while np.linalg.norm(self.tcp() - target) > 1e-3 and time.monotonic() < end:
            self.spin_for(0.1)
        self.assertLess(np.linalg.norm(self.tcp() - target), 1e-3)
        streamed = self.commands[start:]
        self.assertGreater(len(streamed), 20)
        self.assertTrue(
            all(list(m.name) == JOINTS[:5] for m in streamed)
        )  # arm only: the gripper keeps its goal
        speeds = [
            np.max(np.abs(np.subtract(b.position, a.position))) / (stamp(b) - stamp(a))
            for a, b in pairwise(streamed)
        ]
        self.assertLessEqual(max(speeds), V_MAX * 1.05)

    def test_3_rejected_targets(self):
        self.wait_for_pose()
        self.assertFalse(self.call("world", (0.2, 0.0, 0.1)).success)
        self.assertFalse(self.call("base_link", (1.0, 0.0, 0.0)).success)


@launch_testing.post_shutdown_test()
class TestBrainShutdown(unittest.TestCase):
    def test_sigint_stops_the_brain_without_traceback(self, proc_info, proc_output, brain):
        launch_testing.asserts.assertExitCodes(proc_info, process=brain)
        self.assertNotIn(b"Traceback", b"".join(io.text for io in proc_output[brain]))
