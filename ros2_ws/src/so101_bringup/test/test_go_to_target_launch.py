"""Component test (launch_testing): /go_to_target streams a min-jerk trajectory (sim driver + brain)."""

import os
import time
import unittest
from itertools import pairwise

import launch
import launch.launch_description_sources
import launch_testing.actions
import numpy as np
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from so101_interfaces.srv import GoToTarget

STARTUP_TIMEOUT = 30.0  # s
V_MAX = 1.0  # rad/s: motion.v_max_rad_s in so101_bringup/config/brain.yaml
ARM = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")


@pytest.mark.launch_test
def generate_test_description():
    sim = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("so101_bringup"), "launch", "sim.launch.py")
        )
    )
    return launch.LaunchDescription([sim, launch_testing.actions.ReadyToTest()])


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class TestGoToTarget(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_go_to_target")
        cls.commands, cls.poses = [], []
        cls.node.create_subscription(JointState, "/joint_command", cls.commands.append, 100)
        cls.node.create_subscription(PoseStamped, "/end_effector_pose", cls.poses.append, 10)
        cls.client = cls.node.create_client(GoToTarget, "/go_to_target")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def tcp(self):
        p = self.poses[-1].pose.position
        return np.array([p.x, p.y, p.z])

    def go_to(self, target):
        self.assertTrue(self.client.wait_for_service(timeout_sec=STARTUP_TIMEOUT), "/go_to_target not served")
        end = time.monotonic() + STARTUP_TIMEOUT
        while not self.poses and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        request = GoToTarget.Request()
        request.target_pose.header.frame_id = "base_link"
        request.target_pose.pose.position.x, request.target_pose.pose.position.y = target[0], target[1]
        request.target_pose.pose.position.z = target[2]
        sent = time.monotonic()
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.assertIsNotNone(future.result(), "no answer from /go_to_target")
        self.assertTrue(future.result().success, future.result().message)
        return time.monotonic() - sent

    def wait_until_reached(self, target, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.spin_for(0.1)
            if np.linalg.norm(self.tcp() - target) < 1e-3:
                return
        self.fail(f"TCP at {self.tcp().round(4)}, target {target}")

    def test_1_motion_is_smooth_and_reaches_the_target(self):
        target = np.array([0.2, 0.0, 0.1])
        start = len(self.commands)
        answered_in = self.go_to(target)
        self.wait_until_reached(target, timeout=6.0)
        error = np.linalg.norm(self.tcp() - target)
        print(f"/go_to_target answered in {answered_in * 1000:.0f} ms; TCP error {error * 1000:.2f} mm")
        self.assertLess(answered_in, 0.5)  # the service never waits for the motion
        streamed = [m for m in self.commands[start:] if tuple(m.name) == ARM]
        self.assertGreater(len(streamed), 20, "no trajectory streamed on /joint_command")
        speeds = [
            np.max(np.abs(np.subtract(b.position, a.position))) / (stamp(b) - stamp(a))
            for a, b in pairwise(streamed)
        ]
        print(f"{len(streamed)} commands, peak joint speed {max(speeds):.3f} rad/s (v_max {V_MAX})")
        self.assertLessEqual(max(speeds), V_MAX * 1.05)

    def test_2_a_new_goal_preempts_the_running_one(self):
        self.go_to([0.25, 0.08, 0.12])
        self.spin_for(0.3)  # mid-motion
        target = np.array([0.18, -0.06, 0.08])
        self.go_to(target)
        self.wait_until_reached(target, timeout=6.0)
