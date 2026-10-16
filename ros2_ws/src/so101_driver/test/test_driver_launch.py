"""Component test (launch_testing): the driver in sim publishes /joint_states and applies partial commands."""

import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from sensor_msgs.msg import JointState

STARTUP_TIMEOUT = 30.0  # s: MuJoCo loads the scene before the first message
ARM = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")  # URDF order, by hand
CLOSED, OPEN = -0.174533, 1.74533  # URDF gripper limits (rad): 0 % and 100 %


@pytest.mark.launch_test
def generate_test_description():
    driver = launch_ros.actions.Node(
        package="so101_driver", executable="driver", parameters=[{"use_sim": True}], output="screen"
    )
    return launch.LaunchDescription([driver, launch_testing.actions.ReadyToTest()])


class TestDriver(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_driver")
        cls.msgs = []
        cls.node.create_subscription(JointState, "/joint_states", cls.msgs.append, 10)
        cls.pub = cls.node.create_publisher(JointState, "/joint_command", 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def wait_for_joint_states(self):
        end = time.monotonic() + STARTUP_TIMEOUT
        while not self.msgs and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(self.msgs, "no /joint_states received")

    def latest(self):
        return dict(zip(self.msgs[-1].name, self.msgs[-1].position, strict=True))

    def test_joint_states_rate_is_at_least_20_hz(self):
        self.wait_for_joint_states()
        start = len(self.msgs)
        self.spin_for(3.0)
        stamps = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in self.msgs[start:]]
        rate = (len(stamps) - 1) / (stamps[-1] - stamps[0])
        print(f"/joint_states rate: {rate:.1f} Hz")
        self.assertGreaterEqual(rate, 20.0)

    def test_partial_command_moves_only_the_named_joint(self):
        self.wait_for_joint_states()
        self.spin_for(1.0)  # let the arm settle on its startup pose
        before = self.latest()
        end = time.monotonic() + 5.0
        while self.pub.get_subscription_count() == 0 and time.monotonic() < end:
            self.spin_for(0.1)
        target = before["shoulder_pan"] + 0.5
        start = len(self.msgs)
        self.pub.publish(JointState(name=["shoulder_pan"], position=[target]))
        self.spin_for(2.0)  # 0.5 rad at 1.5 rad/s: 0.33 s, plus settling
        after = self.latest()
        pan = [(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.position[0]) for m in self.msgs[start:]]
        t10 = next(t for t, p in pan if p - before["shoulder_pan"] >= 0.05)  # 10 % of the move
        t90 = next(t for t, p in pan if p - before["shoulder_pan"] >= 0.45)  # 90 %
        speed = 0.4 / (t90 - t10)
        print(
            f"shoulder_pan {before['shoulder_pan']:.4f} -> {after['shoulder_pan']:.4f} (target {target:.4f})"
        )
        print(f"shoulder_pan mean speed {speed:.2f} rad/s (limiter: 1.5)")
        self.assertAlmostEqual(after["shoulder_pan"], target, delta=0.02)
        self.assertLess(speed, 1.5 * 1.1)  # mean speed over 10-90 % of the move
        for name in before.keys() - {"shoulder_pan"}:
            self.assertLess(abs(after[name] - before[name]), 0.005, name)

    def test_unnamed_command_follows_joint_order_and_gripper_comes_back_in_rad(self):
        self.wait_for_joint_states()
        self.spin_for(1.0)
        arm = [self.latest()[n] for n in ARM]
        for pct, rad in ((100.0, OPEN), (0.0, CLOSED)):
            self.pub.publish(JointState(position=[*arm, pct]))  # no names: JOINT_NAMES order, gripper in %
            self.spin_for(2.0)
            print(f"gripper {pct:.0f} % -> {self.latest()['gripper']:.4f} rad (URDF limit {rad})")
            self.assertAlmostEqual(self.latest()["gripper"], rad, delta=0.05)
