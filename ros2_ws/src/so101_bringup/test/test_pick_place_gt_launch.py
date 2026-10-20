"""System test in sim (steps 4.2b, 4.2c): driver, brain and the ground-truth perception stub of
gt_stub.launch.py. /sim/randomize places the cube and the box, /brain/start runs a full cycle to DONE, and
the ground truth then has the cube in the box (success criterion of docs/project/08 §3)."""

import time
import unittest
from pathlib import Path

import launch
import launch.launch_description_sources
import launch_testing.actions
import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

STARTUP_TIMEOUT = 30.0  # s
CYCLE_TIMEOUT = 60.0  # s, as the evaluation bench (08 §3)
BOX_HALF_SIZE = 0.05  # m
SEQUENCE = ["WAIT_TARGET", "PLAN", "APPROACH", "DESCEND", "GRASP", "LIFT", "TRANSPORT", "RELEASE", "HOME"]


@pytest.mark.launch_test
def generate_test_description():
    stack = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            str(Path(__file__).with_name("gt_stub.launch.py"))
        )
    )
    return launch.LaunchDescription([stack, launch_testing.actions.ReadyToTest()])


def xyz(msg):
    p = msg.pose.position
    return np.array([p.x, p.y, p.z])


class TestPickPlace(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_pick_place")
        cls.states, cls.truth = [], {}
        latched = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        cls.node.create_subscription(String, "/brain_state", lambda m: cls.states.append(m.data), latched)
        for name in ("object", "drop_box"):
            cls.node.create_subscription(
                PoseStamped, f"/sim/ground_truth/{name}", lambda m, n=name: cls.truth.__setitem__(n, m), 10
            )
        cls.randomize = cls.node.create_client(Trigger, "/sim/randomize")
        cls.start = cls.node.create_client(Trigger, "/brain/start")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def call(self, client):
        self.assertTrue(client.wait_for_service(timeout_sec=STARTUP_TIMEOUT), f"{client.srv_name} not served")
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertIsNotNone(future.result(), f"no answer from {client.srv_name}")
        self.assertTrue(future.result().success, future.result().message)

    def spin_until(self, condition, timeout, what):
        end = time.monotonic() + timeout
        while not condition() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(condition(), f"{what}: {self.states[-12:]}")

    def test_a_cycle_puts_the_cube_in_the_box(self):
        self.call(self.randomize)
        self.spin_until(lambda: self.states and self.states[-1] == "IDLE", STARTUP_TIMEOUT, "no IDLE")
        begun, start = len(self.states), time.monotonic()
        self.call(self.start)
        self.spin_until(lambda: "WAIT_TARGET" in self.states[begun:], 5.0, "no WAIT_TARGET")
        self.spin_until(
            lambda: self.states[-1] in ("IDLE", "DONE") or self.states[-1].startswith("ERROR"),
            CYCLE_TIMEOUT,
            "the cycle did not end",
        )
        cycle = self.states[begun:]
        self.assertIn("DONE", cycle, "the cycle ended without DONE")  # DONE lasts one tick, then IDLE
        seen = [s for s in cycle if s in SEQUENCE]  # /brain_state also repeats the state at 1 Hz
        self.assertEqual([s for i, s in enumerate(seen) if i == 0 or seen[i - 1] != s], SEQUENCE)
        self.truth.clear()
        self.spin_until(lambda: len(self.truth) == 2, 2.0, "no ground truth")
        cube, box = xyz(self.truth["object"]), xyz(self.truth["drop_box"])
        print(f"cycle in {time.monotonic() - start:.1f} s: cube {np.round(cube, 3)}, box {np.round(box, 3)}")
        self.assertTrue(np.all(np.abs(cube[:2] - box[:2]) < BOX_HALF_SIZE), "cube outside the box footprint")
        self.assertLess(cube[2], 0.06)
