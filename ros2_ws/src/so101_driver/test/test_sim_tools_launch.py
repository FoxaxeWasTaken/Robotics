"""Component test (launch_testing): sim tools of the evaluation bench (docs/project/03 §7, ADR-018).

/sim/randomize moves the cube and the box; expose_ground_truth:=true publishes their true poses.
"""

import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

STARTUP_TIMEOUT = 30.0  # s
TOPICS = ("/sim/ground_truth/object", "/sim/ground_truth/drop_box")


@pytest.mark.launch_test
def generate_test_description():
    driver = launch_ros.actions.Node(
        package="so101_driver",
        executable="driver",
        parameters=[{"use_sim": True, "expose_ground_truth": True, "seed": 42}],
        output="screen",
    )
    return launch.LaunchDescription([driver, launch_testing.actions.ReadyToTest()]), {"driver": driver}


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def xyz(msg):
    p = msg.pose.position
    return np.array([p.x, p.y, p.z])


class TestSimTools(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_sim_tools")
        cls.truth = {topic: [] for topic in TOPICS}
        for topic in TOPICS:
            cls.node.create_subscription(PoseStamped, topic, cls.truth[topic].append, 10)
        cls.client = cls.node.create_client(Trigger, "/sim/randomize")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_until(self, condition, timeout, message):
        end = time.monotonic() + timeout
        while not condition() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(condition(), message)

    def poses_after(self, t):
        """Cube and box positions from the first ground-truth messages stamped after t."""

        def first_after(topic):
            return next((m for m in self.truth[topic] if seconds(m.header.stamp) > t), None)

        self.spin_until(lambda: all(first_after(topic) for topic in TOPICS), 5.0, "no ground truth after t")
        return tuple(xyz(first_after(topic)) for topic in TOPICS)

    def test_ground_truth_is_published_at_10_hz_in_base_link(self):
        self.spin_until(
            lambda: all(len(m) > 30 for m in self.truth.values()), STARTUP_TIMEOUT, "no ground truth"
        )
        for topic, msgs in self.truth.items():
            # Median period: a step of the system clock (WSL2 time sync) shifts one interval only.
            rate = 1.0 / np.median(np.diff([seconds(m.header.stamp) for m in msgs[-30:]]))
            print(f"{topic}: {rate:.1f} Hz, frame {msgs[-1].header.frame_id}, at {xyz(msgs[-1]).round(3)}")
            self.assertAlmostEqual(rate, 10.0, delta=1.0)
            self.assertEqual(msgs[-1].header.frame_id, "base_link")
        cube, box = xyz(self.truth[TOPICS[0]][-1]), xyz(self.truth[TOPICS[1]][-1])
        self.assertAlmostEqual(cube[2], 0.025, delta=0.005)  # resting on the floor: base_link is the world
        self.assertAlmostEqual(box[2], 0.0, delta=1e-6)

    def test_randomize_moves_the_cube_and_the_box(self):
        self.assertTrue(
            self.client.wait_for_service(timeout_sec=STARTUP_TIMEOUT), "/sim/randomize not served"
        )
        previous = self.poses_after(0.0)
        for _ in range(3):
            future = self.client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
            response = future.result()
            self.assertTrue(response.success, response.message)
            self.assertFalse(any(c.isdigit() for c in response.message), "the answer must carry no pose")
            cube, box = self.poses_after(self.node.get_clock().now().nanoseconds * 1e-9)
            print(
                f"cube {cube.round(3)}, box {box.round(3)}, {np.linalg.norm(cube[:2] - box[:2]):.3f} m apart"
            )
            self.assertGreater(np.linalg.norm(cube - previous[0]), 1e-3, "cube not moved")
            self.assertGreater(np.linalg.norm(box - previous[1]), 1e-3, "box not moved")
            self.assertGreaterEqual(np.linalg.norm(cube[:2] - box[:2]), 0.15)
            self.assertAlmostEqual(cube[2], 0.025, delta=0.005)  # settled, not inside a wall
            previous = cube, box


@launch_testing.post_shutdown_test()
class TestSimToolsShutdown(unittest.TestCase):
    def test_sigint_stops_the_driver_without_traceback(self, proc_info, proc_output, driver):
        launch_testing.asserts.assertExitCodes(proc_info, process=driver)
        self.assertNotIn(b"Traceback", b"".join(io.text for io in proc_output[driver]))
