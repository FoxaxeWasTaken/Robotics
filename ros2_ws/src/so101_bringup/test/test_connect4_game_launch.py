"""System test (launch_testing, sim): the game's whole stack as real processes, sim.launch.py connect4:=true
with the board reader and the ground truth, as test_evaluate_launch.py does for the pick & place.

- The QoS of the game's topics is the contract's (01 §3.1), on the processes themselves: the package's tests
  run the nodes in-process, with their publishers replaced.
- The game bench's command line: a game cut by a short bench timeout fails, as `bench_timeout`, and keeps its
  rosbag; the arm is stopped for the next game.
"""

import csv
import json
import os
import subprocess
import tempfile
import time
import unittest
from pathlib import Path

import launch
import launch.actions
import launch.launch_description_sources
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from so101_bringup import games

STARTUP_TIMEOUT = 90.0  # s: the driver, the brain, the reader's first pose of the grid
RELIABLE, VOLATILE, LATCHED = (
    ReliabilityPolicy.RELIABLE,
    DurabilityPolicy.VOLATILE,
    DurabilityPolicy.TRANSIENT_LOCAL,
)
LATCH = QoSProfile(depth=1, durability=LATCHED)
PUBLISHED = {  # topic: (reliability, durability), 01 §3.1
    "/connect4/state": (RELIABLE, LATCHED),
    "/connect4/board": (RELIABLE, VOLATILE),
    "/connect4/grid_pose": (RELIABLE, LATCHED),
    "/connect4/debug_image": (RELIABLE, VOLATILE),
}


@pytest.mark.launch_test
def generate_test_description():
    sim = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("so101_bringup"), "launch", "sim.launch.py")
        ),
        launch_arguments={"connect4": "true", "perception": "true", "expose_ground_truth": "true"}.items(),
    )
    return launch.LaunchDescription([sim, launch_testing.actions.ReadyToTest()])


class TestConnect4Game(unittest.TestCase):
    def test_1_the_games_topics_have_the_qos_of_the_contract(self):
        rclpy.init()
        node = rclpy.create_node("test_connect4_game")
        try:
            end = time.monotonic() + STARTUP_TIMEOUT
            while time.monotonic() < end and not all(node.get_publishers_info_by_topic(t) for t in PUBLISHED):
                rclpy.spin_once(node, timeout_sec=0.2)
            for topic, (reliability, durability) in PUBLISHED.items():
                (publisher,) = node.get_publishers_info_by_topic(topic)
                qos = publisher.qos_profile
                self.assertEqual(qos.reliability, reliability, topic)
                self.assertEqual(qos.durability, durability, topic)
            # the reader takes every image: a best-effort subscriber gets 1 of the 10 Hz at 640x480 [MES-67]
            readers = [
                s.qos_profile.reliability
                for s in node.get_subscriptions_info_by_topic("/external_cam/image_raw")
                if s.node_name == "board_reader"
            ]
            self.assertEqual(readers, [RELIABLE])
        finally:
            node.destroy_node()
            rclpy.shutdown()

    def test_2_a_cut_game_fails_and_keeps_its_rosbag(self):
        with tempfile.TemporaryDirectory() as tmp:
            out, bags = Path(tmp) / "games.csv", Path(tmp) / "bags"
            evaluate = ["ros2", "run", "so101_bringup", "evaluate", "--games", "1", "--timeout", "20"]
            run = subprocess.run(
                [*evaluate, "--out", str(out), "--bags", str(bags)],
                capture_output=True,
                text=True,
                timeout=300,
            )
            self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
            with out.open() as f:
                (row,) = list(csv.DictReader(f))
            self.assertEqual(list(row), games.FIELDS)
            # 20 s: the person's token and one move of the robot at most; the bench stops the arm
            self.assertEqual((row["success"], row["cause"]), ("False", "bench_timeout"), row)
            self.assertNotIn(row["final_state"], ("GAME_OVER", "IDLE"), row)  # the state it was cut in
            self.assertIn("bench_timeout: 1", run.stdout)
            self.assertTrue((bags / "game_001" / "metadata.yaml").is_file(), list(bags.rglob("*")))
        # ... and the game node was told to stop: the arm is free for the next game
        rclpy.init()
        node = rclpy.create_node("test_connect4_game_after")
        states = []
        node.create_subscription(
            String, "/connect4/state", lambda m: states.append(json.loads(m.data)), LATCH
        )
        try:
            end = time.monotonic() + 10.0
            while time.monotonic() < end and not states:
                rclpy.spin_once(node, timeout_sec=0.2)
            self.assertEqual([(s["state"], s["reason"]) for s in states[-1:]], [("ERROR", "stopped")])
        finally:
            node.destroy_node()
            rclpy.shutdown()
