"""Component test (launch_testing): one move of the game on the sim (#84, #85, targets P2-02 and P2-03 of
phase2/P2-01 §4). The stack of sim.launch.py connect4:=true with the board reader: the driver on the game's
scene, the brain in game mode. The test is the game node and the person: /brain/handover, the robot's token,
then /brain/drop on board.column_target of a random column, the grid where the camera sees it (#260: the
brain refuses a drop without the measured pose). The token is taken by /brain/pick at the dispenser's
highest station that holds one (#259), the dispenser where the camera sees it too;
CONNECT4_TOKEN_SOURCE=hand gives it with /sim/connect4/give_token instead, as before the dispenser.
Empty jaws first: no_token, the arm still. The ground truth (the true board) only scores (C-06).
CONNECT4_DROPS=50 runs the measurement of MES-69 (hand) or MES-77 (dispenser); colcon test drops one token."""

import os
import re
import time
import unittest

import launch
import launch.launch_description_sources
import launch_testing.actions
import numpy as np
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import Log
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from test_connect4_scene import HANDOVER, LAYOUT

from so101_connect4 import board
from so101_interfaces.msg import Connect4Board
from so101_interfaces.srv import Drop

DROPS = int(os.environ.get("CONNECT4_DROPS", "1"))
SOURCE = os.environ.get("CONNECT4_TOKEN_SOURCE", "dispenser")
RAILS, PER_RAIL = LAYOUT["dispenser"]["rails"], LAYOUT["dispenser"]["tokens_per_rail"]
STARTUP_TIMEOUT = 60.0  # s
MOVE_TIMEOUT = 40.0  # s: a move lasts about 15, a pick about 10
LAND_TIMEOUT = 5.0  # s, from the reply: the token fell while the arm came back
ARM = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
ROBOT = 2
ROBOT_TOKENS = 21  # in the scene: the board is cleared when they are all played
LATCH = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


@pytest.mark.launch_test
def generate_test_description():
    sim = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("so101_bringup"), "launch", "sim.launch.py")
        ),
        launch_arguments={"connect4": "true", "expose_ground_truth": "true", "perception": "true"}.items(),
    )
    return launch.LaunchDescription([sim, launch_testing.actions.ReadyToTest()])


class TestConnect4Drop(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_connect4_drop")
        cls.poses, cls.joints, cls.truth, cls.logs = [], [], [], []
        cls.frames = {}  # the grid's and the dispenser's frames in base_link, as the camera sees them
        for name in ("grid", "dispenser"):
            cls.node.create_subscription(
                PoseStamped,
                f"/connect4/{name}_pose",
                lambda m, name=name: cls.frames.update({name: m}),
                LATCH,
            )
        cls.node.create_subscription(PoseStamped, "/end_effector_pose", cls.poses.append, 10)
        cls.node.create_subscription(JointState, "/joint_states", cls.joints.append, 10)
        cls.node.create_subscription(Connect4Board, "/sim/ground_truth/connect4_board", cls.truth.append, 10)
        cls.node.create_subscription(Log, "/rosout", lambda m: cls.logs.append(m.msg), 100)
        cls.handover = cls.node.create_client(Trigger, "/brain/handover")
        cls.drop = cls.node.create_client(Drop, "/brain/drop")
        cls.pick = cls.node.create_client(Drop, "/brain/pick")
        cls.give = cls.node.create_client(Trigger, "/sim/connect4/give_token")
        cls.reset = cls.node.create_client(Trigger, "/sim/connect4/reset_board")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def call(self, client, request=None, timeout=MOVE_TIMEOUT):
        self.assertTrue(client.wait_for_service(timeout_sec=STARTUP_TIMEOUT), f"{client.srv_name} not served")
        future = client.call_async(request or client.srv_type.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        self.assertTrue(future.done(), f"no reply from {client.srv_name} in {timeout} s")
        return future.result()

    def arm(self):
        positions = dict(zip(self.joints[-1].name, self.joints[-1].position, strict=True))
        return np.array([positions[name] for name in ARM])

    def cells(self):
        """The true board, from a message newer than this call."""
        seen, end = len(self.truth), time.monotonic() + LAND_TIMEOUT
        while len(self.truth) == seen and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return list(self.truth[-1].cells)

    def seen(self, name) -> np.ndarray:
        """The frame of the grid or of the dispenser where the camera sees it now."""
        p, o = self.frames[name].pose.position, self.frames[name].pose.orientation
        return board.transform((p.x, p.y, p.z), (o.x, o.y, o.z, o.w))

    @staticmethod
    def to(target):
        request = Drop.Request()
        request.target.x, request.target.y, request.target.z = (float(v) for v in target)
        return request

    def play(self, column, played):
        """One move into `column`, the `played`-th token since the dispenser was full: the reply of
        /brain/drop, or of a /brain/pick that failed, and the duration in s of the drop alone."""
        if SOURCE == "hand":
            given = self.call(self.give)
            self.assertTrue(given.success, given.message)
        else:  # the highest station that holds a token, as the game node chooses it
            rail = RAILS - 1 - played // PER_RAIL
            reply = self.call(self.pick, self.to(board.station_target(LAYOUT, self.seen("dispenser"), rail)))
            if not reply.success:
                return reply, 0.0
        start = time.monotonic()
        target = board.column_target(LAYOUT, self.seen("grid"), column)
        return self.call(self.drop, self.to(target)), time.monotonic() - start

    def test_empty_jaws_are_refused_then_tokens_go_into_random_columns(self):
        end = time.monotonic() + STARTUP_TIMEOUT
        # the brain has /joint_states (AGENTS.md), the camera has placed the grid and the dispenser
        while not (self.poses and len(self.frames) == 2) and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(self.poses, "no /end_effector_pose")
        self.assertEqual(sorted(self.frames), ["dispenser", "grid"])
        rclpy.spin_once(self.node, timeout_sec=1.0)  # the brain gets the same poses
        reply = self.call(self.handover)
        self.assertTrue(reply.success, reply.message)
        np.testing.assert_allclose(self.arm(), HANDOVER, atol=0.02)

        # P2-03, the other half: without a token, 10 tries, no_token, the arm never moves
        request = Drop.Request()
        target = board.column_target(LAYOUT, self.seen("grid"), 3)
        request.target.x, request.target.y, request.target.z = (float(v) for v in target)
        seen = len(self.joints)
        for _ in range(10):
            reply = self.call(self.drop, request)
            self.assertEqual((reply.success, reply.message), (False, "no_token"))
        moved = max(
            np.max(np.abs(np.array(dict(zip(m.name, m.position, strict=True))[name]) - q))
            for m in self.joints[seen:]
            for name, q in zip(ARM, HANDOVER, strict=True)
        )
        self.assertLess(moved, 0.02, "the arm moved without a token")

        rng = np.random.default_rng(84)
        heights, landed, held, seconds, played = [0] * 7, 0, 0, [], 0
        by_column = {}
        for k in range(DROPS):
            if played == ROBOT_TOKENS:
                self.assertTrue(self.call(self.reset).success)
                heights, played = [0] * 7, 0
            column = int(rng.choice([c for c in range(7) if heights[c] < 6]))
            reply, duration = self.play(column, played)
            played += 1
            held += reply.message != "no_token"
            cell = heights[column] * 7 + column
            end = time.monotonic() + LAND_TIMEOUT
            cells = self.cells()
            while cells[cell] != ROBOT and time.monotonic() < end:
                cells = self.cells()
            ok = reply.success and cells[cell] == ROBOT and sum(c == ROBOT for c in cells) == sum(heights) + 1
            if not ok:
                print(f"drop {k}, column {column}: {reply.message}; board {cells}")
                self.assertTrue(self.call(self.reset).success)  # a clean board for the next ones
                self.assertTrue(self.call(self.handover).success)
                heights, played = [0] * 7, 0
                continue
            landed += 1
            heights[column] += 1
            seconds.append(duration)
            by_column.setdefault(column, []).append(duration)
            np.testing.assert_allclose(self.arm(), HANDOVER, atol=0.02)  # replied at HANDOVER
        self.node.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.5))
        rclpy.spin_once(self.node, timeout_sec=0.1)
        ticks = [
            (float(p99), float(longest))
            for line in self.logs
            for p99, longest in re.findall(r"/brain/drop: done in .*p99 (\d+\.\d) ms, max (\d+\.\d) ms", line)
        ]
        p99, longest = (max(values, default=0.0) for values in zip(*ticks, strict=True)) if ticks else (0, 0)
        print(
            f"connect4 drops: {landed}/{DROPS} in their cell, token held {held}/{DROPS}; a move "
            f"{min(seconds, default=0):.1f} to {max(seconds, default=0):.1f} s; brain tick during the moves: "
            f"p99 {p99:.1f} ms, max {longest:.1f} ms"
        )

        def medians(values):
            return ", ".join(f"{key} {np.median(values[key]):.1f}" for key in values)

        print(f"connect4 drops, median s by column: {medians(dict(sorted(by_column.items())))}")
        for move in ("drop", "pick"):  # the seconds of each step, from the brain's log line of each move done
            steps, whole = {}, []
            for line in self.logs:
                for total, found in re.findall(rf"/brain/{move}: done in (\S+) s \((.*?)\)", line):
                    whole.append(float(total))
                    for name, value in (part.split() for part in found.split(", ")):
                        steps.setdefault(name, []).append(float(value))
            if whole:
                print(
                    f"connect4 {move}s: {len(whole)} done, {min(whole):.1f} to {max(whole):.1f} s; "
                    f"median s by step: {medians(steps)}"
                )
        self.assertGreaterEqual(landed, np.ceil(0.95 * DROPS))  # target P2-02
        self.assertGreaterEqual(held, np.ceil(0.98 * DROPS))  # target P2-03, the sim's part
