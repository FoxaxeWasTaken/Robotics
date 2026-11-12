"""/brain/drop and /brain/handover (#84, #85): the BrainNode in game mode (connect4.enabled), in this process
so that its coverage counts, with a fake arm that echoes /joint_command into /joint_states; its gripper effort
stands for a token between the jaws. No driver, no sim: the sim-backed drops are in so101_bringup."""

import re
import time
from pathlib import Path

import numpy as np
import pytest
import rclpy
import rclpy.node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import Log
from rclpy.clock import Clock, ClockType
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from test_drop import COLUMNS, token_centre

from so101_brain.brain_node import BrainNode
from so101_brain.fsm import State
from so101_brain.kinematics import Kinematics
from so101_common.robot import pct_to_rad
from so101_interfaces.srv import Drop, GoToTarget

STARTUP_TIMEOUT = 20.0  # s
MOVE_TIMEOUT = 30.0  # s: a move lasts about 15
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
HANDOVER = [-1.0, -0.6, 0.6, 1.1, 1.57]  # default of connect4.handover_pose
URDF = Path(get_package_share_directory("so101_description")) / "urdf" / "so101.urdf"
LATCHED = QoSProfile(
    depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
)


class FakeArm(rclpy.node.Node):
    """The driver's side of the brain, and the clients of its services."""

    def __init__(self):
        super().__init__("fake_arm")
        self.q = dict.fromkeys(JOINTS, 0.0)  # at q = 0, jaws closed
        self.alive = self.follows = True  # it publishes /joint_states; it follows the arm commands
        self.token = False  # a token between the jaws: an effort once they are commanded closed
        self.closed = True
        self.commands, self.poses, self.sent, self.states, self.logs = [], [], [], [], []
        self.released_at = None  # the arm joints when the jaws were last commanded open
        self.executor_ = None
        self.pub_states = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(JointState, "/joint_command", self.on_command, 100)
        self.create_subscription(PoseStamped, "/end_effector_pose", self.poses.append, 10)
        self.create_subscription(String, "/brain_state", lambda m: self.states.append(m.data), LATCHED)
        self.create_subscription(Log, "/rosout", lambda m: self.logs.append(m.msg), 100)
        # Steady clock: a WSL2 clock step would pause the fake driver, and the brain would see it as dead.
        self.create_timer(1.0 / 30.0, self.publish_states, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.drop = self.create_client(Drop, "/brain/drop")
        self.handover = self.create_client(Trigger, "/brain/handover")
        self.start = self.create_client(Trigger, "/brain/start")
        self.stop = self.create_client(Trigger, "/brain/stop")
        self.go_to_target = self.create_client(GoToTarget, "/go_to_target")

    def on_command(self, msg):
        positions = dict(zip(msg.name, msg.position, strict=True))
        self.commands.append((time.monotonic(), dict(positions)))  # (arrival, {joint: value})
        if "gripper" in positions:  # % in /joint_command, rad in /joint_states, as the driver (01 §3.1)
            closed = positions["gripper"] < 1.0
            if self.closed and not closed:
                self.released_at = self.arm()
            self.closed = closed
            self.q["gripper"] = pct_to_rad(positions.pop("gripper"))
        if self.follows:
            self.q.update(positions)

    def publish_states(self):
        if not self.alive:
            return
        msg = JointState(name=JOINTS, position=[self.q[n] for n in JOINTS])
        effort = 0.5 if self.token and self.closed else 0.0
        msg.effort = [effort if n == "gripper" else 0.0 for n in JOINTS]
        msg.header.stamp = self.get_clock().now().to_msg()
        self.sent.append(msg.header.stamp)
        self.pub_states.publish(msg)

    def arm(self):
        return np.array([self.q[n] for n in JOINTS[:5]])

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.executor_.spin_once(timeout_sec=0.01)

    def wait_for_fresh_joint_states(self):
        """Until a /end_effector_pose comes from /joint_states sent from now on: the brain sees the arm."""
        sent, end = len(self.sent), time.monotonic() + STARTUP_TIMEOUT
        while not any(pose.header.stamp in self.sent[sent:] for pose in self.poses[-3:]):
            assert time.monotonic() < end, "the brain does not see /joint_states"
            self.executor_.spin_once(timeout_sec=0.01)

    def request(self, client, request=None):
        """The pending call of a service of the brain."""
        assert client.wait_for_service(timeout_sec=STARTUP_TIMEOUT), f"{client.srv_name} not served"
        return client.call_async(request or client.srv_type.Request())

    def reply(self, future, timeout=MOVE_TIMEOUT):
        self.executor_.spin_until_future_complete(future, timeout_sec=timeout)
        assert future.done(), f"no reply in {timeout} s"
        return future.result()

    def call(self, client, request=None, timeout=MOVE_TIMEOUT):
        return self.reply(self.request(client, request), timeout)

    def to_handover(self):
        """The arm at HANDOVER, the jaws open, the brain seeing it."""
        self.alive = self.follows = True
        self.wait_for_fresh_joint_states()
        reply = self.call(self.handover)
        assert reply.success, reply.message

    def arm_commands(self, since):
        return [c for arrival, c in self.commands if arrival > since and "shoulder_pan" in c]


def drop_request(target=COLUMNS[3]):
    request = Drop.Request()
    request.target.x, request.target.y, request.target.z = (float(v) for v in target)
    return request


@pytest.fixture(scope="module")
def arm():
    rclpy.init(args=["--ros-args", "-p", "connect4.enabled:=true"])
    brain, fake = BrainNode(), FakeArm()
    fake.brain = brain
    fake.executor_ = SingleThreadedExecutor()
    fake.executor_.add_node(brain)
    fake.executor_.add_node(fake)
    yield fake
    fake.executor_.shutdown()
    fake.destroy_node()
    brain.destroy_node()
    rclpy.shutdown()


def test_a_drop_away_from_handover_is_refused_at_once(arm):
    arm.wait_for_fresh_joint_states()
    before, start = len(arm.commands), time.monotonic()
    reply = arm.call(arm.drop, drop_request(), timeout=3.0)
    assert (reply.success, reply.message) == (False, "not_at_handover")
    assert time.monotonic() - start < 1.0
    assert len(arm.commands) == before


def test_handover_opens_the_jaws_reaches_the_pose_and_replies_there(arm):
    arm.wait_for_fresh_joint_states()
    before = len(arm.commands)
    reply = arm.call(arm.handover)
    assert reply.success, reply.message
    assert re.fullmatch(r"done in \d+\.\d s", reply.message)
    np.testing.assert_allclose(arm.arm(), HANDOVER, atol=0.01)  # replied at the end, not at the start
    assert arm.commands[before][1].get("gripper") == 100.0
    assert set(arm.states) == {"IDLE"}


def test_a_drop_releases_the_token_above_the_column_and_replies_at_handover(arm):
    arm.to_handover()
    arm.token = True
    before = len(arm.commands)
    try:
        reply = arm.call(arm.drop, drop_request(COLUMNS[5]))
    finally:
        arm.token = False
    assert reply.success, reply.message
    assert re.fullmatch(r"done in \d+\.\d s", reply.message)
    np.testing.assert_allclose(arm.arm(), HANDOVER, atol=0.01)
    jaws = [c["gripper"] for _, c in arm.commands[before:] if "gripper" in c]
    assert jaws == [0.0, 100.0]  # closed on the token, opened once
    missed = np.linalg.norm(token_centre(Kinematics(str(URDF)), arm.released_at) - COLUMNS[5])
    assert missed < 0.002, "the jaws opened away from the target"
    assert set(arm.states) == {"IDLE"}  # /brain_state never leaves IDLE (P2-02 §6)
    # The tick kept its period meanwhile: the reply waits in its own callback group [MES-64]
    arm.spin_for(0.2)
    pattern = r"/brain/drop: done in .*tick period p99 (\d+\.\d) ms, max (\d+\.\d) ms"
    ticks = [found for line in arm.logs for found in re.findall(pattern, line)]
    assert ticks, "no tick line for the drop"
    print(f"tick period during a move: p99 {ticks[-1][0]} ms, max {ticks[-1][1]} ms")
    assert float(ticks[-1][0]) <= 25.0


def test_a_move_refuses_every_other_request_and_a_stop_cuts_it(arm):
    arm.to_handover()
    arm.token = True
    try:
        move = arm.request(arm.drop, drop_request())
        arm.spin_for(2.5)  # past the closing time: the arm moves
        assert not move.done()
        for client, request in ((arm.drop, drop_request()), (arm.handover, None)):
            reply = arm.call(client, request, timeout=3.0)
            assert (reply.success, reply.message) == (False, "busy")
        assert not arm.call(arm.start, timeout=3.0).success
        target = GoToTarget.Request()
        target.target_pose.pose.position.x, target.target_pose.pose.position.z = 0.2, 0.1
        assert not arm.call(arm.go_to_target, target, timeout=3.0).success
        assert not move.done()
        assert arm.call(arm.stop, timeout=3.0).success
        reply = arm.reply(move, timeout=3.0)
    finally:
        arm.token = False
    assert (reply.success, reply.message) == (False, "stopped")
    stopped_at, stopped = time.monotonic(), arm.arm()
    arm.spin_for(1.0)
    np.testing.assert_allclose(arm.arm(), stopped, atol=1e-6)  # frozen where the stop found it
    assert not arm.arm_commands(stopped_at + 0.2)
    assert arm.states[-1] == "IDLE"


def test_empty_jaws_are_no_token_ten_times_and_the_arm_never_moves(arm):
    arm.to_handover()
    since = time.monotonic()
    for _ in range(10):
        reply = arm.call(arm.drop, drop_request())
        assert (reply.success, reply.message) == (False, "no_token")
        assert arm.commands[-1][1] == {"gripper": 100.0}  # open again at the reply: a token can be given
    assert arm.arm_commands(since) == []
    np.testing.assert_allclose(arm.arm(), HANDOVER, atol=0.01)


def test_an_unreachable_target_is_refused_without_any_command(arm):
    arm.to_handover()
    before = len(arm.commands)
    reply = arm.call(arm.drop, drop_request((0.7, 0.0, 0.2)), timeout=3.0)
    assert (reply.success, reply.message) == (False, "unreachable")
    assert len(arm.commands) == before


def test_a_dead_driver_refuses_a_move_and_ends_a_running_one(arm):
    arm.to_handover()
    arm.token = True
    try:
        arm.alive = False
        arm.spin_for(1.0)  # /joint_states older than fsm.max_joint_state_age_s
        before = len(arm.commands)
        reply = arm.call(arm.drop, drop_request(), timeout=3.0)
        assert (reply.success, reply.message) == (False, "no_joint_states")
        assert len(arm.commands) == before
        arm.alive = True
        arm.wait_for_fresh_joint_states()
        move = arm.request(arm.drop, drop_request())
        arm.spin_for(2.5)
        assert not move.done()
        arm.alive = False
        reply = arm.reply(move, timeout=1.5)
        assert (reply.success, reply.message) == (False, "no_joint_states")
        ended = time.monotonic()
        arm.spin_for(0.5)
        assert [c for arrival, c in arm.commands if arrival > ended + 0.1] == []
    finally:
        arm.alive, arm.token = True, False


def test_an_arm_that_stops_following_ends_the_move_with_its_step(arm):
    arm.to_handover()
    arm.token = True
    try:
        move = arm.request(arm.drop, drop_request())
        arm.spin_for(1.2)  # past the closing time, in the first leg
        arm.follows = False
        reply = arm.reply(move, timeout=6.0)
    finally:
        arm.follows, arm.token = True, False
    assert (reply.success, reply.message) == (False, "timeout:rise")


def test_the_pick_and_place_cycle_is_refused_in_game_mode(arm):
    """No cube on the game's scene, and the way to PARK crosses the grid (so101_bringup's scene test)."""
    arm.to_handover()
    before = len(arm.commands)
    reply = arm.call(arm.start, timeout=3.0)
    assert not reply.success
    assert "game mode" in reply.message
    arm.spin_for(0.5)
    assert len(arm.commands) == before
    assert set(arm.states) <= {"IDLE", "STOPPED"}  # never a state of the cycle


def test_a_brain_that_is_not_idle_refuses_a_move(arm):
    arm.brain._fsm.state = State.STOPPED  # as in the tick after /brain/stop
    try:
        assert arm.brain._refusal() == "not_idle:STOPPED"
    finally:
        arm.brain._fsm.state = State.IDLE
