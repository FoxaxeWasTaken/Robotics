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
from test_drop import COLUMNS, DISPENSER, STATIONS, about_pan, frame, token_centre

from so101_brain import brain_node, drop
from so101_brain.brain_node import BrainNode
from so101_brain.fsm import State
from so101_brain.kinematics import Kinematics
from so101_common.robot import pct_to_rad
from so101_interfaces.srv import Drop, GoToTarget

STARTUP_TIMEOUT = 20.0  # s
MOVE_TIMEOUT = 30.0  # s: a move lasts about 15
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
HANDOVER = [-0.8, -0.6, 0.6, 1.1, 1.57]  # default of connect4.handover_pose
URDF = Path(get_package_share_directory("so101_description")) / "urdf" / "so101.urdf"
LATCHED = QoSProfile(
    depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
)
# Where the camera sees the grid (#260): a frame along its columns, here at column 0's release target. Its
# volume in that frame, as connect4.grid_boxes: the body (posts, plates, base) from the table to the slot
# entries, 55 mm under a held token's centre, then the depth guide above the columns.
GRID = frame(-40.0, COLUMNS[0])
GRID_BOXES = [
    -0.0635,
    -0.00775,
    -0.305,
    0.2855,
    0.00775,
    -0.055,
    -0.0185,
    -0.017,
    -0.055,
    0.2405,
    0.017,
    -0.025,
]


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
        self.closed_at = None  # and when they were last commanded closed
        self.executor_ = None
        self.pub_states = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(JointState, "/joint_command", self.on_command, 100)
        self.create_subscription(PoseStamped, "/end_effector_pose", self.poses.append, 10)
        self.create_subscription(String, "/brain_state", lambda m: self.states.append(m.data), LATCHED)
        self.create_subscription(Log, "/rosout", lambda m: self.logs.append(m.msg), 100)
        self.pub_poses = {
            name: self.create_publisher(PoseStamped, f"/connect4/{name}_pose", LATCHED)
            for name in ("grid", "dispenser")
        }
        # Steady clock: a WSL2 clock step would pause the fake driver, and the brain would see it as dead.
        self.create_timer(1.0 / 30.0, self.publish_states, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.drop = self.create_client(Drop, "/brain/drop")
        self.handover = self.create_client(Trigger, "/brain/handover")
        self.pick = self.create_client(Drop, "/brain/pick")
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
            if closed and not self.closed:
                self.closed_at = self.arm()
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

    def place(self, grid=None, dispenser=None):
        """The camera's side: where it sees the grid, the dispenser (frames in base_link), until the brain
        has them."""
        for name, T in (("grid", grid), ("dispenser", dispenser)):
            if T is None:
                continue
            msg, yaw = PoseStamped(), np.arctan2(T[1, 0], T[0, 0])
            msg.header.frame_id = "base_link"
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(v) for v in T[:3, 3])
            msg.pose.orientation.z, msg.pose.orientation.w = float(np.sin(yaw / 2)), float(np.cos(yaw / 2))
            self.pub_poses[name].publish(msg)
            end = time.monotonic() + STARTUP_TIMEOUT
            while (seen := getattr(self.brain, f"_{name}")) is None or not np.allclose(seen, T, atol=1e-9):
                assert time.monotonic() < end, f"the brain does not get the {name}'s pose"
                self.executor_.spin_once(timeout_sec=0.01)

    def arm_commands(self, since):
        return [c for arrival, c in self.commands if arrival > since and "shoulder_pan" in c]


def drop_request(target=COLUMNS[3], check_only=False):
    request = Drop.Request(check_only=check_only)
    request.target.x, request.target.y, request.target.z = (float(v) for v in target)
    return request


@pytest.fixture(scope="module")
def arm():
    boxes = f"connect4.grid_boxes:={GRID_BOXES}"
    rclpy.init(args=["--ros-args", "-p", "connect4.enabled:=true", "-p", boxes])
    brain, fake = BrainNode(), FakeArm()
    fake.brain = brain
    fake.executor_ = SingleThreadedExecutor()
    fake.executor_.add_node(brain)
    fake.executor_.add_node(fake)
    fake.place(grid=GRID, dispenser=DISPENSER)
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
    # ... and says where the move's time went, step by step (#247)
    steps = r"done in \d+\.\d s \(close \d+\.\d, rise \d+\.\d, column \d+\.\d, descent \d+\.\d, "
    steps += r"release \d+\.\d, lift \d+\.\d, clear \d+\.\d, handover \d+\.\d\);"
    assert any(re.search(steps, line) for line in arm.logs), arm.logs[-3:]


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


def test_a_drop_whose_poses_take_long_to_solve_still_starts(arm, monkeypatch):
    """The poses are solved in the service call, which holds the executor: the /joint_states that came
    meanwhile are still queued at the move's first tick. On a slow machine the solve took over
    fsm.max_joint_state_age_s, and the move ended there with `no_joint_states` (a CI runner, #252)."""
    solve = drop.drop_steps

    def slow(*args):
        time.sleep(0.8)  # longer than the age that makes joints stale, 0.5 s
        return solve(*args)

    monkeypatch.setattr(brain_node.drop, "drop_steps", slow)
    arm.to_handover()
    arm.token = True
    try:
        reply = arm.call(arm.drop, drop_request())
    finally:
        arm.token = False
    assert reply.success, reply.message


def test_a_request_right_after_a_long_check_is_not_refused_as_a_dead_driver(arm, monkeypatch):
    """A check_only request holds the executor for its IK, as a move's solve does: the /joint_states that
    came meanwhile are still queued when the next request is judged. The game node asks ten checks in a row
    at its start; a /brain/handover between two of them was refused, `no_joint_states` (found by
    test_connect4_drop_launch, #260)."""
    solve = drop.drop_steps

    def slow(*args):
        time.sleep(0.8)  # longer than the age that makes joints stale, 0.5 s
        return solve(*args)

    monkeypatch.setattr(brain_node.drop, "drop_steps", slow)
    arm.to_handover()
    for _ in range(3):  # whatever order the executor takes the queued callbacks in
        check = arm.request(arm.drop, drop_request(check_only=True))
        move = arm.request(arm.handover)  # waits behind the check
        assert arm.reply(check, timeout=5.0).success
        reply = arm.reply(move)
        assert reply.success, reply.message


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


# /brain/pick (#259): a token taken at a station of the dispenser, the same request as a drop's.


def test_a_pick_closes_the_jaws_on_the_stations_token_and_replies_at_handover_holding_it(arm):
    arm.to_handover()
    arm.token = True
    kin, since = Kinematics(str(URDF)), time.monotonic()
    reply = arm.call(arm.pick, drop_request(STATIONS[1]))
    assert reply.success, reply.message
    assert re.fullmatch(r"done in \d+\.\d s", reply.message)
    np.testing.assert_allclose(token_centre(kin, arm.closed_at), STATIONS[1], atol=2e-3)
    np.testing.assert_allclose(arm.arm(), HANDOVER, atol=0.01)
    assert arm.closed  # the token leaves with the arm
    grippers = [c["gripper"] for arrival, c in arm.commands if arrival > since and "gripper" in c]
    assert list(dict.fromkeys(grippers)) == [drop.PICK_OPEN_PCT, arm.brain._p.close_pct]
    line = next(text for text in reversed(arm.logs) if text.startswith("/brain/pick"))
    assert re.match(r"/brain/pick: done in \d+\.\d s \(above \d", line), line
    # the drop starts from there, the token in the jaws
    reply = arm.call(arm.drop, drop_request())
    assert reply.success, reply.message
    arm.token = False


def test_a_pick_on_an_empty_station_is_no_token_the_arm_back_at_handover_with_open_jaws(arm):
    arm.to_handover()
    reply = arm.call(arm.pick, drop_request(STATIONS[0]))
    assert (reply.success, reply.message) == (False, "no_token")
    np.testing.assert_allclose(arm.arm(), HANDOVER, atol=0.01)
    assert not arm.closed


def test_a_pick_away_from_handover_or_out_of_reach_is_refused_without_any_command(arm):
    arm.to_handover()
    before = len(arm.commands)
    reply = arm.call(arm.pick, drop_request((0.7, 0.0, 0.2)), timeout=3.0)
    assert (reply.success, reply.message) == (False, "unreachable")
    arm.q["shoulder_pan"] = 0.0  # the arm elsewhere: the way to a station starts at HANDOVER
    arm.wait_for_fresh_joint_states()
    reply = arm.call(arm.pick, drop_request(STATIONS[0]), timeout=3.0)
    assert (reply.success, reply.message) == (False, "not_at_handover")
    assert len(arm.commands) == before


def handover_line(arm):
    return next(text for text in reversed(arm.logs) if text.startswith("/brain/handover"))


def test_a_handover_after_a_pick_cut_short_rises_straight_up_first_and_not_after_a_drop(arm):
    """A stop may leave the arm down in a station: from there one joint motion sweeps the dispenser's stop
    [MES-77]. Above the grid the direct way is the measured one [MES-73]."""
    arm.to_handover()
    move = arm.request(arm.pick, drop_request(STATIONS[2]))
    arm.spin_for(3.0)  # on its way down into the station
    assert not move.done()
    assert arm.call(arm.stop, timeout=3.0).success
    assert arm.reply(move, timeout=3.0).message == "stopped"
    arm.spin_for(0.5)
    stopped = Kinematics(str(URDF)).fk(arm.arm())[:3, 3]
    since = time.monotonic()
    assert arm.call(arm.handover).success
    assert re.match(r"/brain/handover: done in \S+ s \(rise \d", handover_line(arm)), handover_line(arm)
    kin = Kinematics(str(URDF))
    first = [kin.fk([c[n] for n in JOINTS[:5]])[:3, 3] - stopped for c in arm.arm_commands(since)[:60]]
    assert max(np.hypot(d[0], d[1]) for d in first) < 0.002  # straight up
    assert max(d[2] for d in first) > 0.02
    np.testing.assert_allclose(arm.arm(), HANDOVER, atol=0.01)
    # the next handover, and one after a drop cut short, are one motion
    assert arm.call(arm.handover).success
    assert re.match(r"/brain/handover: done in \S+ s \(handover \d", handover_line(arm)), handover_line(arm)
    arm.token = True
    try:
        move = arm.request(arm.drop, drop_request())
        arm.spin_for(2.5)
        assert arm.call(arm.stop, timeout=3.0).success
        assert arm.reply(move, timeout=3.0).message == "stopped"
    finally:
        arm.token = False
    arm.spin_for(0.5)
    assert arm.call(arm.handover).success
    assert re.match(r"/brain/handover: done in \S+ s \(handover \d", handover_line(arm)), handover_line(arm)


# Positions from perception (#260): the brain knows the grid and the dispenser where the camera sees them.
def moved(T, along=0.0) -> np.ndarray:
    """The frame moved along its own x, m."""
    out = T.copy()
    out[:3, 3] += along * T[:3, 0]
    return out


def test_a_drop_or_a_pick_is_refused_until_the_camera_has_placed_the_grid_and_the_dispenser(arm):
    arm.to_handover()
    before, brain = len(arm.commands), arm.brain
    grid, dispenser = brain._grid, brain._dispenser
    try:
        brain._grid = None
        assert arm.call(arm.drop, drop_request(), timeout=3.0).message == "no_grid_pose"
        assert arm.call(arm.pick, drop_request(STATIONS[0]), timeout=3.0).message == "no_grid_pose"
        brain._grid, brain._dispenser = grid, None
        assert arm.call(arm.pick, drop_request(STATIONS[0]), timeout=3.0).message == "no_dispenser_pose"
    finally:
        brain._grid, brain._dispenser = grid, dispenser
    assert len(arm.commands) == before


def test_a_drop_whose_way_passes_too_near_the_measured_grid_is_refused_without_any_command(arm):
    """The grid 25 mm nearer the arm: every column still has a pose, and the shoulder would pass through
    its near post [MES-78]."""
    arm.to_handover()
    before = len(arm.commands)
    try:
        arm.place(grid=moved(GRID, -0.025))
        reply = arm.call(arm.drop, drop_request(COLUMNS[3] - 0.025 * GRID[:3, 0]), timeout=5.0)
        assert not reply.success
        assert (reply.success, reply.message) == (False, "unsafe_path:column")  # the first step too near
        line = next(text for text in reversed(arm.logs) if text.startswith("/brain/drop"))
        assert re.fullmatch(r"/brain/drop: unsafe_path:column, -?\d+\.\d mm from the grid", line), line
    finally:
        arm.place(grid=GRID)
    assert len(arm.commands) == before
    arm.token = True
    reply = arm.call(arm.drop, drop_request())  # the grid back where it was: the same drop is done
    arm.token = False
    assert reply.success, reply.message


def test_a_drop_on_a_grid_that_is_not_edge_on_is_refused_without_any_command(arm):
    arm.to_handover()
    before = len(arm.commands)
    try:
        arm.place(grid=frame(-40.0, COLUMNS[3], turned_deg=3.0))  # turned about the column aimed at
        reply = arm.call(arm.drop, drop_request(COLUMNS[3]), timeout=5.0)
        assert (reply.success, reply.message) == (False, "not_edge_on")
        arm.place(dispenser=frame(60.0, STATIONS[1], turned_deg=6.0))
        reply = arm.call(arm.pick, drop_request(STATIONS[1]), timeout=5.0)
        assert (reply.success, reply.message) == (False, "not_edge_on")
    finally:
        arm.place(grid=GRID, dispenser=DISPENSER)
    assert len(arm.commands) == before


def test_a_check_only_request_judges_the_move_from_anywhere_and_moves_nothing(arm):
    """What the game asks before it says ready: would this drop, this pick, be done?"""
    arm.to_handover()
    arm.q["shoulder_pan"] = 0.0  # away from HANDOVER: a check does not need the arm there
    arm.wait_for_fresh_joint_states()
    before, start = len(arm.commands), time.monotonic()
    for client, target in ((arm.drop, COLUMNS[0]), (arm.drop, COLUMNS[6]), (arm.pick, STATIONS[2])):
        reply = arm.call(client, drop_request(target, check_only=True), timeout=5.0)
        assert (reply.success, reply.message) == (True, ""), reply.message
    assert time.monotonic() - start < 6.0
    reply = arm.call(arm.drop, drop_request((0.7, 0.0, 0.2), check_only=True), timeout=5.0)
    assert (reply.success, reply.message) == (False, "unreachable")
    try:
        arm.place(grid=moved(GRID, -0.025))
        reply = arm.call(
            arm.drop, drop_request(COLUMNS[3] - 0.025 * GRID[:3, 0], check_only=True), timeout=5.0
        )
        assert reply.message.startswith("unsafe_path:"), reply.message
        arm.place(grid=frame(-40.0, COLUMNS[3], turned_deg=3.0))
        reply = arm.call(arm.drop, drop_request(COLUMNS[3], check_only=True), timeout=5.0)
        assert (reply.success, reply.message) == (False, "not_edge_on")
    finally:
        arm.place(grid=GRID)
    assert len(arm.commands) == before


def test_a_check_asked_during_a_move_is_refused_at_once_and_the_move_goes_on(arm):
    """A check holds the executor for its IK, half a second: during a move the 50 Hz commands would stop
    that long, and the joints would look stale at the next tick (test_connect4_drop_launch, #260)."""
    arm.to_handover()
    arm.q["shoulder_pan"] = -0.3  # away from HANDOVER: the way back is a move of a second
    arm.wait_for_fresh_joint_states()
    move = arm.request(arm.handover)
    arm.spin_for(0.3)
    assert not move.done()
    start = time.monotonic()
    for client, target in ((arm.drop, COLUMNS[3]), (arm.pick, STATIONS[0])):
        reply = arm.call(client, drop_request(target, check_only=True), timeout=3.0)
        assert (reply.success, reply.message) == (False, "busy")
    assert time.monotonic() - start < 0.2
    assert arm.reply(move).success
    np.testing.assert_allclose(arm.arm(), HANDOVER, atol=0.01)
    reply = arm.call(arm.drop, drop_request(check_only=True), timeout=5.0)  # asked again once it has ended
    assert (reply.success, reply.message) == (True, "")


def test_the_arm_waits_beside_the_dispenser_where_the_camera_sees_it(arm):
    """HANDOVER is where the layout puts the dispenser; elsewhere the arm waits at the same place beside it,
    and a pick and a drop start from there."""
    arm.to_handover()
    np.testing.assert_allclose(arm.arm(), HANDOVER, atol=0.01)
    turned = about_pan(DISPENSER, 10.0)
    stations = [(about_pan(frame(0.0, station), 10.0))[:3, 3] for station in STATIONS]
    try:
        arm.place(dispenser=turned)
        assert arm.call(arm.handover).success
        np.testing.assert_allclose(
            arm.arm(), np.array(HANDOVER) + np.array([-np.radians(10.0), 0, 0, 0, 0]), atol=0.01
        )
        arm.token = True
        reply = arm.call(arm.pick, drop_request(stations[0]))
        assert reply.success, reply.message
        reply = arm.call(arm.drop, drop_request())
        assert reply.success, reply.message
        np.testing.assert_allclose(
            arm.arm(), np.array(HANDOVER) + np.array([-np.radians(10.0), 0, 0, 0, 0]), atol=0.01
        )
        # a dispenser nobody can reach: the arm stays where it waits
        before = len(arm.commands)
        arm.place(dispenser=moved(DISPENSER, 0.3))
        reply = arm.call(arm.handover, timeout=5.0)
        assert (reply.success, reply.message) == (False, "unreachable")
        assert len(arm.commands) == before
    finally:
        arm.token = False
        arm.place(dispenser=DISPENSER)
    assert arm.call(arm.handover).success
    np.testing.assert_allclose(arm.arm(), HANDOVER, atol=0.01)


def test_a_handover_whose_line_crosses_the_measured_grid_is_refused(arm):
    """The arm left beyond the grid, low: the joint line back to HANDOVER goes through it [MES-69]."""
    arm.to_handover()
    arm.q.update(shoulder_pan=1.3, shoulder_lift=0.2, elbow_flex=0.4, wrist_flex=0.6)
    arm.wait_for_fresh_joint_states()
    before = len(arm.commands)
    reply = arm.call(arm.handover, timeout=5.0)
    assert (reply.success, reply.message) == (False, "unsafe_path:handover")
    assert len(arm.commands) == before
    arm.q.update(dict(zip(JOINTS[:5], HANDOVER, strict=True)))
    arm.to_handover()
