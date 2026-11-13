"""The drop of phase 2 (drop.py, phase2/P2-02 §2): on the 7 columns of the layout of P2-02 §7, the held
token's centre on the target, the jaws normal to the grid, the approach tilted 70°; then one move of
/brain/drop and of /brain/handover, step by step, on an arm that follows its commands. No ROS, no sim."""

import time
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory

from so101_brain import drop
from so101_brain.drop import ROLL, TILT_DEG, TOKEN_IN_TCP, release_pose
from so101_brain.fsm import Params
from so101_brain.kinematics import PAN_AXIS_X, Kinematics

URDF = Path(get_package_share_directory("so101_description")) / "urdf" / "so101.urdf"
# The layout of P2-02 §7 (so101_connect4/config/connect4_grid.yaml): azimuth -40°, columns 0.14 m + 37 mm
# apart, slot entries 25 cm above a table 10 cm below base_link, a 30 mm depth guide; the token's bottom 8 mm
# above the guide (P2-02 §2.3). Across the grid, the token centre 4.85 mm off the pan axis on the side of the
# TCP line (MES-64: the TCP 8 mm off the axis, the token 3.15 mm from it), as e7_cycle.py dropped 42 of 42.
AZIMUTH = np.radians(-40.0)
ALONG = np.array([np.cos(AZIMUTH), np.sin(AZIMUTH), 0.0])
NORMAL = np.array([-np.sin(AZIMUTH), np.cos(AZIMUTH), 0.0])  # the grid's normal
TOKEN_Z = -0.10 + 0.25 + 0.030 + 0.008 + 0.017  # table, entries, guide, clearance, token radius
COLUMNS = [
    np.array([PAN_AXIS_X, 0.0, TOKEN_Z]) + (0.14 + 0.037 * c) * ALONG - 0.00485 * NORMAL for c in range(7)
]


class CountingKinematics(Kinematics):
    """Counts the solutions _solve rejects as folded onto the arm."""

    rejected = 0

    def self_collides(self, q):
        folded = super().self_collides(q)
        self.rejected += folded
        return folded


@pytest.fixture(scope="module")
def kin():
    return CountingKinematics(str(URDF))


def token_centre(kin, q):
    pose = kin.fk(q)
    return pose[:3, 3] + pose[:3, :3] @ TOKEN_IN_TCP


@pytest.mark.parametrize("column", range(7))
def test_each_column_gets_the_token_on_target_jaws_normal_to_the_grid_tilted_70(kin, column):
    kin.rejected = 0
    start = time.perf_counter()
    q = release_pose(kin, COLUMNS[column])
    elapsed = time.perf_counter() - start
    print(f"column {column}: {elapsed * 1000:.0f} ms")  # 10 to 36 ms in MES-64
    assert q is not None
    pose = kin.fk(q)
    assert np.linalg.norm(token_centre(kin, q) - COLUMNS[column]) < 1e-3
    jaw_error = np.degrees(np.arccos(min(1.0, abs(pose[:3, 0] @ NORMAL))))
    assert jaw_error < 0.2
    tilt = np.degrees(np.arccos(np.clip(-pose[2, 2], -1.0, 1.0)))  # bare ik_pos flips the tool: 114-149°
    assert tilt == pytest.approx(TILT_DEG, abs=2.0)
    assert q[4] == pytest.approx(ROLL)
    assert kin.rejected == 0


def test_the_seed_does_not_change_the_pose(kin):
    """From HANDOVER's candidate (P2-02 §7) as from the default bent seed: the same joints."""
    handover = np.array([-0.8, -0.6, 0.6, 1.1, 1.57])
    for target in (COLUMNS[0], COLUMNS[6]):
        np.testing.assert_allclose(release_pose(kin, target, handover), release_pose(kin, target), atol=0.02)


@pytest.mark.parametrize(
    "target", [(0.7, 0.0, 0.2), (PAN_AXIS_X + 0.02, 0.0, 0.05)], ids=["far", "on-the-base"]
)
def test_an_unreachable_target_gives_none(kin, target):
    assert release_pose(kin, np.array(target)) is None


HANDOVER = np.array([-0.8, -0.6, 0.6, 1.1, 1.57])  # brain.yaml's connect4.handover_pose
P = Params()
DT = 0.02


def names(steps):
    return [step.name for step in steps]


@pytest.mark.parametrize("column", range(7))
def test_a_move_rises_first_then_turns_to_the_column_descends_and_lifts_straight_up(kin, column):
    target = COLUMNS[column]
    steps = drop.drop_steps(kin, target, HANDOVER, P)
    by_name = {step.name: step for step in steps}
    lifts = [step for step in steps if step.name.startswith("lift")]
    assert names(steps) == [
        "close",
        "rise",
        "column",
        "descent",
        "release",
        *names(lifts),
        "clear",
        "handover",
    ]
    pre, release = by_name["column"].q, by_name["descent"].q
    # height first: the pan stays HANDOVER's while the other joints take the pre-drop's
    np.testing.assert_allclose(by_name["rise"].q, [HANDOVER[0], *pre[1:]])
    np.testing.assert_allclose(token_centre(kin, pre) - target, [0, 0, drop.PRE_DROP], atol=1e-3)
    np.testing.assert_allclose(token_centre(kin, release), target, atol=1e-3)
    assert by_name["descent"].v == drop.DESCENT_V
    # the lift: straight up every 5 mm to the pre-drop, never one joint move (it swept the guide, MES-63)
    heights = [token_centre(kin, step.q) - target for step in lifts]
    assert len(lifts) == 5
    for k, offset in enumerate(heights, start=1):
        np.testing.assert_allclose(offset, [0, 0, k * drop.LIFT_STEP], atol=1e-3)
    np.testing.assert_allclose(lifts[-1].q, pre)
    np.testing.assert_allclose(by_name["clear"].q, by_name["rise"].q)
    np.testing.assert_allclose(by_name["handover"].q, HANDOVER)
    # the jaws: closed first, opened at the release only
    assert [(step.name, step.gripper) for step in steps if step.gripper is not None] == [
        ("close", P.close_pct),
        ("release", P.open_pct),
    ]


def test_a_lift_waypoint_without_a_pose_is_skipped(kin, monkeypatch):
    solve, calls = drop.release_pose, []

    def no_pose_at_10_mm(kin, target, q_seed=None):
        calls.append(target)
        return None if len(calls) == 4 else solve(kin, target, q_seed)  # pre-drop, release, 5 mm, 10 mm

    monkeypatch.setattr(drop, "release_pose", no_pose_at_10_mm)
    steps = drop.drop_steps(kin, COLUMNS[3], HANDOVER, P)
    lifts = [name for name in names(steps) if name.startswith("lift")]
    assert lifts == ["lift_1", "lift_3", "lift_4", "lift_5"]


def test_a_move_to_an_unreachable_target_has_no_steps(kin):
    assert drop.drop_steps(kin, np.array([0.7, 0.0, 0.2]), HANDOVER, P) is None


class Arm:
    """An arm that follows every command at once, the jaws too; its closing effort is `effort` while the
    jaws are commanded closed."""

    def __init__(self, q, effort=0.5, follows=True, stops_at_pct=0.0, roll_off=0.0):
        self.q, self.effort, self.follows, self.stops_at = (
            np.array(q, dtype=float),
            effort,
            follows,
            stops_at_pct,
        )
        self.roll_off = roll_off  # rad: the wrist roll stops this far from its command
        self.gripper, self.t, self.log = 100.0, 0.0, []

    def run(self, move, limit=60.0):
        """Steps the move to its result: (result, the seconds it took)."""
        while self.t < limit:
            closing, name = self.gripper <= self.stops_at, move.name
            out = move.step(self.t, self.q.copy(), self.effort if closing else 0.0, self.gripper)
            self.log.append((self.t, name, out))
            if out.command is not None and self.follows:
                self.q = np.array(out.command)
                self.q[4] += self.roll_off
            if out.gripper is not None:
                self.gripper = max(out.gripper, self.stops_at)
            if out.result is not None:
                return out.result, self.t
            self.t += DT
        raise AssertionError(f"still in {move.name} after {limit} s")

    def commands(self, name):
        return [out.command for _, step, out in self.log if step == name and out.command is not None]


def test_a_drop_runs_its_steps_in_order_and_ends_at_handover(kin):
    steps = drop.drop_steps(kin, COLUMNS[3], HANDOVER, P)
    arm = Arm(HANDOVER)
    result, seconds = arm.run(drop.Move(steps, P))
    assert result == ""
    np.testing.assert_allclose(arm.q, HANDOVER)
    assert arm.gripper == P.open_pct
    visited = list(dict.fromkeys(step for _, step, _ in arm.log))
    assert visited == names(steps)
    print(f"one move on a perfect arm: {seconds:.1f} s")
    assert seconds < 15.0


def test_a_move_counts_the_seconds_of_each_step_the_lift_as_one(kin):
    """The move's log line needs no model of the motion to say where its time went (#247)."""
    arm, move = Arm(HANDOVER), drop.Move(drop.drop_steps(kin, COLUMNS[3], HANDOVER, P), P)
    _, seconds = arm.run(move)
    steps = ["close", "rise", "column", "descent", "release", "lift", "clear", "handover"]
    assert list(move.seconds) == steps
    assert sum(move.seconds.values()) == pytest.approx(seconds, abs=DT)
    assert move.seconds["close"] == pytest.approx(P.grasp_wait_s, abs=DT)
    assert move.seconds["lift"] == pytest.approx(5 * P.min_motion_s, abs=5 * DT)


def test_a_move_cut_short_counts_the_step_it_ended_in(kin):
    arm, move = Arm(HANDOVER, follows=False), drop.Move(drop.drop_steps(kin, COLUMNS[3], HANDOVER, P), P)
    _, seconds = arm.run(move)
    assert list(move.seconds) == ["close", "rise"]
    assert sum(move.seconds.values()) == pytest.approx(seconds, abs=DT)


def test_the_legs_back_go_faster_once_the_jaws_are_empty_the_ones_with_the_token_do_not(kin):
    steps = drop.drop_steps(kin, COLUMNS[6], HANDOVER, P)
    by_name = {step.name: step for step in steps}
    assert by_name["rise"].v is None  # the token held: the motion's v_max
    assert by_name["column"].v is None
    assert all(step.v is None for step in steps if step.name.startswith("lift"))  # next to the guide
    arm = Arm(HANDOVER)
    arm.run(drop.Move(steps, P))
    for name in ("clear", "handover"):
        speed = np.max(np.abs(np.diff(np.array(arm.commands(name)), axis=0))) / DT
        assert P.v_max < speed <= drop.EMPTY_SPEEDUP * P.v_max * 1.01, name


def test_a_slower_arm_slows_every_leg_of_the_move(kin):
    """The real arm's overlay lowers motion.v_max_rad_s: the legs back and the descent follow it."""
    slow = replace(P, v_max=0.2)
    arm = Arm(HANDOVER)
    arm.run(drop.Move(drop.drop_steps(kin, COLUMNS[6], HANDOVER, slow), slow), limit=120.0)
    for name, limit in (
        ("clear", drop.EMPTY_SPEEDUP * 0.2),
        ("handover", drop.EMPTY_SPEEDUP * 0.2),
        ("descent", 0.2),
    ):
        speed = np.max(np.abs(np.diff(np.array(arm.commands(name)), axis=0))) / DT
        assert speed <= limit * 1.01, name


def test_the_arm_waits_for_the_closing_time_before_the_grip_is_judged(kin):
    arm = Arm(HANDOVER)
    arm.run(drop.Move(drop.drop_steps(kin, COLUMNS[0], HANDOVER, P), P))
    first_motion = next(t for t, _, out in arm.log if out.command is not None)
    assert first_motion == pytest.approx(P.grasp_wait_s, abs=DT)


def test_empty_jaws_are_no_token_the_arm_never_commanded_the_jaws_open_again_at_the_result(kin):
    arm = Arm(HANDOVER, effort=0.0)
    result, seconds = arm.run(drop.Move(drop.drop_steps(kin, COLUMNS[3], HANDOVER, P), P))
    assert result == "no_token"
    # the result waits for the jaws: a token given right after it needs them open (the sim refuses it)
    assert seconds == pytest.approx(P.grasp_timeout_s + P.release_wait_s, abs=DT)
    opened = next(t for t, _, out in arm.log if out.gripper == P.open_pct)
    assert opened == pytest.approx(P.grasp_timeout_s, abs=DT)
    assert [out.command for _, _, out in arm.log if out.command is not None] == []
    assert arm.gripper == P.open_pct


def test_jaws_stopped_above_their_goal_hold_a_token_without_any_effort(kin):
    """The real arm publishes no effort (#69): the phase-1 rule, the jaws' gap."""
    arm = Arm(HANDOVER, effort=0.0, stops_at_pct=P.close_pct + P.hold_gap_pct + 1.0)
    result, _ = arm.run(drop.Move(drop.drop_steps(kin, COLUMNS[3], HANDOVER, P), P))
    assert result == ""


def test_an_arm_that_does_not_follow_times_out_in_its_step(kin):
    steps = drop.drop_steps(kin, COLUMNS[3], HANDOVER, P)
    arm = Arm(HANDOVER, follows=False)
    result, seconds = arm.run(drop.Move(steps, P))
    assert result == "timeout:rise"
    rise = min_jerk_duration(HANDOVER, steps[1].q, P.v_max)
    assert seconds == pytest.approx(P.grasp_wait_s + rise + P.motion_margin_s, abs=3 * DT)


def min_jerk_duration(start, goal, v_max):
    return max(P.min_motion_s, float(np.max(np.abs(goal - start))) * 1.875 / v_max)


def test_the_descent_never_goes_faster_than_its_speed(kin):
    arm = Arm(HANDOVER)
    arm.run(drop.Move(drop.drop_steps(kin, COLUMNS[6], HANDOVER, P), P))
    descent = np.array(arm.commands("descent"))
    assert len(descent) > 10
    assert np.max(np.abs(np.diff(descent, axis=0))) / DT <= drop.DESCENT_V * 1.01


def test_each_leg_starts_from_the_measured_joints(kin):
    """Nothing jumps if the arm lagged (06 §5)."""
    steps = drop.drop_steps(kin, COLUMNS[3], HANDOVER, P)
    lagging = HANDOVER + np.array([0.004, -0.004, 0.004, 0.0, 0.0])  # within the reach tolerance
    move = drop.Move(steps, P)
    out, t = None, 0.0
    while out is None or out.command is None:
        out = move.step(t, lagging, 0.5, 0.0)
        t += DT
    np.testing.assert_allclose(out.command, lagging)


def test_a_handover_opens_the_jaws_and_reaches_the_pose():
    arm = Arm([0.0, 0.0, 0.0, 0.0, 0.0])
    arm.gripper = 0.0
    result, _ = arm.run(drop.Move(drop.handover_steps(HANDOVER, P), P))
    assert result == ""
    np.testing.assert_allclose(arm.q, HANDOVER)
    assert arm.log[0][2].gripper == P.open_pct
    assert arm.gripper == P.open_pct


# Found by mutation testing (mutmut on drop.py, 08 §1) [MES-74]


def test_a_target_whose_pre_drop_alone_is_out_of_reach_has_no_steps(kin):
    """Every pose is solved before anything moves: the release in reach is not enough."""
    radius = next(
        r
        for r in np.arange(0.30, 0.50, 0.002)
        if release_pose(kin, [r, 0.0, 0.25]) is not None and release_pose(kin, [r, 0.0, 0.25 + 0.025]) is None
    )
    assert drop.PRE_DROP == 0.025
    assert drop.drop_steps(kin, np.array([radius, 0.0, 0.25]), HANDOVER, P) is None


def test_the_steps_are_named_as_the_logs_and_the_timeouts_say_them(kin):
    assert names(drop.handover_steps(HANDOVER, P)) == ["handover"]
    arm, move = Arm(HANDOVER, effort=0.0), drop.Move(drop.drop_steps(kin, COLUMNS[3], HANDOVER, P), P)
    arm.run(move)
    assert list(move.seconds) == ["close", "reopen"]  # empty jaws: they open again


def test_a_step_that_ends_at_once_still_sends_its_gripper_command():
    """A gripper command of a step with no wait is not lost when the next step starts on the same tick, nor
    at the move's last tick."""
    move = drop.Move([drop.Step("open", gripper=60.0), drop.Step("nothing")], P)
    out = move.step(0.0, HANDOVER, 0.0, 100.0)
    assert (out.gripper, out.result) == (60.0, "")


def test_the_last_tick_of_a_move_commands_its_goal():
    arm = Arm([0.0, 0.0, 0.0, 0.0, 0.0])
    arm.run(drop.Move(drop.handover_steps(HANDOVER, P), P))
    _, _, last = arm.log[-1]
    assert last.result == ""
    np.testing.assert_allclose(last.command, HANDOVER)


def test_a_timeout_still_commands_the_arm_on_its_last_tick(kin):
    arm = Arm(HANDOVER, follows=False)
    arm.run(drop.Move(drop.drop_steps(kin, COLUMNS[3], HANDOVER, P), P))
    _, _, last = arm.log[-1]
    assert last.result == "timeout:rise"
    assert last.command is not None


def test_the_wrist_roll_has_its_own_tolerance(kin):
    """The real arm's roll stops further from its goal than the other joints (brain_real.yaml)."""
    loose = replace(P, reach_tol_roll=0.1)
    arm = Arm(HANDOVER, roll_off=0.05)
    result, _ = arm.run(drop.Move(drop.drop_steps(kin, COLUMNS[3], HANDOVER, loose), loose))
    assert result == ""
    arm = Arm(HANDOVER, roll_off=0.05)
    result, _ = arm.run(drop.Move(drop.drop_steps(kin, COLUMNS[3], HANDOVER, P), P))
    assert result == "timeout:rise"


def test_the_jaws_gap_is_counted_from_their_closing_command(kin):
    """Jaws commanded to 20 % that stop at 22 % hold nothing: the gap is 2 %, under hold_gap_pct."""
    tight = replace(P, close_pct=20.0)
    assert tight.hold_gap_pct > 2.0
    arm = Arm(HANDOVER, effort=0.0, stops_at_pct=22.0)
    result, _ = arm.run(drop.Move(drop.drop_steps(kin, COLUMNS[3], HANDOVER, tight), tight))
    assert result == "no_token"


# The pick at the token dispenser (#259, ADR-041): the three stations of connect4_grid.yaml, in the arm's
# vertical plane at azimuth 60°: the lowest 0.27 m from the pan axis and 2 cm above base_link, each next one
# 40 mm further out and 62.8 mm higher; the tokens' plane 3.85 mm off the pan axis.
OUT = np.array([np.cos(np.radians(60.0)), np.sin(np.radians(60.0)), 0.0])
ASIDE = np.array([-OUT[1], OUT[0], 0.0])
STATIONS = [
    np.array([PAN_AXIS_X, 0.0, 0.02 + 0.0628 * k]) + (0.27 + 0.04 * k) * OUT - 0.00385 * ASIDE
    for k in range(3)
]


@pytest.mark.parametrize("station", range(3))
def test_a_pick_goes_down_beside_the_token_seats_the_fixed_jaw_on_it_closes_and_lifts_up(kin, station):
    target = STATIONS[station]
    steps = drop.pick_steps(kin, target, HANDOVER, P)
    downs = [step for step in steps if step.name.startswith("down")]
    lifts = [step for step in steps if step.name.startswith("lift")]
    assert names(steps) == ["above", *names(downs), "seat", "close", *names(lifts), "handover"]
    # as the logs and the timeouts say them: down_1 leaves the pose above, lift_1 the station
    assert names(downs) == ["down_1", "down_2", "down_3", "down_4"]
    assert names(lifts) == ["lift_1", "lift_2", "lift_3", "lift_4"]
    by_name = {step.name: step for step in steps}
    # The jaws' axis, from the token to the fixed jaw: the TCP is on the fixed jaw's side of a held token
    assert TOKEN_IN_TCP[0] < 0
    jaws = kin.fk(by_name["seat"].q)[:3, 0]
    assert abs(jaws[2]) < 0.01

    def off(step, height, aside):
        """m between the pose's token centre and the point `height` above the station's token, `aside`
        along the jaws' axis."""
        return np.linalg.norm(token_centre(kin, step.q) - target - [0, 0, height] - aside * jaws)

    # The way down: straight, every 10 mm, the fixed jaw 2.5 mm further from the token than where it holds
    # one: the token has 1.5 mm of play in its slot, the camera places the station within 1.5 mm [MES-77]
    assert (drop.PICK_ASIDE, drop.PICK_SEAT) == (0.0025, 0.0008)
    assert off(by_name["above"], drop.PICK_ABOVE, drop.PICK_ASIDE) < 2e-4
    count = round(drop.PICK_ABOVE / drop.PICK_STEP)
    assert count == 4
    assert len(downs) == count
    assert len(lifts) == count
    for k, step in enumerate(downs, start=1):
        assert off(step, drop.PICK_ABOVE - k * drop.PICK_STEP, drop.PICK_ASIDE) < 2e-4, step.name
    # Seated: sideways at the station's level, the fixed jaw against the face of a token in the slot's middle
    assert off(by_name["seat"], 0.0, -drop.PICK_SEAT) < 2e-4
    # The way up: straight above where the jaws closed, every 10 mm
    for k, step in enumerate(lifts, start=1):
        assert off(step, k * drop.PICK_STEP, -drop.PICK_SEAT) < 2e-4, step.name
    np.testing.assert_allclose(by_name["handover"].q, HANDOVER)
    # the jaws: part open on the way, closed on the token, never opened again: it leaves with the arm
    assert [(step.name, step.gripper) for step in steps if step.gripper is not None] == [
        ("above", drop.PICK_OPEN_PCT),
        ("close", P.close_pct),
    ]
    assert by_name["close"].held_by == P.grasp_timeout_s
    assert by_name["close"].wait == P.grasp_wait_s
    # the token is carried at the motion's speed, never faster
    assert {step.v for step in steps} == {None}


def test_a_pick_runs_its_steps_in_order_and_ends_at_handover_the_jaws_closed_on_the_token(kin):
    steps = drop.pick_steps(kin, STATIONS[0], HANDOVER, P)
    arm, move = Arm(HANDOVER), drop.Move(steps, P)
    result, seconds = arm.run(move)
    assert result == ""
    np.testing.assert_allclose(arm.q, HANDOVER)
    assert arm.gripper == P.close_pct
    assert list(dict.fromkeys(step for _, step, _ in arm.log)) == names(steps)
    assert list(move.seconds) == ["above", "down", "seat", "close", "lift", "handover"]
    print(f"one pick on a perfect arm: {seconds:.1f} s")
    assert seconds < 13.0  # 8.2 s at the highest station, 12.3 s at the lowest: the wrist turns 1.8 rad


def test_a_pick_that_closes_on_nothing_opens_the_jaws_and_comes_back_before_it_says_no_token(kin):
    """The arm is down in the station: it does not stay there, and the next pick starts from HANDOVER."""
    arm, move = Arm(HANDOVER, effort=0.0), drop.Move(drop.pick_steps(kin, STATIONS[1], HANDOVER, P), P)
    result, _ = arm.run(move)
    assert result == "no_token"
    np.testing.assert_allclose(arm.q, HANDOVER)
    assert arm.gripper == P.open_pct
    assert list(move.seconds) == ["above", "down", "seat", "close", "reopen", "lift", "handover"]
    opened = next(t for t, _, out in arm.log if out.gripper == P.open_pct)
    lifted = next(t for t, step, _ in arm.log if step == "lift_1")
    # the jaws are open before the arm lifts; the log names a step one tick after it starts
    assert lifted - opened == pytest.approx(P.release_wait_s + DT, abs=DT)


def test_a_pick_out_of_reach_has_no_steps(kin):
    """The station in reach is not enough: the pose above it is solved before anything moves, too."""
    assert drop.pick_steps(kin, np.array([0.7, 0.0, 0.2]), HANDOVER, P) is None
    radius = next(
        r
        for r in np.arange(0.30, 0.50, 0.002)
        if release_pose(kin, [r, 0.0, 0.25]) is not None
        and release_pose(kin, [r, 0.0, 0.25 + drop.PICK_ABOVE]) is None
    )
    assert drop.pick_steps(kin, np.array([radius, 0.0, 0.25]), HANDOVER, P) is None


def test_a_waypoint_of_a_pick_without_a_pose_refuses_the_pick(kin, monkeypatch):
    """A lift may skip a waypoint, the pre-drop is its last; the way down into a station may not: a jaw
    passes 3 mm from the dispenser [MES-75]."""
    solve = drop.release_pose

    def no_pose_at_20_mm(kin, target, q_seed=None):
        if abs(target[2] - STATIONS[0][2] - 0.020) < 1e-6:
            return None
        return solve(kin, target, q_seed)

    monkeypatch.setattr(drop, "release_pose", no_pose_at_20_mm)
    assert drop.pick_steps(kin, STATIONS[0], HANDOVER, P) is None


# A handover from where a stop left the arm (#259): down in a station, one joint motion to HANDOVER swept the
# stop of the dispenser by 9 mm.


def test_a_handover_from_elsewhere_rises_straight_up_before_it_turns_to_handover(kin):
    start = release_pose(kin, STATIONS[2])
    steps = drop.handover_steps(HANDOVER, P, kin, start)
    assert names(steps) == ["rise_1", "rise_2", "rise_3", "rise_4", "handover"]
    assert drop.HANDOVER_RISE == 0.040
    pose = kin.fk(start)
    for k, step in enumerate(steps[:-1], start=1):
        risen = kin.fk(step.q)
        np.testing.assert_allclose(risen[:3, 3] - pose[:3, 3], [0, 0, k * 0.010], atol=1e-3)
        np.testing.assert_allclose(risen[:3, :3], pose[:3, :3], atol=0.02)  # the tool keeps its orientation
    np.testing.assert_allclose(steps[-1].q, HANDOVER)
    # the jaws open as the arm starts to rise, once
    assert [(step.name, step.gripper) for step in steps if step.gripper is not None] == [
        ("rise_1", P.open_pct)
    ]
    arm = Arm(start)
    arm.gripper = 0.0
    result, _ = arm.run(move := drop.Move(steps, P))
    assert result == ""
    assert list(move.seconds) == ["rise", "handover"]
    np.testing.assert_allclose(arm.q, HANDOVER)


def test_a_handover_at_handover_or_with_no_pose_above_is_one_motion(kin, monkeypatch):
    nearby = HANDOVER + 0.04  # every joint within the 0.05 rad a drop accepts as HANDOVER
    assert names(drop.handover_steps(HANDOVER, P, kin, nearby)) == ["handover"]
    away = HANDOVER + np.array([0.0, 0.0, 0.0, 0.0, 0.06])
    assert names(drop.handover_steps(HANDOVER, P, kin, away))[0] == "rise_1"
    assert names(drop.handover_steps(HANDOVER, P)) == ["handover"]  # no joints known: as before
    monkeypatch.setattr(kin, "ik_pose", lambda *args, **kwargs: None)
    steps = drop.handover_steps(HANDOVER, P, kin, release_pose(kin, STATIONS[2]))
    assert [(step.name, step.gripper) for step in steps] == [("handover", P.open_pct)]


def test_a_rise_that_stops_having_a_pose_keeps_the_waypoints_it_has(kin, monkeypatch):
    solve, calls = kin.ik_pose, []

    def two_poses(*args, **kwargs):
        calls.append(1)
        return solve(*args, **kwargs) if len(calls) <= 2 else None

    monkeypatch.setattr(kin, "ik_pose", two_poses)
    steps = drop.handover_steps(HANDOVER, P, kin, release_pose(kin, STATIONS[2]))
    assert names(steps) == ["rise_1", "rise_2", "handover"]


# Where the grid and the dispenser stand (#260): the brain is told by the camera, as frames in base_link. A
# slot stands in the x-z plane of its frame, x outwards: the grid's along its columns, the dispenser's along
# the stations.
def frame(azimuth_deg, origin, turned_deg=0.0) -> np.ndarray:
    """A frame at `origin`, its x at this azimuth, then turned about the vertical through its origin."""
    a = np.radians(azimuth_deg + turned_deg)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 3] = [np.cos(a), np.sin(a), 0.0], [-np.sin(a), np.cos(a), 0.0], origin
    return T


def about_pan(T, degrees) -> np.ndarray:
    """The frame carried around the pan axis."""
    a, turn = np.radians(degrees), np.eye(4)
    turn[:2, :2] = [[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]]
    turn[:3, 3] = [PAN_AXIS_X - PAN_AXIS_X * np.cos(a), -PAN_AXIS_X * np.sin(a), 0.0]
    return turn @ T


DISPENSER = frame(60.0, STATIONS[0])


@pytest.mark.parametrize("column", [0, 3, 6])
def test_a_drop_meets_the_grid_edge_on_from_6_degrees_one_way_to_1_the_other(kin, column):
    """A token dropped 1.5° past the plane on the fixed jaw's side fell beside its column; 8° the other way
    it fell in [MES-79]."""
    steps = drop.drop_steps(kin, COLUMNS[column], HANDOVER, P)
    release = next(step.q for step in steps if step.name == "descent")
    assert abs(np.degrees(drop.off_plane(frame(-40.0, COLUMNS[column]), release))) < 1.0  # as the layout
    assert drop.off_plane(frame(-40.0, COLUMNS[column], turned_deg=3.0), release) == pytest.approx(
        drop.off_plane(frame(-40.0, COLUMNS[column]), release) + np.radians(3.0)
    )
    for turned, fine in ((-7.0, False), (-5.0, True), (0.0, True), (1.0, True), (2.0, False), (179.0, False)):
        assert drop.edge_on(frame(-40.0, COLUMNS[column], turned), steps) is fine, turned


def test_the_angle_off_the_arms_plane_is_wrapped_to_half_a_turn_either_way():
    """A slot along azimuth 179 deg and the arm at azimuth -179: 2 deg apart, not 358."""
    origin = [0.0, 0.0, 0.0]
    assert np.degrees(drop.off_plane(frame(179.0, origin), [np.radians(179.0)])) == pytest.approx(-2.0)
    assert np.degrees(drop.off_plane(frame(-179.0, origin), [np.radians(-179.0)])) == pytest.approx(2.0)


@pytest.mark.parametrize("station", range(3))
def test_a_pick_meets_the_dispenser_edge_on_within_4_degrees(kin, station):
    """The jaws touched the stop from 6° on, either way [MES-79]."""
    steps = drop.pick_steps(kin, STATIONS[station], HANDOVER, P)
    for turned, fine in ((-5.0, False), (-3.0, True), (0.0, True), (3.0, True), (5.0, False)):
        assert drop.edge_on(frame(60.0, STATIONS[station], turned), steps) is fine, turned


def test_a_grid_carried_around_the_pan_axis_is_as_edge_on_as_before(kin):
    """Edge-on is about the arm's plane, not about base_link: the arm turns with the grid."""
    for degrees in (-30.0, 20.0):
        T = about_pan(frame(-40.0, COLUMNS[3]), degrees)
        steps = drop.drop_steps(kin, T[:3, 3], HANDOVER, P)
        assert drop.edge_on(T, steps), degrees


def test_the_arm_waits_where_the_dispenser_is_as_handover_does_where_the_layout_puts_it(kin):
    assert np.allclose(drop.wait_pose(kin, DISPENSER, HANDOVER), HANDOVER, atol=0.01)


def test_the_wait_pose_follows_the_dispenser(kin):
    turned = drop.wait_pose(kin, about_pan(DISPENSER, 10.0), HANDOVER)
    assert np.allclose(
        turned, HANDOVER + np.array([-np.radians(10.0), 0, 0, 0, 0]), atol=0.01
    )  # pan = -azimuth
    further = DISPENSER.copy()
    further[:3, 3] += 0.02 * OUT
    out, there = drop.wait_pose(kin, further, HANDOVER), kin.fk(HANDOVER)
    moved = kin.fk(out)[:3, 3] - there[:3, 3]
    assert np.allclose(moved, 0.02 * OUT, atol=0.001)  # the tool 2 cm further out, as the dispenser
    assert kin.fk(out)[:3, 2] @ there[:3, 2] > np.cos(np.radians(3.0))  # pointing as at HANDOVER
    assert out[4] == pytest.approx(HANDOVER[4])  # the roll kept


def test_a_dispenser_out_of_reach_has_no_wait_pose(kin):
    far = DISPENSER.copy()
    far[:3, 3] += 0.3 * OUT
    assert drop.wait_pose(kin, far, HANDOVER) is None
