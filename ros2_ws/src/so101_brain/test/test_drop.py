"""The drop of phase 2 (drop.py, phase2/P2-02 §2): on the 7 columns of the layout of P2-02 §7, the held
token's centre on the target, the jaws normal to the grid, the approach tilted 70°; then one move of
/brain/drop and of /brain/handover, step by step, on an arm that follows its commands. No ROS, no sim."""

import time
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
    handover = np.array([-1.0, -0.6, 0.6, 1.1, 1.57])
    for target in (COLUMNS[0], COLUMNS[6]):
        np.testing.assert_allclose(release_pose(kin, target, handover), release_pose(kin, target), atol=0.02)


@pytest.mark.parametrize(
    "target", [(0.7, 0.0, 0.2), (PAN_AXIS_X + 0.02, 0.0, 0.05)], ids=["far", "on-the-base"]
)
def test_an_unreachable_target_gives_none(kin, target):
    assert release_pose(kin, np.array(target)) is None


HANDOVER = np.array([-1.0, -0.6, 0.6, 1.1, 1.57])  # brain.yaml's connect4.handover_pose
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

    def __init__(self, q, effort=0.5, follows=True, stops_at_pct=0.0):
        self.q, self.effort, self.follows, self.stops_at = (
            np.array(q, dtype=float),
            effort,
            follows,
            stops_at_pct,
        )
        self.gripper, self.t, self.log = 100.0, 0.0, []

    def run(self, move, limit=60.0):
        """Steps the move to its result: (result, the seconds it took)."""
        while self.t < limit:
            closing, name = self.gripper <= self.stops_at, move.name
            out = move.step(self.t, self.q.copy(), self.effort if closing else 0.0, self.gripper)
            self.log.append((self.t, name, out))
            if out.command is not None and self.follows:
                self.q = np.array(out.command)
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
    assert arm.log[0][2].gripper == P.open_pct and arm.gripper == P.open_pct
