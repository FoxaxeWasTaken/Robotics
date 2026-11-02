"""Unit tests of the /joint_command merge (docs/project/03 §4): partial commands, limits, rejections."""

import math

import pytest
from hypothesis import given
from hypothesis import strategies as st

from so101_common.robot import ARM_JOINT_NAMES, JOINT_LIMITS, JOINT_NAMES, pct_to_rad
from so101_driver.command import apply_command, setpoint_positions

GOAL = {**dict.fromkeys(ARM_JOINT_NAMES, 0.0), "gripper": 50.0}  # arm in rad, gripper in %
NON_FINITE = (math.nan, math.inf, -math.inf)


def within_limits(goal):
    return all(
        (0.0 <= v <= 100.0) if n == "gripper" else (JOINT_LIMITS[n][0] <= v <= JOINT_LIMITS[n][1])
        for n, v in goal.items()
    )


def test_partial_command_moves_only_the_named_joint():
    goal, reason = apply_command(GOAL, ["elbow_flex"], [0.3])
    assert reason is None
    assert goal == {**GOAL, "elbow_flex": 0.3}
    assert GOAL["elbow_flex"] == 0.0  # the input goal is not modified


def test_empty_name_list_follows_joint_order():
    goal, reason = apply_command(GOAL, [], [0.1, 0.2, 0.3, 0.4, 0.5, 60.0])
    assert reason is None
    assert goal == dict(zip(JOINT_NAMES, [0.1, 0.2, 0.3, 0.4, 0.5, 60.0], strict=True))


@pytest.mark.parametrize(
    ("name", "value", "expected"),
    [
        ("shoulder_pan", 5.0, 1.91986),
        ("shoulder_pan", -5.0, -1.91986),
        ("wrist_roll", 3.0, 2.84121),
        ("wrist_roll", -3.0, -2.74385),
        ("gripper", 150.0, 100.0),
        ("gripper", -20.0, 0.0),
    ],
)
def test_out_of_range_values_are_clamped(name, value, expected):
    goal, reason = apply_command(GOAL, [name], [value])
    assert reason is None
    assert goal[name] == expected


@pytest.mark.parametrize("bad", NON_FINITE)
def test_non_finite_value_rejects_the_whole_message(bad):
    goal, reason = apply_command(GOAL, ["shoulder_pan", "elbow_flex"], [0.5, bad])
    assert goal == GOAL  # not even the valid pair is applied
    assert "non-finite" in reason


def test_the_rejection_says_why():
    """The driver logs the reason of each rejected command (03 §4)."""
    assert apply_command(GOAL, [], [math.nan, 0.0, 0.0, 0.0, 0.0, 0.0]) == (GOAL, "non-finite position")
    assert apply_command(GOAL, ["elbow_flex"], [0.1, 0.2]) == (GOAL, "1 names, 2 positions")


def test_an_unknown_name_does_not_hide_the_joints_after_it():
    goal, _ = apply_command(GOAL, ["camera_tilt", "elbow_flex"], [0.5, 0.3])
    assert goal == {**GOAL, "elbow_flex": 0.3}


def test_unknown_name_is_ignored():
    goal, reason = apply_command(GOAL, ["shoulder_pan", "nose"], [0.5, 1.0])
    assert reason is None
    assert goal == {**GOAL, "shoulder_pan": 0.5}


@pytest.mark.parametrize(("names", "positions"), [(["shoulder_pan"], [0.1, 0.2]), ([], [0.1])])
def test_size_mismatch_rejects_the_whole_message(names, positions):
    goal, reason = apply_command(GOAL, names, positions)
    assert goal == GOAL
    assert "names" in reason


commands = st.lists(
    st.tuples(st.sampled_from([*JOINT_NAMES, "unknown"]), st.floats()), min_size=1, max_size=8
)
finite_commands = st.lists(
    st.tuples(st.sampled_from(JOINT_NAMES), st.floats(allow_nan=False, allow_infinity=False)),
    min_size=1,
    max_size=8,
)


@given(commands)
def test_any_command_keeps_every_goal_within_the_limits(cmd):
    goal, _ = apply_command(GOAL, [n for n, _ in cmd], [v for _, v in cmd])
    assert within_limits(goal)


@given(finite_commands)
def test_clamping_twice_changes_nothing(cmd):
    names, values = [n for n, _ in cmd], [v for _, v in cmd]
    once, _ = apply_command(GOAL, names, values)
    twice, _ = apply_command(once, names, values)
    assert twice == once


@given(finite_commands, st.sampled_from(NON_FINITE), st.integers(min_value=0))
def test_a_message_with_a_non_finite_value_never_moves_a_joint(cmd, bad, where):
    names, values = [n for n, _ in cmd], [v for _, v in cmd]
    values[where % len(values)] = bad
    goal, reason = apply_command(GOAL, names, values)
    assert goal == GOAL
    assert reason


def test_the_setpoint_is_published_in_urdf_order_in_rad():
    """/joint_setpoint (#200): every joint, JOINT_NAMES order whatever the dict's, the gripper % -> rad."""
    setpoint = {"gripper": 100.0, **{name: 0.1 * (i + 1) for i, name in enumerate(reversed(ARM_JOINT_NAMES))}}
    positions = setpoint_positions(setpoint)
    assert len(positions) == len(JOINT_NAMES)
    assert positions[:-1] == [setpoint[name] for name in ARM_JOINT_NAMES]
    assert positions[-1] == pytest.approx(pct_to_rad(100.0))
    assert setpoint_positions({**setpoint, "gripper": 0.0})[-1] == pytest.approx(pct_to_rad(0.0))
