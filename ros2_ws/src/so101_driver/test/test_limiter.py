"""Speed limiter (ADR-023): each control tick moves the setpoint towards the goal by at most v_max * dt."""

import pytest
from lerobot.robots.utils import ensure_safe_goal_position

from so101_driver.limiter import max_relative_target, step_towards

DT, V_ARM, V_GRIPPER = 0.02, 1.5, 200.0  # one tick: 0.03 rad for an arm joint, 4 % for the gripper


def step(setpoint, goal):
    return step_towards(setpoint, goal, DT, V_ARM, V_GRIPPER)


def test_far_goal_moves_one_step_in_its_direction():
    out = step({"shoulder_pan": 0.0, "elbow_flex": 0.0}, {"shoulder_pan": 1.0, "elbow_flex": -1.0})
    assert out == pytest.approx({"shoulder_pan": 0.03, "elbow_flex": -0.03})


def test_close_goal_is_reached_without_overshoot():
    assert step({"wrist_flex": 0.49}, {"wrist_flex": 0.5}) == pytest.approx({"wrist_flex": 0.5})


def test_gripper_has_its_own_speed_in_percent():
    assert step({"gripper": 0.0}, {"gripper": 100.0}) == pytest.approx({"gripper": 4.0})


def test_max_relative_target_caps_each_of_the_6_motors_in_its_own_unit():
    # SO101Follower.send_action caps every motor it commands: a missing one raises ValueError.
    motors = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
    safe = ensure_safe_goal_position(dict.fromkeys(motors, (50.0, 0.0)), max_relative_target(0.2, 25.0))
    assert safe == pytest.approx({**dict.fromkeys(motors, 0.2), "gripper": 25.0})  # arm in rad, gripper in %
