"""Speed limiter (ADR-023): each control tick moves the setpoint towards the goal by at most v_max * dt."""

import pytest

from so101_driver.limiter import step_towards

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
