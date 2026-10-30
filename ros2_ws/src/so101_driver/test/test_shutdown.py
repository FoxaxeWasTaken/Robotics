"""Shutdown move (docs/project/03 §8): before disconnect() cuts the torque, the driver brings the arm slowly
to its rest pose by direct backend calls (the ROS context is already down on SIGINT)."""

import pytest
from lerobot.robots.utils import ensure_safe_goal_position
from so101_sim import SO101Sim

from so101_common.robot import JOINT_LIMITS
from so101_driver.driver_node import PARAMETERS, SHUTDOWN_POSE, SHUTDOWN_TIMEOUT
from so101_driver.limiter import max_relative_target
from so101_driver.shutdown import move_to

PARK = {"shoulder_pan": 1.9, "shoulder_lift": -1.7, "elbow_flex": 1.5, "wrist_flex": 0.7, "wrist_roll": 0.0}
RATE_HZ, VELOCITY, TOLERANCE = 50.0, 0.5, 0.05


@pytest.fixture
def sim():
    sim = SO101Sim()
    sim.connect()
    yield sim
    sim.disconnect()


def startup_pose(sim):
    return {key.removesuffix(".pos"): value for key, value in sim.get_observation().items()}


def move(sim, setpoint, goal, timeout):
    return move_to(sim, setpoint, goal, RATE_HZ, VELOCITY, timeout, TOLERANCE)


def test_the_arm_reaches_the_rest_pose_no_faster_than_the_shutdown_speed(sim):
    start = startup_pose(sim)
    distance = max(abs(PARK[name] - start[name]) for name in PARK)
    elapsed, error = move(sim, start, PARK, timeout=10.0)
    print(
        f"startup pose {start}: {distance:.2f} rad to go, reached in {elapsed:.2f} s, error {error:.3f} rad"
    )
    assert error <= TOLERANCE
    assert elapsed >= (distance - TOLERANCE) / VELOCITY
    reached = startup_pose(sim)
    assert reached["gripper"] == pytest.approx(start["gripper"], abs=1.0)  # not commanded: it keeps its grip


def test_a_pose_out_of_time_gives_up_at_the_timeout(sim):
    start = startup_pose(sim)
    elapsed, error = move(
        sim, start, {"shoulder_pan": start["shoulder_pan"] + 1.5}, timeout=0.5
    )  # 3 s needed
    assert error > 1.0
    assert 0.5 <= elapsed < 0.8


class RealArmChecks(SO101Sim):
    """SO101Sim with the check SO101Follower.send_action makes on the real arm: one max_relative_target per
    motor, so every action must name the 6 motors (ensure_safe_goal_position)."""

    def send_action(self, action):
        present = self.get_observation()
        ensure_safe_goal_position(
            {key.removesuffix(".pos"): (value, present[key]) for key, value in action.items()},
            max_relative_target(0.2, 25.0),
        )
        return super().send_action(action)


def test_the_shutdown_move_sends_every_motor_as_the_real_arm_requires():
    """#194: the move sent the 5 arm joints only: the real arm raised a ValueError at its first command."""
    sim = RealArmChecks()
    sim.connect()
    try:
        _, error = move(sim, startup_pose(sim), PARK, timeout=10.0)
    finally:
        sim.disconnect()
    assert error <= TOLERANCE


def test_the_shutdown_move_has_time_to_reach_park_from_any_pose():
    """#195: 4 s at 0.5 rad/s is 2 rad, and shoulder_pan can be 3.8 rad from its PARK value."""
    park = dict(zip(PARK, SHUTDOWN_POSE, strict=True))
    farthest = max(
        max(abs(low - park[name]), abs(high - park[name]))
        for name, (low, high) in JOINT_LIMITS.items()
        if name in park
    )
    assert farthest / PARAMETERS["shutdown_velocity"][0] <= SHUTDOWN_TIMEOUT
