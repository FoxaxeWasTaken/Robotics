"""Shutdown move (docs/project/03 §8): before disconnect() cuts the torque, the driver brings the arm slowly
to its rest pose by direct backend calls (the ROS context is already down on SIGINT)."""

import pytest
from so101_sim import SO101Sim

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
