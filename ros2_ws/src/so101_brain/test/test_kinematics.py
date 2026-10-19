"""Unit tests of the kinematics façade (docs/project/04 §1-§3, §8). No ROS, no sim: FK against MuJoCo is
checked in so101_bringup (sim-backed tests, C-05)."""

from pathlib import Path

import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory
from hypothesis import given, settings
from hypothesis import strategies as st

from so101_brain.kinematics import Kinematics
from so101_common.robot import ARM_JOINT_NAMES, JOINT_LIMITS

URDF = Path(get_package_share_directory("so101_description")) / "urdf" / "so101.urdf"
LOW = np.array([JOINT_LIMITS[n][0] for n in ARM_JOINT_NAMES])
HIGH = np.array([JOINT_LIMITS[n][1] for n in ARM_JOINT_NAMES])
configurations = st.tuples(*(st.floats(lo, hi) for lo, hi in zip(LOW, HIGH, strict=True))).map(np.array)


@pytest.fixture(scope="module")
def kin():
    return Kinematics(str(URDF))


def tcp(kin, q):
    return kin.fk(q)[:3, 3]


def test_tcp_at_zero_is_the_stretched_arm(kin):
    np.testing.assert_allclose(tcp(kin, np.zeros(5)), [0.391, 0.0, 0.226], atol=1e-3)  # 04 §1.1


def test_azimuth_is_measured_from_the_pan_axis(kin):
    assert kin.arm_azimuth([0.0388 + 0.1, 0.1, 0.05]) == pytest.approx(np.pi / 4)
    assert kin.arm_azimuth([0.0388 + 0.2, 0.0, 0.05]) == pytest.approx(0.0)
    assert kin.arm_azimuth([0.0388 + 0.1, -0.1, 0.05]) == pytest.approx(-np.pi / 4)


@settings(max_examples=200, deadline=None)
@given(configurations)
def test_ik_reaches_every_reachable_position(kin, q):
    p = tcp(kin, q)
    solution = kin.ik_pos(p)
    assert solution is not None
    assert np.linalg.norm(tcp(kin, solution) - p) <= 1e-3
    assert np.all(solution >= LOW - 1e-9)
    assert np.all(solution <= HIGH + 1e-9)


def test_a_seed_past_the_bounds_does_not_crash(kin):
    # Real arm at rest: elbow 1.7007 > 1.69; sim overshoot on wrist_roll. Unclipped: "x0 is infeasible".
    seed = np.array([0.0, 0.0, 1.7007, 0.0, 3.0])
    assert kin.ik_pos([0.22, 0.05, 0.10], q_seed=seed) is not None


def test_an_unreachable_position_gives_none(kin):
    assert kin.ik_pos([1.0, 0.0, 0.0]) is None
