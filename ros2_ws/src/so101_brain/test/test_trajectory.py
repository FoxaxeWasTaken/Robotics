"""Unit tests of the minimum-jerk trajectory (docs/project/04 §6, ADR-010)."""

import numpy as np
from hypothesis import given
from hypothesis import strategies as st
from hypothesis.extra.numpy import arrays

from so101_brain.trajectory import min_jerk

Q0 = np.array([0.0, -0.5, 0.3, 1.0, 0.0])
Q1 = np.array([0.8, 0.2, -0.4, 0.5, 1.2])
joints = arrays(float, 5, elements=st.floats(-3.0, 3.0))


def peak_speed(traj, n=2001):
    """Largest finite-difference speed over the motion, any joint."""
    ts = np.linspace(0.0, traj.duration, n)
    qs = np.array([traj.sample(t) for t in ts])
    return float((np.abs(np.diff(qs, axis=0)) / np.diff(ts)[:, None]).max())


def test_endpoints_are_exact():
    traj = min_jerk(Q0, Q1, v_max=1.0, t_min=0.5)
    np.testing.assert_array_equal(traj.sample(0.0), Q0)
    np.testing.assert_array_equal(traj.sample(traj.duration), Q1)


def test_velocity_is_zero_at_both_ends():
    traj = min_jerk(Q0, Q1, v_max=1.0, t_min=0.5)
    h = 1e-4
    start = (traj.sample(h) - traj.sample(0.0)) / h
    end = (traj.sample(traj.duration) - traj.sample(traj.duration - h)) / h
    assert np.abs(start).max() < 1e-6
    assert np.abs(end).max() < 1e-6


def test_duration_follows_the_slowest_joint():
    traj = min_jerk(Q0, Q1, v_max=1.0, t_min=0.5)
    assert traj.duration == 1.2 * 1.875  # wrist_roll moves the most: 1.2 rad


def test_a_short_motion_lasts_t_min():
    traj = min_jerk(Q0, Q0 + 0.01, v_max=1.0, t_min=0.5)
    assert traj.duration == 0.5


def test_sample_clamps_outside_the_motion():
    traj = min_jerk(Q0, Q1, v_max=1.0, t_min=0.5)
    np.testing.assert_array_equal(traj.sample(-1.0), Q0)
    np.testing.assert_array_equal(traj.sample(traj.duration + 5.0), Q1)


def test_no_motion_and_no_minimum_duration():
    traj = min_jerk(Q0, Q0, v_max=1.0, t_min=0.0)
    assert traj.duration == 0.0
    np.testing.assert_array_equal(traj.sample(0.0), Q0)


@given(joints, joints, st.floats(0.1, 3.0))
def test_peak_velocity_never_exceeds_v_max(q0, q1, v_max):
    traj = min_jerk(q0, q1, v_max=v_max, t_min=0.01)
    assert peak_speed(traj) <= v_max + 1e-6
