"""Gripper % <-> rad conversion: the driver publishes rad, the brain commands %; both use these functions."""

import pytest
from hypothesis import given
from hypothesis import strategies as st

from so101_common.robot import pct_to_rad, rad_to_pct

CLOSED, OPEN = -0.174533, 1.74533  # URDF gripper limits, copied by hand
MID = 0.7853985  # (CLOSED + OPEN) / 2


@pytest.mark.parametrize(("pct", "rad"), [(0, CLOSED), (100, OPEN), (50, MID), (-20, CLOSED), (150, OPEN)])
def test_pct_to_rad(pct, rad):
    assert pct_to_rad(pct) == pytest.approx(rad, abs=1e-12)


@pytest.mark.parametrize(("rad", "pct"), [(CLOSED, 0), (OPEN, 100), (MID, 50), (-1.0, 0), (3.0, 100)])
def test_rad_to_pct(rad, pct):
    assert rad_to_pct(rad) == pytest.approx(pct, abs=1e-9)


@pytest.mark.parametrize("pct", [0, 0.1, 12.5, 33.3, 50, 99.9, 100])
def test_round_trip(pct):
    assert abs(rad_to_pct(pct_to_rad(pct)) - pct) < 1e-9


# Property tests (Hypothesis): the same checks on hundreds of generated inputs, edge cases first.
@given(st.floats(min_value=0, max_value=100))
def test_round_trip_for_any_opening(pct):
    assert rad_to_pct(pct_to_rad(pct)) == pytest.approx(pct, abs=1e-9)


@given(st.floats(allow_nan=False))  # NaN is rejected by the driver before any conversion (ticket 1.1a)
def test_any_input_stays_within_the_gripper_range(value):
    assert CLOSED <= pct_to_rad(value) <= OPEN
    assert 0.0 <= rad_to_pct(value) <= 100.0
