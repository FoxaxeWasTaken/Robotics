"""Unit tests of the temporal filter (docs/project/05 §7): median of 5, a jump over 2 cm rejected unless it
persists 5 times. Pure: no ROS."""

import numpy as np

from so101_perception.filter import MedianFilter

ORIGIN, MOVED = np.zeros(3), np.array([0.1, 0.0, 0.0])


def test_the_first_measurement_comes_out_as_is():
    np.testing.assert_array_equal(MedianFilter().update([0.2, 0.1, 0.025]), [0.2, 0.1, 0.025])


def test_the_output_is_the_median_of_the_last_5():
    f = MedianFilter()
    for x in (0.001, 0.009, 0.003, 0.004, 0.002, 0.005):  # the first one leaves the window
        out = f.update([x, 0.0, 0.0])
    np.testing.assert_allclose(out, [0.004, 0.0, 0.0])


def test_a_jump_over_2_cm_is_rejected_and_leaves_the_median_alone():
    f = MedianFilter()
    for _ in range(5):
        f.update(ORIGIN)
    assert f.update([0.021, 0.0, 0.0]) is None
    np.testing.assert_allclose(f.update([0.019, 0.0, 0.0]), ORIGIN)


def test_a_jump_that_persists_5_times_is_the_object_that_moved():
    f = MedianFilter()
    for _ in range(5):
        f.update(ORIGIN)
    assert [f.update(MOVED) for _ in range(4)] == [None] * 4
    np.testing.assert_allclose(f.update(MOVED), MOVED)
    np.testing.assert_allclose(f.update(MOVED + 0.001), MOVED)


def test_a_measurement_back_near_the_median_restarts_the_count():
    f = MedianFilter()
    for _ in range(5):
        f.update(ORIGIN)
    for _ in range(4):
        f.update(MOVED)
    f.update(ORIGIN)
    assert [f.update(MOVED) for _ in range(4)] == [None] * 4
