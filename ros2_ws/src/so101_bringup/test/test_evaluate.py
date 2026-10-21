"""Unit tests of the evaluation bench's scoring (docs/project/08 §3). Pure: no ROS, no sim."""

import pytest

from so101_bringup.evaluate import outcome, summary

BOX = (0.10, -0.25, 0.0)  # floor center


@pytest.mark.parametrize(
    ("cube", "inside"),
    [
        ((0.10, -0.25, 0.025), True),
        ((0.149, -0.25, 0.025), True),  # 4.9 cm off in x: still within the 5 cm footprint
        ((0.151, -0.25, 0.025), False),
        ((0.10, -0.201, 0.025), True),
        ((0.10, -0.199, 0.025), False),
        ((0.10, -0.25, 0.059), True),
        ((0.10, -0.25, 0.061), False),  # z >= 6 cm: on a wall, not in the box
    ],
)
def test_success_is_done_with_the_cube_in_the_box_footprint_below_6_cm(cube, inside):
    assert outcome("DONE", cube, BOX) == (inside, "" if inside else "not_in_box")


@pytest.mark.parametrize(
    ("final", "cause"),
    [
        ("ERROR:timeout:PLAN", "timeout:PLAN"),
        ("ERROR:unreachable", "unreachable"),
        ("TIMEOUT", "bench_timeout"),
    ],
)
def test_a_failed_cycle_gives_its_cause_even_with_the_cube_in_the_box(final, cause):
    assert outcome(final, (0.10, -0.25, 0.025), BOX) == (False, cause)


def test_the_summary_gives_the_rate_the_durations_of_the_successes_and_the_causes():
    rows = [
        {"success": True, "duration_s": 10.0, "cause": ""},
        {"success": True, "duration_s": 20.0, "cause": ""},
        {"success": False, "duration_s": 60.0, "cause": "bench_timeout"},
        {"success": False, "duration_s": 3.0, "cause": "unreachable"},
        {"success": False, "duration_s": 4.0, "cause": "unreachable"},
    ]
    text = summary(rows)
    assert "success 2/5 (40 %)" in text
    assert "p50 15.0 s" in text and "p95 19.5 s" in text  # successes only: a timeout is not a duration
    assert "unreachable: 2" in text and "bench_timeout: 1" in text


def test_the_summary_of_no_success_has_no_duration():
    assert summary([{"success": False, "duration_s": 3.0, "cause": "unreachable"}]).startswith(
        "success 0/1 (0 %); duration: no success; causes: unreachable: 1"
    )


def test_the_summary_of_only_successes_has_no_cause():
    rows = [{"success": True, "duration_s": d, "cause": ""} for d in (14.0, 16.0)]
    assert summary(rows) == "success 2/2 (100 %); duration p50 15.0 s, p95 15.9 s; causes: none"
