"""Unit tests of the evaluation bench's scoring and checks (docs/project/08 §3). Pure: no ROS, no sim."""

import argparse

import pytest

from so101_bringup.evaluate import FIELDS, outcome, run, summary

BOX = (0.10, -0.25, 0.0)  # floor center


class FakeBench:
    """What run() reads of the bench, without ROS: the driver's seed, the ground truth, the brain's pose."""

    def __init__(self, pose):
        self.truth = {"object": None, "drop_box": None}
        self.pose = pose
        self.episodes = []

    def driver_seed(self):
        return 0

    def spin_until(self, condition, timeout_s):
        return bool(condition())

    def episode(self, number, seed, timeout_s):
        self.episodes.append(number)
        return {**dict.fromkeys(FIELDS, 0), "episode": number, "seed": seed, "final_state": "DONE",
                "duration_s": 15.0, "success": True, "cause": ""}  # fmt: skip


def bench_args(tmp_path):
    return argparse.Namespace(episodes=2, out=tmp_path / "eval.csv", bags=None, seed=None, timeout=60.0)


def test_no_episode_starts_while_the_brain_does_not_see_joint_states(tmp_path, capsys):
    """A /brain/start before the brain's first /joint_states is refused (ERROR:no_joint_states, #132): the
    bench waits for the pose the brain computes from them, then gives up with exit 2, as its other checks."""
    bench = FakeBench(pose=None)
    assert run(bench_args(tmp_path), bench) == 2
    assert bench.episodes == []
    assert "/end_effector_pose" in capsys.readouterr().err
    assert not (tmp_path / "eval.csv").exists()


def test_the_episodes_run_once_the_brain_publishes_its_pose(tmp_path):
    bench = FakeBench(pose=object())
    assert run(bench_args(tmp_path), bench) == 0
    assert bench.episodes == [1, 2]


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
