"""System test (launch_testing, sim): the evaluation bench CLI of 08 §3 against gt_stub.launch.py (driver
with ground truth, brain, ground-truth perception stub). Two episodes succeed and keep no rosbag; an episode
cut by a short bench timeout fails and keeps its rosbag."""

import csv
import subprocess
import tempfile
import unittest
from pathlib import Path

import launch
import launch.actions
import launch.launch_description_sources
import launch_testing.actions
import pytest

GT_STUB = Path(__file__).parent / "gt_stub.launch.py"
FIELDS = [
    "episode", "seed", "cube_x", "cube_y", "cube_z", "box_x", "box_y", "box_z",
    "final_state", "duration_s", "success", "cause",
]  # fmt: skip


@pytest.mark.launch_test
def generate_test_description():
    stack = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(str(GT_STUB))
    )
    return launch.LaunchDescription([stack, launch_testing.actions.ReadyToTest()])


def evaluate(*args, timeout=300):
    return subprocess.run(
        ["ros2", "run", "so101_bringup", "evaluate", *args], capture_output=True, text=True, timeout=timeout
    )


class TestEvaluate(unittest.TestCase):
    def test_1_two_episodes_succeed_and_keep_no_rosbag(self):
        with tempfile.TemporaryDirectory() as tmp:
            out, bags = Path(tmp) / "eval.csv", Path(tmp) / "bags"
            run = evaluate("--episodes", "2", "--out", str(out), "--bags", str(bags), "--seed", "0")
            self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
            with out.open() as f:
                rows = list(csv.DictReader(f))
            self.assertEqual(list(rows[0]), FIELDS)
            self.assertEqual([r["episode"] for r in rows], ["1", "2"])
            for r in rows:
                self.assertEqual(
                    (r["seed"], r["final_state"], r["success"], r["cause"]), ("0", "DONE", "True", ""), r
                )
                self.assertLess(abs(float(r["cube_x"]) - float(r["box_x"])), 0.05, r)
                self.assertLess(float(r["duration_s"]), 60.0)
            self.assertIn("success 2/2 (100 %)", run.stdout)
            self.assertNotIn("recorder", run.stderr, "an episode started before its recorder listened (#274)")
            self.assertEqual(list(bags.glob("*")), [], "a successful episode kept its rosbag")

    def test_2_a_cut_episode_fails_and_keeps_its_rosbag(self):
        with tempfile.TemporaryDirectory() as tmp:
            out, bags = Path(tmp) / "eval.csv", Path(tmp) / "bags"
            run = evaluate("--episodes", "1", "--out", str(out), "--bags", str(bags), "--timeout", "3")
            self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
            with out.open() as f:
                (row,) = list(csv.DictReader(f))
            self.assertEqual(
                (row["final_state"], row["success"], row["cause"]), ("TIMEOUT", "False", "bench_timeout")
            )
            self.assertIn("bench_timeout: 1", run.stdout)
            self.assertTrue((bags / "episode_001" / "metadata.yaml").is_file(), list(bags.rglob("*")))

    def test_3_a_seed_other_than_the_drivers_is_refused(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = evaluate("--episodes", "1", "--out", str(Path(tmp) / "eval.csv"), "--seed", "7", timeout=60)
            self.assertEqual(run.returncode, 2)
            self.assertIn("seed", run.stderr)
            self.assertFalse((Path(tmp) / "eval.csv").exists())
