"""Unit tests of the evaluation bench's scoring and checks (docs/project/08 §3). Pure: no ROS, no sim."""

import argparse
import inspect
import re

import pytest
import rclpy
from rclpy.task import Future

from so101_bringup import evaluate
from so101_bringup.evaluate import FIELDS, Bench, outcome, run, summary

BOX = (0.10, -0.25, 0.0)  # floor center


class FakeBench:
    """What run() reads of the bench, without ROS: the driver's seed, the ground truth, the brain's pose."""

    def __init__(self, pose, successes=None):
        self.truth = {"object": None, "drop_box": None}
        self.pose = pose
        self.successes = successes  # success of each episode; None: all succeed
        self.episodes = []
        self.first_wait = None
        self.waits = []  # timeout of each spin_until
        self.attempts = 1

    def driver_seed(self, timeout_s):
        self.first_wait = timeout_s
        return 0

    def spin_until(self, condition, timeout_s):
        self.waits.append(timeout_s)
        return bool(condition())

    def episode(self, number, seed, timeout_s):
        self.episodes.append(number)
        success = self.successes is None or self.successes[number - 1]
        return {**dict.fromkeys(FIELDS, 0), "episode": number, "seed": seed,
                "final_state": "DONE" if success else "ERROR:timeout:APPROACH", "duration_s": 15.0,
                "success": success, "cause": "" if success else "timeout:APPROACH"}  # fmt: skip


def bench_args(tmp_path, episodes=2, min_success=None):
    return argparse.Namespace(
        episodes=episodes,
        out=tmp_path / "eval.csv",
        bags=None,
        seed=None,
        timeout=60.0,
        min_success=min_success,
    )


def test_no_episode_starts_while_the_brain_does_not_see_joint_states(tmp_path, capsys):
    """A /brain/start before the brain's first /joint_states is refused (ERROR:no_joint_states, #132): the
    bench waits for the pose the brain computes from them, then gives up with exit 2, as its other checks."""
    bench = FakeBench(pose=None)
    assert run(bench_args(tmp_path), bench) == 2
    assert bench.episodes == []
    assert "/end_effector_pose" in capsys.readouterr().err
    assert not (tmp_path / "eval.csv").exists()


def test_the_first_call_waits_a_minute_for_the_driver_and_says_how_long_it_took(tmp_path, capsys):
    """A fresh process can find the stack late on a CI runner (#141): the first call to the driver waits up to
    60 s and the bench prints how long it took; later calls keep 10 s, so a driver that dies mid-run still
    stops the bench quickly."""
    bench = FakeBench(pose=object())
    assert run(bench_args(tmp_path), bench) == 0
    assert bench.first_wait == 60.0
    assert re.search(r"the driver answered in \d+\.\d s", capsys.readouterr().out)
    assert inspect.signature(Bench.call).parameters["timeout_s"].default == 10.0


class Pipe:
    """A stdout that is a pipe, as through `docker compose exec -T` (tools/so101 bench): what is written stays
    in the buffer until flush()."""

    def __init__(self):
        self.buffer, self.shown = "", ""

    def write(self, text):
        self.buffer += text
        return len(text)

    def flush(self):
        self.shown, self.buffer = self.shown + self.buffer, ""


def test_each_episode_line_shows_before_the_next_episode_starts(tmp_path, monkeypatch):
    """Through tools/so101 bench, stdout is a pipe: an unflushed line waited for the end of the run, the bench
    looked stuck for minutes and a newcomer stopped it (#58)."""
    out = Pipe()
    monkeypatch.setattr("sys.stdout", out)
    seen = []

    class Watched(FakeBench):
        def episode(self, number, seed, timeout_s):
            seen.append(out.shown)
            return super().episode(number, seed, timeout_s)

    assert run(bench_args(tmp_path, episodes=3), Watched(pose=object())) == 0
    assert "episode 1: DONE" in seen[1]
    assert "episode 2: DONE" in seen[2]
    assert "success 3/3" in out.shown  # the summary too, before the process ends


def test_ctrl_c_stops_the_bench_cleanly(tmp_path, monkeypatch, capsys):
    """From a terminal Ctrl-C now reaches evaluate (#58): rclpy's SIGINT handler shuts its context down and a
    KeyboardInterrupt comes out of the spin. A short message and exit 130, not two tracebacks and exit 1."""

    def interrupted(args, bench):
        rclpy.shutdown()  # what rclpy's SIGINT handler does before the KeyboardInterrupt
        raise KeyboardInterrupt

    monkeypatch.setattr(evaluate, "run", interrupted)
    assert evaluate.main(["--episodes", "3", "--out", str(tmp_path / "eval.csv")]) == 130
    assert "stopped (Ctrl-C)" in capsys.readouterr().err


def test_the_ground_truth_and_the_brain_pose_get_the_same_minute(tmp_path):
    """Once in CI the driver answered but /end_effector_pose did not come within 10 s (#141): every check
    before the first episode (driver, ground truth, brain pose) waits up to 60 s."""
    bench = FakeBench(pose=object())
    assert run(bench_args(tmp_path), bench) == 0
    assert bench.waits[:2] == [60.0, 60.0]


class LossyService:
    """A service found by wait_for_service whose first `lost` replies never come (#141)."""

    srv_name = "/so101_driver/get_parameters"

    def __init__(self, lost):
        self.lost, self.sent = lost, 0

    def wait_for_service(self, timeout_sec):
        return True

    def call_async(self, request):
        self.sent += 1
        future = Future()
        if self.sent > self.lost:
            future.set_result("reply")
        return future


@pytest.fixture
def clocked_bench(monkeypatch):
    """Bench.call without ROS: spin_until advances a fake clock by its timeout when nothing comes."""
    clock = [0.0]
    monkeypatch.setattr(evaluate.time, "monotonic", lambda: clock[0])
    bench = Bench.__new__(Bench)

    def spin_until(condition, timeout_s):
        if not condition():
            clock[0] += timeout_s
        return bool(condition())

    bench.spin_until = spin_until
    return bench, clock


def test_an_idempotent_request_without_reply_is_sent_again(clocked_bench):
    """In CI the driver was found, then its reply never came in 60 s (#141): it is asked again every 5 s."""
    bench, clock = clocked_bench
    service = LossyService(lost=2)
    assert bench.call(service, "request", 60.0, resend_s=5.0) == "reply"
    assert (service.sent, bench.attempts, clock[0]) == (3, 3, 10.0)


def test_other_requests_are_sent_once_and_stop_the_bench_without_reply(clocked_bench):
    """/sim/randomize or /brain/start twice would change the episode: no resend."""
    bench, _ = clocked_bench
    service = LossyService(lost=1)
    with pytest.raises(RuntimeError, match="no answer in 10 s"):
        bench.call(service, "request", 10.0)
    assert service.sent == 1


@pytest.mark.parametrize(
    ("successes", "min_success", "code"),
    [
        ([True, False, True], 3, 1),
        ([True, True, True], 3, 0),
        ([True, False, True], 2, 0),
        ([False, False, False], None, 0),  # without the option: the nightly stays informative (#151)
    ],
)
def test_min_success_fails_the_run_below_it_after_the_csv_and_the_summary(
    tmp_path, capsys, successes, min_success, code
):
    """#156: the PR's E2E job fails on a failed pick & place, not only on a crash."""
    bench = FakeBench(pose=object(), successes=successes)
    assert run(bench_args(tmp_path, len(successes), min_success), bench) == code
    assert len((tmp_path / "eval.csv").read_text().splitlines()) == 1 + len(successes)
    out = capsys.readouterr()
    assert re.search(rf"^success {sum(successes)}/{len(successes)} ", out.out, re.MULTILINE)
    assert (f"fewer than --min-success {min_success}" in out.err) == bool(code)


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
    assert "p50 15.0 s" in text  # successes only: a timeout is not a duration
    assert "p95 19.5 s" in text
    assert "unreachable: 2" in text
    assert "bench_timeout: 1" in text


def test_the_summary_of_no_success_has_no_duration():
    assert summary([{"success": False, "duration_s": 3.0, "cause": "unreachable"}]).startswith(
        "success 0/1 (0 %); duration: no success; causes: unreachable: 1"
    )


def test_the_summary_of_only_successes_has_no_cause():
    rows = [{"success": True, "duration_s": d, "cause": ""} for d in (14.0, 16.0)]
    assert summary(rows) == "success 2/2 (100 %); duration p50 15.0 s, p95 15.9 s; causes: none"


class Recorded(FakeBench):
    """A bench with --bags: what happens around an episode, in order."""

    def __init__(self, listens=True):
        super().__init__(pose=object())
        self.events, self.listens = [], listens

    def listeners(self, topics):
        self.events.append("listeners")
        return {"/joint_states": 1}

    def joined(self, before):
        self.events.append(("joined", before))
        return self.listens

    def episode(self, number, seed, timeout_s):
        self.events.append("episode")
        return super().episode(number, seed, timeout_s)


def recorded_run(tmp_path, monkeypatch, bench):
    class Recorder:
        def send_signal(self, _signal):
            bench.events.append("stop")

        def wait(self, timeout):
            pass

    monkeypatch.setattr(evaluate, "record", lambda path, topics: bench.events.append("record") or Recorder())
    args = bench_args(tmp_path, episodes=1)
    args.bags = tmp_path / "bags"
    return run(args, bench)


def test_with_bags_an_episode_starts_once_its_recorder_listens(tmp_path, monkeypatch, capsys):
    """A DDS participant that joins holds the driver's executor for 0.3 s, once in ten [MES-80]. The recorder
    takes 2 s to start: joined during the cycle, it cost the nightly an episode twice (`no_joint_states`,
    #274). The bench counts the topics' subscribers, starts the recorder, waits for it, then the episode."""
    bench = Recorded()
    assert recorded_run(tmp_path, monkeypatch, bench) == 0
    assert bench.events == ["listeners", "record", ("joined", {"/joint_states": 1}), "episode", "stop"]
    assert capsys.readouterr().err == ""


def test_a_recorder_that_never_listens_is_said_and_the_episode_runs(tmp_path, monkeypatch, capsys):
    bench = Recorded(listens=False)
    assert recorded_run(tmp_path, monkeypatch, bench) == 0
    assert bench.episodes == [1]
    assert "the recorder of episode 1 did not subscribe" in capsys.readouterr().err


def test_a_recorder_is_seen_though_the_one_before_it_was_still_counted(monkeypatch):
    """The recorder of the episode before may still be in the graph when the subscribers are counted: it
    leaves, the new one joins, and the count is back where it was. Joined is one more than the lowest seen."""

    class Graph:
        counts = iter([1, 1, 2, 2, 2, 2])  # the old recorder leaves (2 before, then 1), the new one joins

        def count_subscribers(self, topic):
            return next(self.counts, 2)

    class Spun:
        node = Graph()

        def spin_until(self, condition, timeout_s):
            return any(condition() for _ in range(4))

    monkeypatch.setattr(evaluate, "SETTLE_S", 0.0)
    assert Bench.joined(Spun(), {"/joint_states": 2}) is True
