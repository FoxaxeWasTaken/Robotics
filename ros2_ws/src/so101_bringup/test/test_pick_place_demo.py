"""pick_place_demo (#282): the console of `tools/so101 demo`, its timeline, its counts and its keys, with a
fake brain. No ROS: the services and the topics are test_pick_place_gt_launch.py's and the stack's."""

from so101_bringup.pick_place_demo import Console, Report, render

CYCLE = [
    "WAIT_TARGET",
    "PLAN",
    "APPROACH",
    "DESCEND",
    "GRASP",
    "LIFT",
    "TRANSPORT",
    "RELEASE",
    "HOME",
    "VERIFY",
]


def run(report, states, start=0.0, step=1.0):
    """A cycle begun at `start`, a state each `step` seconds; the time after the last one."""
    report.begin(start)
    for i, state in enumerate(states):
        report.on_state(state, start + i * step)
    return start + (len(states) - 1) * step


def test_each_step_has_its_duration_and_the_running_one_its_time_so_far():
    report = Report()
    run(report, ["WAIT_TARGET", "PLAN", "APPROACH"], step=2.0)
    report.on_state("APPROACH", 4.5)  # the brain says its state again each second: no new step
    assert report.timeline(now=5.5) == [
        ("WAIT_TARGET", 2.0, True),
        ("PLAN", 2.0, True),
        ("APPROACH", 1.5, False),
    ]
    assert report.running


def test_done_ends_the_cycle_and_counts_its_time():
    report = Report()
    end = run(report, [*CYCLE, "DONE"])
    assert not report.running and (report.done, report.failed) == (1, 0)
    assert report.times == [end]
    assert report.timeline(now=99.0)[-1] == ("DONE", 0.0, True)  # the end has no duration, and no clock
    run(report, [*CYCLE, "DONE"], start=20.0, step=0.5)
    assert report.times == [10.0, 5.0]


def test_an_error_is_a_failed_cycle_with_its_reason_and_a_retry_is_counted():
    report = Report()
    run(report, ["WAIT_TARGET", "PLAN", "APPROACH", "DESCEND", "GRASP", "RETRY", "HOME_RETRY", "WAIT_TARGET"])
    report.on_state("ERROR:grasp_failed", 9.0)
    assert not report.running and (report.done, report.failed, report.retries) == (0, 1, 1)
    assert report.times == []
    assert report.timeline(now=9.0)[-1][0] == "ERROR:grasp_failed"


def test_states_out_of_a_cycle_and_the_last_cycles_end_are_not_steps():
    report = Report()
    report.on_state("IDLE", 0.0)  # before any cycle
    assert report.timeline(now=1.0) == []
    report.begin(1.0)
    report.on_state("DONE", 1.1)  # the end of the cycle before, said again as this one starts
    report.on_state("IDLE", 1.2)
    assert report.running and report.timeline(now=2.0) == []


def test_the_screen_shows_the_poses_the_steps_and_the_counts():
    report = Report()
    run(report, [*CYCLE, "DONE"])
    run(report, ["WAIT_TARGET", "PLAN"], start=20.0)
    poses = {"cube": (0.2141, -0.0523, 0.0151), "box": None, "hand": (0.21, -0.05, 0.12)}
    text = render(report, now=22.5, state="PLAN", poses=poses, message="")
    lines = text.splitlines()
    assert any("cube" in line and "+0.214" in line and "-0.052" in line for line in lines)
    assert any("box" in line and "not seen" in line for line in lines)
    assert any("ok" in line and "WAIT_TARGET" in line and "1.0 s" in line for line in lines)
    assert any(">" in line and "PLAN" in line and "1.5 s" in line for line in lines)
    assert any("cycle 2" in line for line in lines)
    assert any("1 done" in line and "0 failed" in line and "last 10.0 s" in line for line in lines)
    assert "\x1b[" not in text  # no color unless asked: a pipe, a test
    assert "\x1b[32m" in render(report, now=22.5, state="PLAN", poses=poses, message="", color=True)


class FakeArm:
    """The brain and the sim as the console sees them."""

    def __init__(self, state="IDLE"):
        self.state, self.poses, self.calls, self.refusal = state, {}, [], None

    def randomize(self):
        self.calls.append("randomize")

    def start(self):
        self.calls.append("start")
        if self.state.startswith("ERROR"):  # the brain's rule: a start in ERROR only re-arms (06 §3.1)
            self.state = "IDLE"
            return True, "re-armed"
        return (False, self.refusal) if self.refusal else (True, "started")

    def stop(self):
        self.calls.append("stop")
        return True, "stopped"


def console(arm, live=False):
    written, clock = [], [0.0]
    made = Console(arm, written.append, clock=lambda: clock[0], live=live)
    return made, written, clock


def test_the_first_cycle_starts_by_itself_at_a_new_place_then_n_starts_the_next():
    arm = FakeArm()
    made, _, clock = console(arm)
    assert made.step() and arm.calls == ["randomize", "start"]
    assert made.step() and arm.calls == ["randomize", "start"]  # once
    assert made.step("n") and arm.calls == ["randomize", "start"]  # a cycle is running: refused, said
    assert "running" in made.message
    for state in (*CYCLE, "DONE"):
        clock[0] += 1.0
        arm.state = state
        made.step()
    assert made.report.done == 1
    assert made.step("n") and arm.calls == ["randomize", "start"] * 2 and made.report.running


def test_no_state_yet_starts_nothing_and_says_why():
    arm = FakeArm(state=None)
    made, _, clock = console(arm)
    assert (
        made.step() and arm.calls == [] and made.message == ""
    )  # the first moments: the state is on its way
    clock[0] = 3.0
    assert made.step() and arm.calls == []
    assert "/brain_state" in made.message
    arm.state = "IDLE"
    assert made.step() and arm.calls == ["randomize", "start"]


def test_a_refused_start_is_said_and_not_tried_again_by_itself():
    arm = FakeArm()
    arm.refusal = "no_joint_states"
    made, _, _ = console(arm)
    made.step()
    made.step()
    assert arm.calls == ["randomize", "start"] and not made.report.running
    assert "no_joint_states" in made.message


def test_after_an_error_n_rearms_the_brain_then_starts():
    arm = FakeArm(state="ERROR:grasp_failed")
    made, _, _ = console(arm)
    made.step()  # the first cycle, by itself
    assert arm.calls == ["start", "randomize", "start"] and made.report.running


def test_s_stops_the_brain_and_q_quits():
    arm = FakeArm()
    made, _, _ = console(arm)
    made.step()
    assert made.step("s") and arm.calls[-1] == "stop"
    assert made.step("q") is False


def test_through_a_pipe_each_step_is_one_line_and_a_terminal_gets_the_whole_screen():
    arm = FakeArm()
    made, written, clock = console(arm)
    made.step()
    for state in ("WAIT_TARGET", "PLAN", "PLAN", "DONE"):
        clock[0] += 1.0
        arm.state = state
        made.step()
    assert [line.split()[0] for line in written if line.split()[0].isupper()] == [
        "WAIT_TARGET",
        "PLAN",
        "DONE",
    ]
    assert any("1 done" in line for line in written)

    made, written, clock = console(FakeArm(), live=True)
    made.step()
    assert written and "n: a new cycle" in written[-1] and "cycle 1" in written[-1]
    count = len(written)
    made.step()  # nothing changed: nothing written again
    assert len(written) == count
