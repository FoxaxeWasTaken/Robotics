"""pick_place_demo (#282): the console of `tools/so101 demo`, its timeline, its counts and its keys, with a
fake brain. No ROS: the services and the topics are test_pick_place_gt_launch.py's and the stack's."""

from so101_bringup.pick_place_demo import Console, Report, render
from so101_common.terminal import WIDTH, Pen

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


def test_the_progress_is_the_ten_steps_of_the_attempt_in_hand():
    report = Report()
    run(report, ["WAIT_TARGET", "PLAN", "APPROACH"], step=2.0)
    progress = report.progress(now=5.5)
    assert [state for state, _, _ in progress] == CYCLE
    assert progress[:4] == [
        ("WAIT_TARGET", 2.0, "done"),
        ("PLAN", 2.0, "done"),
        ("APPROACH", 1.5, "run"),
        ("DESCEND", None, "todo"),
    ]
    assert report.elapsed(now=5.5) == 5.5 and report.attempt == 1


def test_a_missed_grasp_marks_its_step_failed_and_the_next_attempt_starts_the_ten_steps_again():
    report = Report()
    run(report, ["WAIT_TARGET", "PLAN", "APPROACH", "DESCEND", "GRASP", "RETRY", "HOME_RETRY"])
    marks = {state: mark for state, _, mark in report.progress(now=7.0)}
    assert (marks["DESCEND"], marks["GRASP"], marks["LIFT"]) == ("done", "fail", "todo")
    report.on_state("WAIT_TARGET", 8.0)
    report.on_state("PLAN", 9.0)
    assert report.attempt == 2
    assert report.progress(now=9.5)[:3] == [
        ("WAIT_TARGET", 1.0, "done"),
        ("PLAN", 0.5, "run"),
        ("APPROACH", None, "todo"),
    ]
    assert report.elapsed(now=9.5) == 9.5  # the cycle's time, its first attempt included


def test_a_finished_cycle_keeps_its_ten_steps_done_and_its_time():
    report = Report()
    end = run(report, [*CYCLE, "DONE"])
    assert all(mark == "done" for _, _, mark in report.progress(now=99.0))
    assert report.elapsed(now=99.0) == end


def test_the_counts_say_how_many_cubes_reached_the_box_and_how_fast():
    report = Report()
    assert report.counts() == "no cycle finished yet"
    run(report, [*CYCLE, "DONE"])
    run(report, ["WAIT_TARGET"], start=20.0)
    report.on_state("ERROR:object_lost", 30.0)
    run(report, [*CYCLE, "DONE"], start=40.0, step=2.0)
    text = report.counts()
    assert "2 of 3 cubes in the box" in text
    assert "last 20.0 s" in text and "average 15.0 s" in text and "best 10.0 s" in text


POSES = {"cube": (0.2141, -0.0523, 0.0151), "box": None}


def screen(report, now, state, message="", pretty=False):
    return render(report, now, state, POSES, message, Pen(pretty)).splitlines()


def test_the_screen_is_a_header_the_scene_the_cycle_with_its_bar_the_total_a_status_and_the_keys():
    report = Report()
    run(report, [*CYCLE, "DONE"])
    run(report, ["WAIT_TARGET", "PLAN", "APPROACH", "DESCEND"], start=20.0)
    lines = screen(report, 23.5, "DESCEND")
    assert lines[0].startswith(" SO-101 | Pick & place, simulation") and lines[0].endswith("DESCEND")
    assert all(len(line) <= WIDTH for line in lines) and len(lines) == 17
    assert any("cube" in line and "x  21.4 cm" in line and "y  -5.2 cm" in line for line in lines)
    assert any(line.rstrip() == "  box    not seen by the camera" for line in lines)
    assert any(line.startswith("-- cycle 2 --") for line in lines)
    bar = next(line for line in lines if "/10" in line)
    assert "#" * 15 + "=" * 5 + "-" * 30 in bar and "3/10" in bar and "3.5 s" in bar
    # the steps in plain words, two columns: the pick on the left, the place on the right
    assert any(
        "+ Moving above the cube" in line and "1.0 s" in line and ". Dropping" in line for line in lines
    )
    assert any("> Going down" in line and "0.5 s" in line and ". Returning" in line for line in lines)
    assert any("1 of 1 cube in the box" in line and "last 10.0 s" in line for line in lines)
    assert lines[-3] == " > cycle 2: going down"
    assert lines[-2] == "" and lines[-1] == " n new cycle  s stop  q quit"
    assert all(
        line.isascii() and "\x1b" not in line for line in lines
    )  # a pipe, a test: no color, no box character
    pretty = "\n".join(screen(report, 23.5, "DESCEND", pretty=True))
    assert "\x1b[" in pretty and "█" in pretty and "►" in pretty


def test_the_status_says_the_end_of_a_cycle_in_plain_words():
    report = Report()
    assert screen(report, 0.0, None)[-3] == " > waiting for the brain's state"
    assert screen(report, 0.0, "IDLE")[-3] == " > starting the first cycle"
    assert any(line.startswith("-- no cycle yet --") for line in screen(report, 0.0, "IDLE"))
    refused = screen(report, 0.0, "IDLE", message="start refused: no_joint_states")
    assert refused[-3] == " > n: a new cycle"  # nothing is starting any more
    run(report, [*CYCLE, "DONE"])
    assert (
        screen(report, 11.0, "DONE")[-3]
        == " + cube in the box, checked by the camera (10.0 s); n: a new cycle"
    )
    run(report, ["WAIT_TARGET", "PLAN", "APPROACH", "DESCEND", "GRASP", "RETRY", "HOME_RETRY"], start=20.0)
    lines = screen(report, 27.0, "HOME_RETRY")
    assert lines[-3] == " > cycle 2: missed the cube, trying again"
    assert any("x Closing the gripper" in line for line in lines)
    report.on_state("WAIT_TARGET", 28.0)
    assert any(line.startswith("-- cycle 2, attempt 2 --") for line in screen(report, 28.5, "WAIT_TARGET"))
    report.on_state("ERROR:object_lost", 38.0)
    assert (
        screen(report, 39.0, "ERROR:object_lost")[-3] == " x the camera did not find the cube; n: a new cycle"
    )
    assert (
        screen(report, 39.0, "ERROR:object_lost", message="start refused: busy")[-2]
        == "   start refused: busy"
    )


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
    assert any("1 of 1 cube in the box" in line for line in written)

    made, written, clock = console(FakeArm(), live=True)
    made.step()
    assert written and "n new cycle" in written[-1] and "cycle 1" in written[-1]
    assert written[-1].startswith("\x1b[H")  # painted over the frame before, no screen cleared first
    count = len(written)
    made.step()  # nothing changed: nothing written again
    assert len(written) == count
