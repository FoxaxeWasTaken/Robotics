"""Unit tests of the robustness demo's cases, triggers and verdicts (step 5.4a, docs/project/06 §10). Pure: no
ROS, no sim."""

from pathlib import Path

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

from so101_bringup.robustness import CASES, MAX_RETRIES, Injector, end_state, lighting_verdict, verdict

CONFIG = Path(get_package_share_directory("so101_bringup")) / "config"


def test_case_2_counts_the_deployed_retries():
    brain = yaml.safe_load((CONFIG / "brain.yaml").read_text())["so101_brain"]["ros__parameters"]
    assert brain["fsm"]["max_retries"] == MAX_RETRIES


def test_the_five_slide_cases_are_covered():
    """[BOOT2 p12] has five cases; the third (the object leaves the view) has two scenarios."""
    assert sorted(CASES) == ["1", "2", "3a", "3b", "4", "5"]
    assert all(case.question and case.expected for case in CASES.values())


def test_case_1_moves_the_cube_once_on_approach():
    injector = Injector(CASES["1"])
    assert injector.before_start() == []
    assert injector.on_state("WAIT_TARGET") is None
    assert injector.on_state("APPROACH") == "randomize"
    assert injector.on_state("APPROACH") is None  # the retry's approach is left alone


def test_case_2_moves_the_cube_on_every_descend():
    injector = Injector(CASES["2"])
    assert [injector.on_state("DESCEND") for _ in range(3)] == ["randomize"] * 3


def test_cases_3a_3b_and_4_stop_perception_at_their_moment():
    assert Injector(CASES["3a"]).on_state("TRANSPORT") == "stop_perception"
    assert Injector(CASES["3a"]).before_start() == []
    assert Injector(CASES["3b"]).before_start() == ["stop_perception"]
    assert Injector(CASES["4"]).before_start() == ["stop_perception", "unreachable_target"]
    assert Injector(CASES["4"]).on_state("WAIT_TARGET") is None


def test_case_5_injects_nothing_during_the_cycle():
    injector = Injector(CASES["5"])
    assert injector.before_start() == []
    assert all(injector.on_state(s) is None for s in ("WAIT_TARGET", "APPROACH", "DESCEND", "TRANSPORT"))


# Observed /brain_state sequences, as the 4.4a runs printed them (#54).
CASE_1_RUN = ["IDLE", "WAIT_TARGET", "PLAN", "APPROACH", "DESCEND", "GRASP", "RETRY", "HOME_RETRY",
              "WAIT_TARGET", "PLAN", "APPROACH", "DESCEND", "GRASP", "LIFT", "TRANSPORT", "RELEASE", "HOME",
              "VERIFY", "DONE"]  # fmt: skip
CASE_3A_RUN = ["IDLE", "WAIT_TARGET", "PLAN", "APPROACH", "DESCEND", "GRASP", "LIFT", "TRANSPORT", "RELEASE",
               "HOME", "VERIFY", "DONE"]  # fmt: skip


def test_the_4_4a_runs_are_handled():
    assert verdict(CASES["1"], CASE_1_RUN) == ("handled", "")
    assert verdict(CASES["3a"], CASE_3A_RUN) == ("handled", "")


def test_a_run_is_judged_on_its_first_end_state():
    """DONE → IDLE follows at once (06 §3.1): the demo, still listening for the last reason, sees it too."""
    assert verdict(CASES["1"], [*CASE_1_RUN, "IDLE"]) == ("handled", "")
    assert verdict(CASES["3b"], ["IDLE", "WAIT_TARGET", "ERROR:object_lost", "IDLE"]) == ("handled", "")


def test_the_end_state_is_the_first_one():
    """What case 5 counts as a success, per cycle: the IDLE after DONE does not hide it."""
    assert end_state([*CASE_1_RUN, "IDLE"]) == "DONE"
    assert end_state(["IDLE", "WAIT_TARGET", "ERROR:object_lost", "IDLE"]) == "ERROR:object_lost"
    assert end_state(["IDLE", "WAIT_TARGET"]) is None


def test_a_wrong_end_is_not_handled():
    assert verdict(CASES["1"], [*CASE_1_RUN[:-1], "ERROR:timeout:VERIFY"]) == (
        "not handled",
        "ended in ERROR:timeout:VERIFY, expected DONE",
    )


def test_no_end_is_not_handled():
    assert verdict(CASES["1"], CASE_1_RUN[:5]) == ("not handled", "no end state (DESCEND last)")


def test_the_right_end_by_another_path_is_partly_handled():
    """Case 1 ending in DONE without a RETRY: the move did not reach the grasp (the cube landed in reach)."""
    assert verdict(CASES["1"], CASE_3A_RUN) == ("partly", "never went through RETRY")


def test_a_motion_where_none_is_allowed_is_not_handled():
    """3b and 4: the arm must not leave PARK, so no APPROACH, even when the end state is the expected one."""
    run = ["IDLE", "WAIT_TARGET", "PLAN", "APPROACH", "ERROR:object_lost"]
    assert verdict(CASES["3b"], run) == ("not handled", "went through APPROACH")
    assert verdict(CASES["3b"], ["IDLE", "WAIT_TARGET", "ERROR:object_lost"]) == ("handled", "")
    assert verdict(CASES["4"], ["IDLE", "WAIT_TARGET", "PLAN", "ERROR:unreachable"]) == ("handled", "")


def test_case_2_needs_every_attempt_to_fail():
    attempt = ["WAIT_TARGET", "PLAN", "APPROACH", "DESCEND", "GRASP", "RETRY", "HOME_RETRY"]
    run = ["IDLE", *attempt * 3, "ERROR:grasp_failed"]
    assert verdict(CASES["2"], run) == ("handled", "")
    assert verdict(CASES["2"], ["IDLE", *attempt, "ERROR:grasp_failed"]) == (
        "partly",
        "went through RETRY 1 time(s), expected 3",
    )


@pytest.mark.parametrize(
    ("successes", "expected"),
    [(5, "handled"), (3, "partly"), (1, "partly"), (0, "not handled")],
)
def test_lighting_is_judged_on_the_cycles(successes, expected):
    assert lighting_verdict(successes, 5)[0] == expected
