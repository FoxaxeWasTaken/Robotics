"""Robustness demo (step 5.4a, docs/project/06 §10): the cases of [BOOT2 p12], when to inject each fault, and
the verdict on the /brain_state sequence a run printed. Pure: no ROS, no sim (robustness_demo.py runs them).

The faults go through the sim-only services or the demo's own stack (stopping its perception node), never
through the ground truth (C-06): the pipeline still sees the scene through the camera only.
"""

from dataclasses import dataclass

END_STATES = ("DONE", "STOPPED", "ERROR")  # a cycle ends in one of them (ERROR with its cause)
MAX_RETRIES = 3  # fsm.max_retries in so101_bringup/config/brain.yaml: failed attempts before grasp_failed


@dataclass(frozen=True)
class Case:
    """One scenario: its fault, where it is injected, and the /brain_state sequence that handles it."""

    question: str  # the slide's question [BOOT2 p12]
    expected: str  # the end state that handles it
    before: tuple = ()  # actions before /brain/start
    trigger: str | None = None  # the state whose entry injects `action`
    action: str | None = None
    every: bool = False  # inject at each entry of `trigger`, not only the first
    required: str | None = None  # a state the run must go through...
    required_count: int = 1  # ... at least this many times
    forbidden: tuple = ()  # states the run must never reach (a motion where none is allowed)


CASES = {
    "1": Case(
        question="The object moves mid-action: recompute the movement?",
        expected="DONE",
        trigger="APPROACH",
        action="randomize",
        required="RETRY",
    ),
    "2": Case(
        question="The grasp fails: how do you know, what do you do?",
        expected="ERROR:grasp_failed",
        trigger="DESCEND",
        action="randomize",
        every=True,
        required="RETRY",
        required_count=MAX_RETRIES,
    ),
    "3a": Case(
        question="The object leaves the view (during the transport): what does perception publish?",
        expected="DONE",
        trigger="TRANSPORT",
        action="stop_perception",
    ),
    "3b": Case(
        question="The object leaves the view (before the cycle): is there a memory?",
        expected="ERROR:object_lost",
        before=("stop_perception",),
        forbidden=("APPROACH",),
    ),
    "4": Case(
        question="Target out of reach: what does IK return, does the arm move anyway?",
        expected="ERROR:unreachable",
        before=("stop_perception", "unreachable_target"),
        forbidden=("APPROACH",),
    ),
    "5": Case(
        question="Lighting changes: does the color threshold still hold?",
        expected="DONE",
    ),
}


class Injector:
    """What to do before /brain/start, and on each new /brain_state of the run."""

    def __init__(self, case: Case):
        self.case = case
        self._fired = False

    def before_start(self) -> list[str]:
        return list(self.case.before)

    def on_state(self, state: str) -> str | None:
        """The action to run now, or None: at the trigger's entry, once unless the case says every time."""
        if state != self.case.trigger or (self._fired and not self.case.every):
            return None
        self._fired = True
        return self.case.action


def end_state(states: list[str]) -> str | None:
    """The run's first end state, or None: the IDLE that follows DONE or STOPPED is not part of the run."""
    return next((state for state in states if state.startswith(END_STATES)), None)


def verdict(case: Case, states: list[str]) -> tuple[str, str]:
    """("handled" | "partly" | "not handled", why) for the distinct /brain_state values of one run, in order.

    Not handled: no end state, a forbidden state (the arm moved), or the wrong end. Partly: the right end by
    another path (the fault did not bite, e.g. the moved cube landed where the grasp still caught it).
    The run ends at its first end state (end_state).
    """
    last = end_state(states)
    if last is None:
        return "not handled", f"no end state ({states[-1] if states else 'nothing'} last)"
    states = states[: states.index(last) + 1]
    for state in case.forbidden:
        if state in states:
            return "not handled", f"went through {state}"
    if last != case.expected:
        return "not handled", f"ended in {last}, expected {case.expected}"
    if case.required is not None:
        count = states.count(case.required)
        if count == 0:
            return "partly", f"never went through {case.required}"
        if count < case.required_count:
            return "partly", f"went through {case.required} {count} time(s), expected {case.required_count}"
    return "handled", ""


def lighting_verdict(successes: int, episodes: int) -> tuple[str, str]:
    """Case 5 is judged on whole cycles: all in the box, some, or none."""
    why = f"{successes}/{episodes} cycles ended in DONE"
    if successes == episodes:
        return "handled", why
    return ("partly" if successes else "not handled"), why
