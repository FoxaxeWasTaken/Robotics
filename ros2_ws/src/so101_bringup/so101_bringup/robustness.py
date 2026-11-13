"""Robustness demo (step 5.4a, docs/project/06 §10): the cases of [BOOT2 p12], when to inject each fault, and
the verdict on the /brain_state sequence a run printed. Pure: no ROS, no sim (robustness_demo.py runs them).

The faults go through the sim-only services or the demo's own stack (stopping its perception node), never
through the ground truth (C-06): the pipeline still sees the scene through the camera only.
"""

from dataclasses import dataclass

END_STATES = ("DONE", "STOPPED", "ERROR")  # a cycle ends in one of them (ERROR with its cause)
NOT_HANDLED = "not handled"
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
        return NOT_HANDLED, f"no end state ({states[-1] if states else 'nothing'} last)"
    states = states[: states.index(last) + 1]
    for state in case.forbidden:
        if state in states:
            return NOT_HANDLED, f"went through {state}"
    if last != case.expected:
        return NOT_HANDLED, f"ended in {last}, expected {case.expected}"
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
    return ("partly" if successes else NOT_HANDLED), why


# The game of phase 2 (#243): the same questions asked of a Connect Four game. A case is a list of steps on a
# game that waits for the person's first move; robustness_game.py runs them.

STILL_RAD = 0.05  # an arm joint further than this from its place at a step's start: the arm moved (P2-05)
WAITS_PERSON = {"state": "WAIT_HUMAN", "turn": "human"}
NO_TOKEN = {"state": "WAIT_HUMAN", "reason": "no_token", "lost": 1}


@dataclass(frozen=True)
class Step:
    """One action, then what must follow: fields of a new /connect4/state, or of the action's reply."""

    action: str  # one of robustness_game.Demo's
    state: dict | None = None  # of a /connect4/state published after the action
    reply: dict | None = None  # of the action's own reply: success, message
    still: bool = False  # no arm joint may move from the action to what must follow


@dataclass(frozen=True)
class GameCase:
    question: str
    steps: tuple


GAME_CASES = {
    "g1": GameCase(
        question="The scene changes mid-action: the person drops two tokens in one turn.",
        steps=(
            Step("cheat", state={"state": "FIX_BOARD"}, still=True),
            Step("new_game", state=WAITS_PERSON | {"moves": 0}),
        ),
    ),
    "g2": GameCase(
        question="The grasp fails: no token is given before the robot's move.",
        steps=(
            Step("turn_without_token", state=NO_TOKEN, still=True),
            Step("give_token", state=WAITS_PERSON | {"moves": 2}),
        ),
    ),
    "g3": GameCase(
        question="The object leaves the view: the board reader stops.",
        steps=(
            Step("stop_reader", reply={"success": True}),
            Step("turn", state={"state": "FIX_BOARD", "reason": "not_seen:human"}, still=True),
            Step("start_reader", reply={"success": True}),
            Step("human_done", state=WAITS_PERSON | {"moves": 2}),
        ),
    ),
    "g4": GameCase(
        question="Target out of reach: a drop too far, a token for a full column.",
        steps=(
            Step("drop_out_of_reach", reply={"success": False, "message": "unreachable"}, still=True),
            Step("fill_column", reply={"success": True}),
            Step("drop_token", reply={"success": False}, still=True),
        ),
    ),
    "g5": GameCase(
        question="Lighting changes: is the grid still read?",
        steps=(Step("game", state={"state": "GAME_OVER", "lost": 0, "missed": 0}),),  # with lighting:=dim
    ),
    "g6": GameCase(
        question="A stop in the middle of a move (ours).",
        steps=(
            Step("turn", state={"state": "MOVE"}),
            Step("wait_5_s", reply={"success": True}),
            Step("stop", state={"state": "ERROR", "reason": "stopped"}),
            Step("wait_2_s", reply={"success": True}, still=True),  # the arm frozen above the column
            Step("new_game", state=WAITS_PERSON | {"moves": 0}),
        ),
    ),
}


def matches(expected: dict, got: dict) -> bool:
    return all(got.get(key) == value for key, value in expected.items())


def game_verdict(case: GameCase, results: list) -> tuple[str, str]:
    """("handled" | "not handled", why) from the result of each step run, in order: `reached`, `got` (what
    came instead) and `moved`, the furthest an arm joint went meanwhile, rad. The runner stops at the first
    step not reached."""
    for number, step in enumerate(case.steps, start=1):
        if number > len(results) or not results[number - 1]["reached"]:
            got = results[number - 1]["got"] if number <= len(results) else "not run"
            return (
                NOT_HANDLED,
                f"step {number}, {step.action}: expected {step.state or step.reply}, got {got}",
            )
        if step.still and results[number - 1]["moved"] > STILL_RAD:
            moved = results[number - 1]["moved"]
            return NOT_HANDLED, f"step {number}, {step.action}: the arm moved, {moved:.2f} rad"
    return "handled", ""
