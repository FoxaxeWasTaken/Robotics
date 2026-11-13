"""The game's robustness cases as they are run (#243): robustness_game.run on a scripted demo. Each step's
action, then the state or the reply that must follow, the arm's motion meanwhile; the run stops at the first
step that is not reached. No ROS, no sim: the cases on the stack are results/robustness_demo.md's."""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import pytest
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

from so101_bringup import robustness_game
from so101_bringup.robustness import GAME_CASES, GameCase, Step, game_verdict


class ScriptedDemo:
    """robustness_game.Demo as run() uses it: `after` gives, per action, the state that follows and how far
    the arm goes meanwhile; an action without an entry changes nothing."""

    def __init__(self, after, replies=None):
        self.after, self.replies, self.actions = after, replies or {}, []
        self.state, self._moved = {"state": "WAIT_HUMAN", "turn": "human", "moves": 0, "reason": ""}, 0.0

    def act(self, action):
        self.actions.append(action)
        state, moved = self.after.get(action, (None, 0.0))
        self._moved += moved  # since the last rest(), as the real one
        if state is not None:
            self.state = state
        return self.replies.get(action, {"success": True, "message": ""})

    def rest(self):
        self._moved = 0.0

    def moved(self):
        return self._moved

    def wait(self, condition, timeout_s):
        return condition(self.state)


def state(name, **fields):
    return {"state": name, "reason": "", "turn": "human", "moves": 0} | fields


def test_g2_is_run_step_by_step_and_handled():
    after = {
        "turn": (state("WAIT_HUMAN", reason="dispenser_empty", turn="robot", lost=0, moves=1), 0.0),
        "refill": (state("WAIT_HUMAN", moves=2), 0.9),  # the robot's move: the arm may move
    }
    demo = ScriptedDemo(after)
    results = robustness_game.run(GAME_CASES["g2"], demo)
    assert demo.actions == ["empty_dispenser", "turn", "refill"]
    assert [r["reached"] for r in results] == [True, True, True]
    assert game_verdict(GAME_CASES["g2"], results) == ("handled", "")


def test_a_state_that_does_not_follow_stops_the_run_and_is_reported():
    demo = ScriptedDemo({"cheat": (state("THINK", turn="robot"), 0.0)})
    results = robustness_game.run(GAME_CASES["g1"], demo)
    assert demo.actions == ["cheat"]  # the new game is not asked for
    assert results == [{"reached": False, "got": "THINK", "moved": 0.0}]


def test_the_state_before_the_action_does_not_count():
    """g1's second step waits for WAIT_HUMAN, the state the case starts in: only a new one counts."""
    demo = ScriptedDemo({"cheat": (state("FIX_BOARD", reason="counts:2-0"), 0.0)})
    results = robustness_game.run(GAME_CASES["g1"], demo)
    assert [r["reached"] for r in results] == [True, False]
    assert results[1]["got"] == "FIX_BOARD:counts:2-0"


def test_a_state_that_already_held_before_the_action_is_not_its_effect():
    waits = {"state": "WAIT_HUMAN", "turn": "human"}
    case = GameCase(question="?", steps=(Step("new_game", state=waits),))
    demo = ScriptedDemo({})  # the action changes nothing: the game still waits, as before it
    assert robustness_game.run(case, demo) == [{"reached": False, "got": "WAIT_HUMAN", "moved": 0.0}]


def test_a_reply_is_judged_as_it_comes_and_the_motion_is_measured_from_the_steps_start():
    replies = {"drop_out_of_reach": {"success": False, "message": "unreachable"}}
    replies["drop_token"] = {"success": False, "message": "column 0 is full"}
    demo = ScriptedDemo({"drop_out_of_reach": (None, 0.2)}, replies)
    results = robustness_game.run(GAME_CASES["g4"], demo)
    assert [r["reached"] for r in results] == [True, True, True]
    assert [r["moved"] for r in results] == [0.2, 0.0, 0.0]
    assert game_verdict(GAME_CASES["g4"], results)[0] == "not handled"  # the arm moved on a refused drop


@pytest.mark.parametrize(("case", "game"), [("g3", "True"), ("3a", "False"), ("5", "False"), ("g5", "True")])
def test_the_games_cases_start_the_demos_stack_in_game_mode(case, game):
    """robustness_demo.launch.py: connect4 for g1 to g6, with perception, which is then the board reader."""
    path = Path(get_package_share_directory("so101_bringup")) / "launch" / "robustness_demo.launch.py"
    spec = spec_from_file_location("robustness_demo_launch", path)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    entities = module.generate_launch_description().entities
    (include,) = [e for e in entities if isinstance(e, IncludeLaunchDescription)]
    context = LaunchContext()
    context.launch_configurations.update(case=case, lighting="")

    def text(value):
        return perform_substitutions(context, normalize_to_list_of_substitutions(value))

    arguments = {text(key): text(value) for key, value in include.launch_arguments}
    assert (arguments["connect4"], arguments["perception"]) == (game, "true")
