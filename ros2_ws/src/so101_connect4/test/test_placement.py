"""placement.py (#260, phase2/P2-02 §2.5): would the brain play on the grid and at the dispenser where the
camera sees them? One question per column and per station, asked one at a time while nothing moves, asked
again when one of them has moved. No ROS."""

from pathlib import Path

import numpy as np

from so101_connect4 import board, layout
from so101_connect4.placement import MOVED_M, Placement

# The package's own file, not the installed one: mutation testing runs these tests on the sources (08 §1)
LAYOUT = layout.load(Path(__file__).resolve().parents[1] / "config" / "connect4_grid.yaml")
GRID, DISPENSER = layout.grid_pose(LAYOUT), layout.dispenser_pose(LAYOUT)
CHECKING = "checking where the grid stands"


def moved(T, along=0.0, turned_deg=0.0) -> np.ndarray:
    a, out = np.radians(turned_deg), T.copy()
    out[:3, :3] = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]]) @ T[:3, :3]
    out[:3, 3] += along * T[:3, 0]
    return out


def run(placement, grid=GRID, dispenser=DISPENSER, refuse=None, idle=True, steps=30, reply=None) -> list:
    """Steps until nothing is asked any more, a brain answering each check at the next step; the checks.
    `reply`: the brain's answer to a check asked before."""
    asked = []
    for _ in range(steps):
        check = placement.step(grid, dispenser, reply, idle)
        reply = None
        if check is not None:
            asked.append(check)
            reply = (refuse or {}).get((check[0], check[1]), (True, ""))
    return asked


def test_nothing_is_asked_and_nothing_is_wrong_before_the_camera_has_seen_the_grid():
    placement = Placement(LAYOUT, dispenser=True)
    assert run(placement, grid=None) == []
    assert run(placement, dispenser=None) == []  # with a dispenser, its pose too
    assert placement.reason == CHECKING  # not playable yet; the node says which pose is missing


def test_every_column_is_asked_as_a_drop_at_its_target_then_the_grid_is_playable():
    placement = Placement(LAYOUT, dispenser=False)
    assert placement.reason == CHECKING
    asked = run(placement, dispenser=None)
    assert [(kind, index) for kind, index, _ in asked] == [("drop", column) for column in range(7)]
    for _, column, target in asked:
        assert np.allclose(target, board.column_target(LAYOUT, GRID, column))
    assert placement.reason == ""


def test_with_a_dispenser_every_station_is_asked_as_a_pick_too():
    placement = Placement(LAYOUT, dispenser=True)
    asked = run(placement)
    assert [(kind, index) for kind, index, _ in asked[7:]] == [("pick", rail) for rail in range(3)]
    for _, rail, target in asked[7:]:
        assert np.allclose(target, board.station_target(LAYOUT, DISPENSER, rail))
    assert placement.reason == ""


def test_one_check_at_a_time_the_next_once_the_brain_has_answered():
    placement = Placement(LAYOUT, dispenser=False)
    assert placement.step(GRID, None, None, True)[:2] == ("drop", 0)
    assert placement.step(GRID, None, None, True) is None  # no answer yet
    assert placement.reason == CHECKING
    assert placement.step(GRID, None, (True, ""), True)[:2] == ("drop", 1)


def test_a_refusal_is_the_reason_with_the_column_as_its_key_names_it():
    placement = Placement(LAYOUT, dispenser=True)
    asked = run(placement, refuse={("drop", 6): (False, "unsafe_path:column"), ("pick", 0): (False, "x")})
    assert len(asked) == 10  # every check is asked: the reason is the first refusal
    assert placement.reason == "column 7: unsafe_path:column"
    placement = Placement(LAYOUT, dispenser=True)
    run(placement, refuse={("pick", 1): (False, "not_edge_on")})
    assert placement.reason == "station 2: not_edge_on"


def test_a_pose_that_moved_is_checked_again_and_the_game_is_not_playable_meanwhile():
    placement = Placement(LAYOUT, dispenser=True)
    run(placement)
    for same in (moved(GRID, along=MOVED_M / 2), moved(GRID, turned_deg=1.0)):  # the camera's own noise
        assert run(placement, grid=same) == []
    nearer = moved(GRID, along=-0.02)
    first = placement.step(nearer, DISPENSER, None, True)
    assert first[:2] == ("drop", 0)
    assert np.allclose(first[2], board.column_target(LAYOUT, nearer, 0))
    assert placement.reason == CHECKING
    assert len(run(placement, grid=nearer, reply=(False, "unsafe_path:column"))) == 9
    assert placement.reason == "column 1: unsafe_path:column"
    assert run(placement, grid=nearer) == []  # judged there: not asked again
    assert len(run(placement, grid=moved(GRID, turned_deg=3.0))) == 10
    assert len(run(placement, dispenser=moved(DISPENSER, along=0.01))) == 10
    assert placement.reason == ""


def test_nothing_is_asked_while_the_arm_or_the_engine_works():
    """A check holds the brain's executor for the time of its IK: never during a move."""
    placement = Placement(LAYOUT, dispenser=False)
    assert run(placement, dispenser=None, idle=False) == []
    run(placement, dispenser=None)
    assert run(placement, grid=moved(GRID, along=-0.02), dispenser=None, idle=False) == []
    assert placement.reason == ""  # the verdict of the last check stands until the next one starts
    assert len(run(placement, grid=moved(GRID, along=-0.02), dispenser=None)) == 7


def test_a_grid_that_moves_while_it_is_checked_is_checked_again_from_the_first_column():
    placement = Placement(LAYOUT, dispenser=False)
    for _ in range(3):
        check = placement.step(GRID, None, (True, ""), True)
    assert check[:2] == ("drop", 2)
    nearer = moved(GRID, along=-0.02)
    asked = run(placement, grid=nearer, dispenser=None)
    assert [index for _, index, _ in asked] == list(range(7))
    assert np.allclose(asked[0][2], board.column_target(LAYOUT, nearer, 0))
    assert placement.reason == ""


def test_a_brain_busy_with_a_move_is_asked_the_same_check_again():
    """The brain refuses a check while it moves the arm: its IK would hold the 50 Hz commands. That is not
    a verdict on the grid."""
    placement = Placement(LAYOUT, dispenser=False)
    first = placement.step(GRID, None, None, True)
    assert first[:2] == ("drop", 0)
    again = placement.step(GRID, None, (False, "busy"), True)
    assert again[:2] == ("drop", 0)
    assert np.allclose(again[2], first[2])
    assert placement.reason == CHECKING
    asked = [again, *run(placement, dispenser=None, reply=(True, ""))]
    assert [index for _, index, _ in asked] == list(range(7))
    assert placement.reason == ""


def test_a_grid_turned_on_itself_is_judged_again_from_2_degrees_either_way():
    placement = Placement(LAYOUT, dispenser=False)
    run(placement, dispenser=None)
    for degrees in (1.5, -1.5):
        assert run(placement, grid=moved(GRID, turned_deg=degrees), dispenser=None) == []
    assert len(run(placement, grid=moved(GRID, turned_deg=2.5), dispenser=None)) == 7
    assert len(run(placement, grid=GRID, dispenser=None)) == 7  # 2.5 degrees from where it was judged last
    assert len(run(placement, grid=moved(GRID, turned_deg=-2.5), dispenser=None)) == 7


def test_without_a_dispenser_its_pose_is_not_looked_at():
    placement = Placement(LAYOUT, dispenser=False)
    assert len(run(placement)) == 7  # the scene has a dispenser, the camera sees it: the game does not use it
    assert run(placement, dispenser=moved(DISPENSER, along=0.05)) == []
    assert placement.reason == ""
