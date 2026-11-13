"""The RViz files (#265): the game's view is the project's with the board shown and the pick-and-place
displays off; nothing else differs, so a change to one is a change to both."""

from pathlib import Path

import yaml

RVIZ = Path(__file__).resolve().parents[1] / "rviz"
GAME_ONLY = "Connect Four board (game mode)"
PICK_AND_PLACE = ["Cube (perception)", "Box floor (perception)", "Drop box", "Brain targets", "Cube (locked)"]


def displays(name) -> dict:
    """{display name: its settings} of an RViz file."""
    config = yaml.safe_load((RVIZ / name).read_text())
    return {display["Name"]: display for display in config["Visualization Manager"]["Displays"]}


def test_the_games_view_shows_the_board_and_hides_the_pick_and_place():
    game, project = displays("connect4.rviz"), displays("so101.rviz")
    assert list(game) == list(project)
    assert game[GAME_ONLY]["Enabled"] is True
    assert project[GAME_ONLY]["Enabled"] is False
    for name in PICK_AND_PLACE:
        assert game[name]["Enabled"] is False, name
        assert project[name]["Enabled"] is True, name
    for name in set(project) - {GAME_ONLY, *PICK_AND_PLACE}:
        assert game[name] == project[name], name
