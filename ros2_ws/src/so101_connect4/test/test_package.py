"""The package (ADR-037): it imports without ROS, and it installs the grid layout both containers read."""

import subprocess
import sys
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from so101_connect4 import turns


def test_the_package_imports_without_ros():
    code = "import sys, so101_connect4; print('rclpy' in sys.modules)"
    out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, check=True).stdout
    assert out.strip() == "False"


def test_the_grid_layout_is_installed_and_puts_the_columns_where_p2_02_reaches():
    path = Path(get_package_share_directory("so101_connect4")) / "config" / "connect4_grid.yaml"
    layout = yaml.safe_load(path.read_text())
    grid, placement = layout["grid"], layout["placement"]
    assert (grid["columns"], grid["rows"]) == (7, 6)
    last = placement["first_column_r"] + (grid["columns"] - 1) * grid["column_pitch"]
    assert 0.36 <= last <= 0.365  # columns at r = 0.14 ... 0.36 m (P2-02 §1)
    assert layout["token"]["thickness"] < grid["slot_gap"]


def test_the_two_launch_files_of_the_game_mode_are_installed():
    """#231: board_reader.launch.py for the perception container, game.launch.py for the control one."""
    launch = Path(get_package_share_directory("so101_connect4")) / "launch"
    for name in ("board_reader.launch.py", "game.launch.py"):
        spec = spec_from_file_location(name.split(".")[0], launch / name)  # its real path: coverage finds it
        module = module_from_spec(spec)
        spec.loader.exec_module(module)
        assert module.generate_launch_description() is not None


def test_the_game_launch_starts_the_game_node_with_its_installed_parameters():
    """#86: game.launch.py is the control container's part of the game; game.yaml only names turns.Params."""
    share = Path(get_package_share_directory("so101_connect4"))
    spec = spec_from_file_location("game", share / "launch" / "game.launch.py")
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    (node,) = [e for e in module.generate_launch_description().entities if isinstance(e, Node)]
    assert (node.node_package, node.node_executable) == ("so101_connect4", "game_node")
    parameters = yaml.safe_load((share / "config" / "game.yaml").read_text())["game_node"]["ros__parameters"]
    assert parameters == {name: getattr(turns.Params(), name) for name in parameters}
    assert set(parameters) == set(turns.Params.__dataclass_fields__)
