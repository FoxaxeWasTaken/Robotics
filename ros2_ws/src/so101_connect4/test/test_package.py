"""The package (ADR-037): it imports without ROS, and it installs the grid layout both containers read."""

import subprocess
import sys
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory


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
