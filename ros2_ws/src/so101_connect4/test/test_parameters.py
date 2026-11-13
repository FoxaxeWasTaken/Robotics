"""The parameters of the game's two nodes, as phase 1's (so101_perception's test_parameters.py): each has a
description, each number a range, a value out of its range or a key the node does not declare stops it at
startup (ROS would ignore the key: a typo in a config file would leave the default)."""

import pytest
import rclpy
from rcl_interfaces.msg import ParameterType
from rclpy.exceptions import InvalidParameterValueException

from so101_connect4.board_reader_node import BoardReaderNode
from so101_connect4.game_node import GameNode

RANGE = {
    ParameterType.PARAMETER_INTEGER: "integer_range",
    ParameterType.PARAMETER_DOUBLE: "floating_point_range",
}


def started(make, *overrides):
    """The descriptors of a node started with these `name:=value` overrides."""
    rclpy.init(args=["--ros-args", *(a for o in overrides for a in ("-p", o))])
    try:
        node = make()
        try:
            names = [n for n in node.list_parameters([], 0).names if n != "use_sim_time"]  # declared by rclpy
            return node.describe_parameters(names)
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


@pytest.mark.parametrize("make", [GameNode, BoardReaderNode])
def test_every_parameter_has_a_description_and_every_number_a_range(make):
    descriptors = started(make)
    assert [d.name for d in descriptors if not d.description] == []
    assert [d.name for d in descriptors if d.type in RANGE and not getattr(d, RANGE[d.type])] == []
    assert len({d.description for d in descriptors}) == len(descriptors), "one description for two parameters"


@pytest.mark.parametrize(
    ("make", "override"),
    [
        (BoardReaderNode, "process_rate_hz:=0.0"),  # a division by zero at the timer
        (BoardReaderNode, "stable_reads:=0"),  # every read would be stable
        (BoardReaderNode, "token_threshold:=7.0"),  # no token would ever be read
        (GameNode, "read_timeout_s:=-1.0"),
        (GameNode, "engine.budget_s:=0.0"),
        (GameNode, "engine.max_depth:=0"),
    ],
)
def test_a_value_out_of_its_range_stops_the_node_at_startup(make, override):
    with pytest.raises(InvalidParameterValueException):
        started(make, override)


@pytest.mark.parametrize(("make", "key"), [(BoardReaderNode, "stable_read"), (GameNode, "engine.depth")])
def test_a_parameter_the_node_does_not_declare_stops_it_at_startup(make, key):
    with pytest.raises(ValueError, match=key):
        started(make, f"{key}:=5")


def test_the_grid_layout_has_one_source_the_installed_file():
    """The reader and the game node both use connect4_grid.yaml: no parameter gives one of them another."""
    with pytest.raises(ValueError, match="layout_file"):
        started(BoardReaderNode, "layout_file:=/tmp/other.yaml")
