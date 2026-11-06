"""Perception's parameters (05, #57): each has a description, each number a range, the median filter's window
and jump are parameters, and a key perception does not declare stops it at startup (ROS would ignore it)."""

import pytest
import rclpy
from rcl_interfaces.msg import ParameterType

from so101_perception.perception_node import PerceptionNode

RANGE = {
    ParameterType.PARAMETER_INTEGER: "integer_range",
    ParameterType.PARAMETER_DOUBLE: "floating_point_range",
}


def started(*overrides):
    """A PerceptionNode started with these `name:=value` overrides: its cube filter and descriptors."""
    rclpy.init(args=["--ros-args", *(a for o in overrides for a in ("-p", o))])
    try:
        node = PerceptionNode()
        try:
            names = [n for n in node.list_parameters([], 0).names if n != "use_sim_time"]  # declared by rclpy
            return node._filters["cube"], node.describe_parameters(names)
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def test_a_parameter_perception_does_not_declare_stops_it_at_startup():
    with pytest.raises(ValueError, match="hsv_cube_hmin"):
        started("hsv_cube_hmin:=150")


def test_the_object_sizes_are_parameters():
    rclpy.init(args=["--ros-args", "-p", "cube_side_m:=0.03", "-p", "box_height_m:=0.08"])
    try:
        node = PerceptionNode()
        try:
            assert (node._cube_side, node._box_height) == (0.03, 0.08)
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def test_every_parameter_has_a_description_and_every_number_a_range():
    _, descriptors = started()
    assert [d.name for d in descriptors if not d.description] == []
    assert [d.name for d in descriptors if d.type in RANGE and not getattr(d, RANGE[d.type])] == []


def test_the_median_filter_takes_its_window_and_jump_from_the_parameters():
    cube, _ = started("filter_size:=3", "filter_jump_m:=0.01")
    assert cube.update([0.20, 0.0, 0.025]) is not None
    # 1.5 cm away: a jump with 0.01, kept with the default 0.02
    assert cube.update([0.215, 0.0, 0.025]) is None
    assert cube.update([0.215, 0.0, 0.025]) is None
    assert cube.update([0.215, 0.0, 0.025]) is not None  # 3 in a row: the window of 3 moves
