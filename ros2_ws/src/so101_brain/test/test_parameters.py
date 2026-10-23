"""The brain's startup parameters (06 §6, #57): every value of fsm.Params is one, each has a description
and each number a range, and a key the brain does not declare stops it at startup: ROS would ignore it, so a
typo in brain_real.yaml would leave the sim's speed on the real arm."""

import dataclasses

import pytest
import rclpy
from rcl_interfaces.msg import ParameterType

from so101_brain.brain_node import PARAMETERS, BrainNode
from so101_brain.fsm import Params

RANGE = {
    ParameterType.PARAMETER_INTEGER: "integer_range",
    ParameterType.PARAMETER_DOUBLE: "floating_point_range",
}


def started(*overrides):
    """A BrainNode started with these `name:=value` overrides: its FSM values and descriptors."""
    rclpy.init(args=["--ros-args", *(a for o in overrides for a in ("-p", o))])
    try:
        node = BrainNode()
        try:
            names = [n for n in node.list_parameters([], 0).names if n != "use_sim_time"]  # declared by rclpy
            return node._fsm.p, node.describe_parameters(names)
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def test_a_parameter_the_brain_does_not_declare_stops_it_at_startup():
    with pytest.raises(ValueError, match=r"motion\.v_max_rads"):
        started("motion.v_max_rads:=0.5")


def test_every_parameter_has_a_description_and_every_number_a_range():
    _, descriptors = started()
    assert [d.name for d in descriptors if not d.description] == []
    assert [d.name for d in descriptors if d.type in RANGE and not getattr(d, RANGE[d.type])] == []


def test_every_value_of_the_fsm_is_a_parameter_that_reaches_it():
    assert {field for field, *_ in PARAMETERS.values()} | {"park_pose"} == {
        f.name for f in dataclasses.fields(Params)
    }
    default = Params()
    values = {  # inside the range, away from the default
        name: high if isinstance(getattr(default, field), int) else (low + high) / 2
        for name, (field, _, low, high) in PARAMETERS.items()
    }
    params, _ = started(*(f"{name}:={value}" for name, value in values.items()))
    assert {name: getattr(params, PARAMETERS[name][0]) for name in values} == values
