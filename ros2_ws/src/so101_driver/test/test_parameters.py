"""The driver's startup parameters (01 §3.3, #57): each has a description, each number a range, and a key
the driver does not declare stops it at startup: ROS would ignore it, so a typo in driver_real.yaml would
leave the sim's value on the real arm."""

import pytest
import rclpy
from rcl_interfaces.msg import ParameterType

from so101_driver.driver_node import DriverNode

RANGE = {
    ParameterType.PARAMETER_INTEGER: "integer_range",
    ParameterType.PARAMETER_DOUBLE: "floating_point_range",
}


def test_a_parameter_the_driver_does_not_declare_stops_it_at_startup():
    rclpy.init(args=["--ros-args", "-p", "max_joint_velocty:=0.8"])
    try:
        with pytest.raises(ValueError, match="max_joint_velocty"):
            DriverNode()
    finally:
        rclpy.shutdown()


def test_every_parameter_has_a_description_and_every_number_a_range():
    rclpy.init()
    try:
        node = DriverNode()
        try:
            names = [n for n in node.list_parameters([], 0).names if n != "use_sim_time"]  # declared by rclpy
            descriptors = node.describe_parameters(names)
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()
    assert "bus_retries" in names and "shutdown_velocity" in names and "shutdown_tolerance" in names
    assert [d.name for d in descriptors if not d.description] == []
    assert [d.name for d in descriptors if d.type in RANGE and not getattr(d, RANGE[d.type])] == []
