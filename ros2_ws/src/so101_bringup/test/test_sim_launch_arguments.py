"""Unit test of sim.launch.py's arguments, read from its launch description (no ROS graph, no sim).
The ground truth and the in-container perception are off unless asked: only the evaluation bench turns
them on (C-06, 08 §3)."""

import os
from importlib.util import module_from_spec, spec_from_file_location

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import perform_substitutions

SIM_LAUNCH = os.path.join(get_package_share_directory("so101_bringup"), "launch", "sim.launch.py")


def entities():
    spec = spec_from_file_location("sim_launch", SIM_LAUNCH)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description().entities


def defaults():
    context = LaunchContext()
    return {
        e.name: perform_substitutions(context, e.default_value)
        for e in entities()
        if isinstance(e, DeclareLaunchArgument)
    }


def test_the_ground_truth_and_the_in_container_perception_are_off_by_default():
    assert defaults()["expose_ground_truth"] == "false"
    assert defaults()["perception"] == "false"


def test_perception_is_included_only_when_asked():
    (include,) = [e for e in entities() if isinstance(e, IncludeLaunchDescription)]
    context = LaunchContext()
    for value, included in (("false", False), ("true", True)):
        context.launch_configurations["perception"] = value
        assert include.condition.evaluate(context) is included
