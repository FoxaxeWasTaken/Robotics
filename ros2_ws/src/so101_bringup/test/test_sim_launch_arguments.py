"""Unit test of sim.launch.py's arguments, read from its launch description (no ROS graph, no sim).
The ground truth and the in-container perception are off unless asked: only the evaluation bench turns
them on (C-06, 08 §3)."""

import os
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

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


def driver_parameters(**configurations):
    """The parameter files (names, in load order) and the values sim.launch.py gives the driver."""
    context = LaunchContext()
    context.launch_configurations.update(defaults(), **configurations)
    (driver,) = [e for e in entities() if isinstance(e, Node) and e.node_package == "so101_driver"]
    evaluated = evaluate_parameters(context, driver._Node__parameters)  # launch_ros has no public getter
    values = {k: v for p in evaluated if isinstance(p, dict) for k, v in p.items()}
    return [Path(p).name for p in evaluated if not isinstance(p, dict)], values


def test_the_ground_truth_and_the_in_container_perception_are_off_by_default():
    assert defaults()["expose_ground_truth"] == "false"
    assert defaults()["perception"] == "false"
    assert defaults()["calibration"] == "false"


def test_calibration_starts_the_driver_on_the_markers_scene_at_640x480():
    """#44: the scene with the ArUco markers of markers_sim.yaml, the image size of ADR-013."""
    files, values = driver_parameters(calibration="true")
    assert files[-1] == "driver_calibration.yaml"
    config = Path(get_package_share_directory("so101_bringup")) / "config" / "driver_calibration.yaml"
    size = yaml.safe_load(config.read_text())["so101_driver"]["ros__parameters"]
    assert (size["camera_width"], size["camera_height"]) == (640, 480)
    scene = Path(values["scene_path"])
    assert scene.is_file() and "aruco_0" in scene.read_text()


def test_without_calibration_the_driver_keeps_its_image_size_and_the_provided_scene():
    files, values = driver_parameters()
    assert files[-1] == "driver.yaml"
    assert values["scene_path"] == ""


def test_the_real_arm_gets_no_generated_scene():
    _, values = driver_parameters(use_sim="false", calibration="true")
    assert values["scene_path"] == ""


def test_perception_is_included_only_when_asked():
    (include,) = [e for e in entities() if isinstance(e, IncludeLaunchDescription)]
    context = LaunchContext()
    for value, included in (("false", False), ("true", True)):
        context.launch_configurations["perception"] = value
        assert include.condition.evaluate(context) is included
