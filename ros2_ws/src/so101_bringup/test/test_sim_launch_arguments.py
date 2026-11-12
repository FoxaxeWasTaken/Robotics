"""Unit test of sim.launch.py's arguments, read from its launch description (no ROS graph, no sim).
The ground truth and the in-container perception are off unless asked: only the evaluation bench turns
them on (C-06, 08 §3)."""

import os
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import mujoco
import numpy as np
import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
from so101_sim import DEFAULT_SCENE

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


def node_parameters(package, **configurations):
    """The parameter files (names, in load order) and the values sim.launch.py gives that package's node."""
    context = LaunchContext()
    context.launch_configurations.update(defaults(), **configurations)
    (node,) = [e for e in entities() if isinstance(e, Node) and e.node_package == package]
    evaluated = evaluate_parameters(context, node._Node__parameters)  # launch_ros has no public getter
    values = {k: v for p in evaluated if isinstance(p, dict) for k, v in p.items()}
    return [Path(p).name for p in evaluated if not isinstance(p, dict)], values


def driver_parameters(**configurations):
    return node_parameters("so101_driver", **configurations)


def test_the_ground_truth_and_the_in_container_perception_are_off_by_default():
    assert defaults()["expose_ground_truth"] == "false"
    assert defaults()["perception"] == "false"
    assert defaults()["calibration"] == "false"
    assert defaults()["lighting"] == ""  # the robustness demo's lighting change (#103): none by default


def test_calibration_starts_the_driver_on_the_markers_scene_at_640x480():
    """#44: the scene with the ArUco markers of markers_sim.yaml, the image size of ADR-013."""
    files, values = driver_parameters(calibration="true")
    assert files[-1] == "driver_calibration.yaml"
    config = Path(get_package_share_directory("so101_bringup")) / "config" / "driver_calibration.yaml"
    size = yaml.safe_load(config.read_text())["so101_driver"]["ros__parameters"]
    assert (size["camera_width"], size["camera_height"]) == (640, 480)
    scene = Path(values["scene_path"])
    assert scene.is_file()
    assert "aruco_0" in scene.read_text()


def test_without_calibration_the_driver_keeps_its_image_size_and_gets_the_moved_camera_only():
    """#47: the sim always runs on our scene, its camera moved (sim_scene.yaml); no markers."""
    files, values = driver_parameters()
    assert files[-1] == "driver.yaml"
    scene = Path(values["scene_path"]).read_text()
    assert "aruco_0" not in scene
    assert 'name="external_cam"' in scene


def test_the_real_arm_gets_no_generated_scene():
    _, values = driver_parameters(use_sim="false", calibration="true")
    assert values["scene_path"] == ""
    _, values = driver_parameters(use_sim="false", connect4="true")
    assert values["scene_path"] == ""


def test_connect4_is_off_by_default_and_the_scene_has_no_grid():
    """#79: phase 1 does not move. The game is asked for, never a default."""
    assert defaults()["connect4"] == "false"
    _, values = driver_parameters()
    scene = Path(values["scene_path"]).read_text()
    assert 'name="grid"' not in scene and "token_" not in scene
    assert values["connect4_layout"] == ""


def test_connect4_starts_the_driver_on_the_game_scene_at_640x480():
    """#79: the grid, its 42 tokens and its markers (P2-02 §7), the image size the board reader needs."""
    files, values = driver_parameters(connect4="true")
    assert files[-1] == "driver_connect4.yaml"
    config = Path(get_package_share_directory("so101_bringup")) / "config" / "driver_connect4.yaml"
    size = yaml.safe_load(config.read_text())["so101_driver"]["ros__parameters"]
    assert (size["camera_width"], size["camera_height"]) == (640, 480)
    model = mujoco.MjModel.from_xml_path(values["scene_path"])
    assert model.body("grid").id > 0
    assert Path(values["connect4_layout"]).name == "connect4_grid.yaml"  # the driver's game mode (#80)
    assert sum(model.body(i).name.startswith("token_") for i in range(model.nbody)) == 42


def test_connect4_with_calibration_stops_the_launch():
    """The calibration markers lie on the table, which the game lowers by the riser."""
    with pytest.raises(ValueError, match="one at a time"):
        driver_parameters(connect4="true", calibration="true")


def included(package, **configurations):
    """Whether sim.launch.py includes that package's launch file, with these arguments over the defaults."""
    context = LaunchContext()
    context.launch_configurations.update(defaults(), **configurations)
    (include,) = [
        e
        for e in entities()
        if isinstance(e, IncludeLaunchDescription)
        and package in e.launch_description_source.location  # its description, before it is resolved
    ]
    return include.condition.evaluate(context)


def test_perception_is_included_only_when_asked():
    assert included("so101_perception") is False
    assert included("so101_perception", perception="true") is True


def test_the_game_is_included_only_with_connect4():
    """#231: so101_connect4's game.launch.py, the game node's place; phase 1 starts the nodes it did."""
    assert included("so101_connect4") is False
    assert included("so101_connect4", connect4="true") is True
    nodes = sorted(e.node_package for e in entities() if isinstance(e, Node))
    assert nodes == ["robot_state_publisher", "so101_brain", "so101_driver", "tf2_ros"]


def test_the_brain_serves_the_games_moves_only_with_connect4():
    """#84, #85: /brain/drop and /brain/handover exist in game mode only; its parameter files stay."""
    files, values = node_parameters("so101_brain")
    assert values["connect4.enabled"] is False
    assert files == ["brain.yaml", "brain.yaml"]
    files, values = node_parameters("so101_brain", connect4="true")
    assert values["connect4.enabled"] is True
    assert files == ["brain.yaml", "brain.yaml"]


def test_lighting_dim_gives_the_driver_the_dimmed_scene():
    """#103, case 5: every light of the scene darker and tinted as sim_scene.yaml's dim says."""
    _, values = driver_parameters(lighting="dim")
    config = Path(get_package_share_directory("so101_bringup")) / "config" / "sim_scene.yaml"
    dim = yaml.safe_load(config.read_text())["lighting"]["dim"]
    provided = mujoco.MjModel.from_xml_path(str(DEFAULT_SCENE))
    dimmed = mujoco.MjModel.from_xml_path(values["scene_path"])
    factor = dim["scale"] * np.array(dim["tint"])
    assert np.allclose(dimmed.light_diffuse, provided.light_diffuse * factor, atol=1e-6)


def test_an_unknown_lighting_stops_the_launch():
    with pytest.raises(ValueError, match="lighting 'disco'"):
        driver_parameters(lighting="disco")
