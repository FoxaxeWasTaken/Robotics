"""The real arm's configuration (#57, 01 §3.3), read from the launch descriptions and the YAML files (no
ROS graph). real.launch.py is sim.launch.py with use_sim:=false, which loads driver_real.yaml and
brain_real.yaml on top of driver.yaml and brain.yaml. The real files change only the keys 01 §3.3 lists, each
a key of the base file: a key only in a real file is a typo or a value missing in sim."""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

SHARE = Path(get_package_share_directory("so101_bringup"))
REAL_KEYS = {  # 01 §3.3: the parameters with a real value
    "driver": {
        "max_joint_velocity",
        "camera_device",
        "camera_width",
        "camera_height",
        "camera_info_file",
        "joint_zero_offsets",
    },
    "brain": {"motion.v_max_rad_s", "grasp.close_pct"},
}


def flat(tree, prefix=""):
    """{"motion": {"v_max_rad_s": 1.0}} -> {"motion.v_max_rad_s": 1.0}, the ROS names."""
    out = {}
    for key, value in tree.items():
        if isinstance(value, dict):
            out.update(flat(value, f"{prefix}{key}."))
        else:
            out[f"{prefix}{key}"] = value
    return out


def parameters(name):
    (node,) = yaml.safe_load((SHARE / "config" / f"{name}.yaml").read_text()).values()
    return flat(node["ros__parameters"])


def entities(launch_file):
    spec = spec_from_file_location(launch_file.replace(".", "_"), SHARE / "launch" / launch_file)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description().entities


def test_the_real_files_change_only_the_documented_keys_of_the_base_files():
    for node, keys in REAL_KEYS.items():
        base, real = parameters(node), parameters(f"{node}_real")
        assert set(real) <= set(base), f"{node}_real.yaml has keys {node}.yaml lacks: {set(real) - set(base)}"
        assert {key for key in real if real[key] != base[key]} == set(real) == keys


def parameter_files(use_sim):
    """The YAML files sim.launch.py gives each node that loads any, in load order (a later file wins)."""
    context = LaunchContext()
    context.launch_configurations.update(
        use_sim=use_sim, port="/dev/ttyACM0", expose_ground_truth="false", calibration="false", lighting=""
    )
    files = {}
    for node in (e for e in entities("sim.launch.py") if isinstance(e, Node)):
        evaluated = evaluate_parameters(context, node._Node__parameters)  # launch_ros has no public getter
        names = [Path(p).name for p in evaluated if not isinstance(p, dict)]
        if names:  # the ghost arm's robot_state_publisher and static TF take none (#200)
            files[node.node_package] = names
    return files


def test_the_sim_loads_the_base_files_and_the_real_arm_its_real_files_on_top():
    assert parameter_files("true") == {
        # the second: driver_calibration.yaml with calibration:=true
        "so101_driver": ["driver.yaml", "driver.yaml"],
        "so101_brain": ["brain.yaml", "brain.yaml"],
    }
    assert parameter_files("false") == {
        "so101_driver": ["driver.yaml", "driver_real.yaml"],
        "so101_brain": ["brain.yaml", "brain_real.yaml"],
    }


def test_the_real_arm_runs_with_the_values_of_its_real_files():
    """#197: driver.yaml came again after driver_real.yaml, and the real arm ran at the sim's 1.5 rad/s."""
    files = parameter_files("false")
    for node, package in (("driver", "so101_driver"), ("brain", "so101_brain")):
        running = {}
        for name in files[package]:  # a later file wins, as in ROS
            running.update(parameters(name.removesuffix(".yaml")))
        real = parameters(f"{node}_real")
        assert {key: running[key] for key in real} == real


def test_real_launch_is_sim_launch_with_use_sim_false():
    (include,) = [e for e in entities("real.launch.py") if isinstance(e, IncludeLaunchDescription)]
    context = LaunchContext()
    context.launch_configurations["port"] = "/dev/ttyACM1"

    def text(value):  # a string, a substitution or a list of them
        return perform_substitutions(context, normalize_to_list_of_substitutions(value))

    arguments = {text(k): text(v) for k, v in include.launch_arguments}
    assert arguments == {"use_sim": "false", "port": "/dev/ttyACM1"}
    include.launch_description_source.get_launch_description(context)
    assert Path(include.launch_description_source.location).name == "sim.launch.py"
