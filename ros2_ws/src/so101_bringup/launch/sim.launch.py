"""sim.launch.py - entry point of the control container: driver + brain, tunables in config/*.yaml.

    ros2 launch so101_bringup sim.launch.py

On the real arm (use_sim:=false, or real.launch.py), driver_real.yaml and brain_real.yaml override a few of
them (01 §3.3).

use_sim and port are deployment choices, not tunables: docker/.env -> control_entrypoint.sh -> these
arguments.
expose_ground_truth and perception are for the evaluation bench only (C-06, 08 §3): the E2E jobs set
EXPOSE_GROUND_TRUTH; a bench on its own ROS_DOMAIN_ID adds perception:=true, the compose stack runs perception
in its own container.

In sim, the driver runs on our scene, generated at launch by so101_bringup.sim_scene, never in sim/ (ADR-036):
the external camera moved to the pose of config/sim_scene.yaml (#47). calibration:=true adds the ArUco markers
of config/markers_sim.yaml and gives 640x480 images (driver_calibration.yaml), for the extrinsic calibration
(3.4, #44). lighting:=dim darkens and warms every light (config/sim_scene.yaml): the robustness demo (#103).
connect4:=true is the game of phase 2 (#79, phase2/P2-02 §7): the grid, its tokens and its markers in the
scene, from so101_connect4's connect4_grid.yaml, and 640x480 images (driver_connect4.yaml).
The ghost arm (#200): a second robot_state_publisher draws the driver's setpoint (/joint_setpoint) as the
ghost/ TF tree, for the "Commanded arm (ghost)" display of RViz, unchecked by default.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import IfElseSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from so101_bringup.sim_scene import SimScene

URDF = Path(get_package_share_directory("so101_description")) / "urdf" / "so101.urdf"


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare("so101_bringup"), "config"])

    def files(name, sim=None):
        """<name>.yaml, then <name>_real.yaml on the real arm; in sim, sim (default: <name>.yaml again,
        no change). One file on top of the base: a third one came last and undid driver_real.yaml (#197)."""
        real = IfElseSubstitution(LaunchConfiguration("use_sim"), sim or f"{name}.yaml", f"{name}_real.yaml")
        return [PathJoinSubstitution([config, f"{name}.yaml"]), PathJoinSubstitution([config, real])]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim", default_value="true", description="Simulated backend or real arm"
            ),
            DeclareLaunchArgument(
                "port", default_value="/dev/ttyACM0", description="Serial port of the real arm"
            ),
            DeclareLaunchArgument(
                "expose_ground_truth",
                default_value="false",
                description="Sim: /sim/ground_truth/*, bench only",
            ),
            DeclareLaunchArgument(
                "perception", default_value="false", description="Also start so101_perception"
            ),
            DeclareLaunchArgument(
                "calibration",
                default_value="false",
                description="Sim: the ArUco calibration scene at 640x480 (3.4)",
            ),
            DeclareLaunchArgument(
                "lighting",
                default_value="",
                description="Sim: a lighting entry of config/sim_scene.yaml (dim), robustness demo (#103)",
            ),
            DeclareLaunchArgument(
                "connect4",
                default_value="false",
                description="Sim: the Connect Four scene of phase 2 at 640x480 (P2-02 §7)",
            ),
            Node(
                package="so101_driver",
                executable="driver",
                name="so101_driver",
                output="screen",
                sigterm_timeout="12",  # SIGKILL 5 s later: after the shutdown move, 11 s at most (#195)
                parameters=[
                    *files(
                        "driver",
                        IfElseSubstitution(
                            LaunchConfiguration("connect4"),
                            "driver_connect4.yaml",
                            IfElseSubstitution(
                                LaunchConfiguration("calibration"), "driver_calibration.yaml", "driver.yaml"
                            ),
                        ),
                    ),
                    {
                        "scene_path": ParameterValue(
                            SimScene(
                                LaunchConfiguration("use_sim"),
                                LaunchConfiguration("calibration"),
                                LaunchConfiguration("lighting"),
                                LaunchConfiguration("connect4"),
                            ),
                            value_type=str,
                        ),
                        "use_sim": ParameterValue(LaunchConfiguration("use_sim"), value_type=bool),
                        "port": LaunchConfiguration("port"),
                        "expose_ground_truth": ParameterValue(
                            LaunchConfiguration("expose_ground_truth"), value_type=bool
                        ),
                    },
                ],
            ),
            Node(
                package="so101_brain",
                executable="brain",
                name="so101_brain",
                output="screen",
                parameters=files("brain"),
            ),
            Node(  # the ghost arm: the setpoint the driver sends, drawn over the measured arm (#200)
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="ghost_state_publisher",
                namespace="ghost",
                parameters=[{"robot_description": URDF.read_text(), "frame_prefix": "ghost/"}],
                remappings=[("joint_states", "/joint_setpoint")],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="ghost_base",
                arguments=["--frame-id", "base_link", "--child-frame-id", "ghost/base_link"],
            ),
            IncludeLaunchDescription(  # resolved only when included: the CI control job does not build it
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("so101_perception"), "launch", "perception.launch.py"]
                    )
                ),
                condition=IfCondition(LaunchConfiguration("perception")),
            ),
        ]
    )
