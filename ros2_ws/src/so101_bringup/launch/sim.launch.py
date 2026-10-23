"""sim.launch.py - entry point of the control container: driver + brain, tunables in config/*.yaml.

    ros2 launch so101_bringup sim.launch.py

use_sim and port are deployment choices, not tunables: docker/.env -> control_entrypoint.sh -> these arguments.
expose_ground_truth and perception are for the evaluation bench only (C-06, 08 §3): the E2E jobs set
EXPOSE_GROUND_TRUTH; a bench on its own ROS_DOMAIN_ID adds perception:=true, the compose stack runs perception
in its own container.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare("so101_bringup"), "config"])
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim", default_value="true", description="Simulated backend or real arm"),
            DeclareLaunchArgument("port", default_value="/dev/ttyACM0", description="Serial port of the real arm"),
            DeclareLaunchArgument(
                "expose_ground_truth", default_value="false", description="Sim: /sim/ground_truth/*, bench only"
            ),
            DeclareLaunchArgument("perception", default_value="false", description="Also start so101_perception"),
            Node(
                package="so101_driver",
                executable="driver",
                name="so101_driver",
                output="screen",
                parameters=[
                    PathJoinSubstitution([config, "driver.yaml"]),
                    {
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
                parameters=[PathJoinSubstitution([config, "brain.yaml"])],
            ),
            IncludeLaunchDescription(  # resolved only when included: the CI control job does not build it
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("so101_perception"), "launch", "perception.launch.py"])
                ),
                condition=IfCondition(LaunchConfiguration("perception")),
            ),
        ]
    )
