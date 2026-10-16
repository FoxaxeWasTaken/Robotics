"""sim.launch.py - entry point of the control container: driver + brain, tunables in config/*.yaml.

    ros2 launch so101_bringup sim.launch.py

use_sim and port are deployment choices, not tunables: docker/.env -> control_entrypoint.sh -> these arguments.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
        ]
    )
