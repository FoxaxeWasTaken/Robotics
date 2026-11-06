"""perception.launch.py - entry point of the perception container (docs/project/02 §3.1).

    ros2 launch so101_perception perception.launch.py

Tunables in config/perception.yaml. The perception container builds only up to this package, so the file
lives here and not in so101_bringup. extrinsics_file: the camera pose (config/camera_extrinsics.yaml by
default); the launch test gives the one of its reference images. On the real arm (USE_SIM=false, compose),
perception_real.yaml overrides a few of those values and the pose is config/camera_extrinsics_real.yaml.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    EnvironmentVariable,
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = FindPackageShare("so101_perception"), "config"
    use_sim = LaunchConfiguration("use_sim")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim",
                default_value=EnvironmentVariable("USE_SIM", default_value="true"),
                description="Simulated camera or the real one",
            ),
            DeclareLaunchArgument(
                "extrinsics_file",
                default_value=PathJoinSubstitution(
                    [
                        *config,
                        IfElseSubstitution(use_sim, "camera_extrinsics.yaml", "camera_extrinsics_real.yaml"),
                    ]
                ),
                description="Camera pose in base_link (so101_perception.geometry.save_extrinsics)",
            ),
            Node(
                package="so101_perception",
                executable="perception",
                name="so101_perception",
                output="screen",
                parameters=[
                    PathJoinSubstitution([*config, "perception.yaml"]),
                    PathJoinSubstitution(
                        [*config, IfElseSubstitution(use_sim, "perception.yaml", "perception_real.yaml")]
                    ),
                    {"extrinsics_file": LaunchConfiguration("extrinsics_file")},
                ],
            ),
        ]
    )
