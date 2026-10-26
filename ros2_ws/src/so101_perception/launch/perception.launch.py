"""perception.launch.py - entry point of the perception container (docs/project/02 §3.1).

    ros2 launch so101_perception perception.launch.py

Tunables in config/perception.yaml. The perception container builds only up to this package, so the file
lives here and not in so101_bringup. extrinsics_file: the camera pose (config/camera_extrinsics.yaml by
default); the launch test gives the one of its reference images.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "extrinsics_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("so101_perception"), "config", "camera_extrinsics.yaml"]
                ),
                description="Camera pose in base_link (so101_perception.geometry.save_extrinsics)",
            ),
            Node(
                package="so101_perception",
                executable="perception",
                name="so101_perception",
                output="screen",
                parameters=[
                    PathJoinSubstitution([FindPackageShare("so101_perception"), "config", "perception.yaml"]),
                    {"extrinsics_file": LaunchConfiguration("extrinsics_file")},
                ],
            ),
        ]
    )
