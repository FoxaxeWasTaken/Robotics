"""perception.launch.py - entry point of the perception container (docs/project/02 §3.1).

    ros2 launch so101_perception perception.launch.py

Tunables in config/perception.yaml. The perception container builds only up to this package, so the file
lives here and not in so101_bringup.
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="so101_perception",
                executable="perception",
                name="so101_perception",
                output="screen",
                parameters=[PathJoinSubstitution([FindPackageShare("so101_perception"), "config", "perception.yaml"])],
            ),
        ]
    )
