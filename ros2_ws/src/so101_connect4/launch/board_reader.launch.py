"""board_reader.launch.py - entry point of the perception container in game mode (CONNECT4=true, #231).

    ros2 launch so101_connect4 board_reader.launch.py

The board reader (phase2/P2-02 §3, #82) in place of the cube's perception; tunables in
config/board_reader.yaml.
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare("so101_connect4"), "config", "board_reader.yaml"])
    return LaunchDescription(
        [
            Node(
                package="so101_connect4",
                executable="board_reader",
                name="board_reader",
                output="screen",
                parameters=[config],
            )
        ]
    )
