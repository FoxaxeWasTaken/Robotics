"""game.launch.py - the game's nodes of the control container; sim.launch.py includes it with connect4:=true
(#231).

The game node (phase2/P2-02 §5, #86), next to the driver and the brain it calls; tunables in config/game.yaml.
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare("so101_connect4"), "config", "game.yaml"])
    return LaunchDescription(
        [
            Node(
                package="so101_connect4",
                executable="game_node",
                name="game_node",
                output="screen",
                parameters=[config],
            )
        ]
    )
