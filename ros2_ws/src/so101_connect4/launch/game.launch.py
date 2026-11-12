"""game.launch.py - the game's nodes of the control container; sim.launch.py includes it with connect4:=true
(#231).

The game node (phase2/P2-02 §5) joins it with #86: until then it starts nothing.
"""

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([])
