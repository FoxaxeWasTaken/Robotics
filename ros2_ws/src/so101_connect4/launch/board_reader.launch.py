"""board_reader.launch.py - entry point of the perception container in game mode (CONNECT4=true, #231).

    ros2 launch so101_connect4 board_reader.launch.py

The board reader (phase2/P2-02 §3) joins it with #82: until then the container builds the package, starts
nothing and stops.
"""

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([])
