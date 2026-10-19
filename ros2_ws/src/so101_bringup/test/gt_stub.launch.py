"""gt_stub.launch.py - DEVELOPMENT ONLY (step 4.2c): the sim stack with the ground-truth perception stub.

    ros2 launch /ros2_ws/src/so101_bringup/test/gt_stub.launch.py

The driver (sim, expose_ground_truth) and the brain as in sim.launch.py, and gt_perception_stub.py in place of
perception. Never referenced by sim.launch.py or real.launch.py (C-06 guard): it lives under test/.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare("so101_bringup"), "config"])
    return LaunchDescription(
        [
            Node(
                package="so101_driver",
                executable="driver",
                name="so101_driver",
                output="screen",
                parameters=[
                    PathJoinSubstitution([config, "driver.yaml"]),
                    {"use_sim": True, "expose_ground_truth": True},
                ],
            ),
            Node(
                package="so101_brain",
                executable="brain",
                name="so101_brain",
                output="screen",
                parameters=[PathJoinSubstitution([config, "brain.yaml"])],
            ),
            ExecuteProcess(
                cmd=["python3", str(Path(__file__).with_name("gt_perception_stub.py"))],
                name="gt_perception_stub",
                output="screen",
            ),
        ]
    )
