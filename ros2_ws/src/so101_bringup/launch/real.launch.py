"""real.launch.py - the control container on the real arm: sim.launch.py with use_sim:=false, so
driver_real.yaml and brain_real.yaml override a few values of driver.yaml and brain.yaml (01 §3.3).

    ros2 launch so101_bringup real.launch.py port:=/dev/ttyACM0

Started by control_entrypoint.sh when USE_SIM=false; the checklist of docs/project/07 §7 comes first.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "port", default_value="/dev/ttyACM0", description="Serial port of the real arm"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("so101_bringup"), "launch", "sim.launch.py"])
                ),
                launch_arguments={"use_sim": "false", "port": LaunchConfiguration("port")}.items(),
            ),
        ]
    )
