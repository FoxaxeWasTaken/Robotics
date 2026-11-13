"""robustness_demo.launch.py - one case of the robustness demo (step 5.4a, #103) on its own stack.

    ros2 launch so101_bringup robustness_demo.launch.py case:=<1|2|3a|3b|4|5> [lighting:=dim] [episodes:=5]
    ros2 launch so101_bringup robustness_demo.launch.py case:=<g1..g6> [lighting:=dim]     the game's (#243)

Starts sim.launch.py with perception in this container (the demo stops that node for cases 3 and 4),
robot_state_publisher (for RViz on this domain), then so101_bringup's robustness_demo for the case; the whole
launch ends with the demo. Run it alone, in a new control container on a ROS domain of its own, the everyday
stack stopped (results/robustness_demo.md gives the command): next to it, two simulations share the CPU and
/joint_states once stalled for 0.5 s (ERROR:no_joint_states). The demo refuses to run next to another
driver on its domain. No ground truth (C-06).
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

URDF = Path(get_package_share_directory("so101_description")) / "urdf" / "so101.urdf"


def generate_launch_description():
    demo = Node(
        package="so101_bringup",
        executable="robustness_demo",
        name="robustness_demo",
        output="screen",
        arguments=["--case", LaunchConfiguration("case"), "--episodes", LaunchConfiguration("episodes")],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "case", description="1, 2, 3a, 3b, 4 or 5; g1 to g6: the game's (so101_bringup.robustness)"
            ),
            DeclareLaunchArgument(
                "lighting", default_value="", description="A lighting entry of config/sim_scene.yaml (dim)"
            ),
            DeclareLaunchArgument("episodes", default_value="5", description="Case 5: cycles to run"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("so101_bringup"), "launch", "sim.launch.py"])
                ),
                launch_arguments={
                    "perception": "true",  # with the game: its board reader
                    "lighting": LaunchConfiguration("lighting"),
                    # the cases g1 to g6 are asked of a Connect Four game (#243)
                    "connect4": PythonExpression(["'", LaunchConfiguration("case"), "'.startswith('g')"]),
                }.items(),
            ),
            # What the viz service runs for RViz, on this domain: the arm's TFs, world -> base_link. Not its
            # drop_box_marker_pub (course kit): it raises at every stop (rclpy shut down twice); RViz shows
            # the box from /drop_box_position anyway.
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": URDF.read_text()}],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
            ),
            demo,
            RegisterEventHandler(
                OnProcessExit(target_action=demo, on_exit=[EmitEvent(event=Shutdown(reason="demo done"))])
            ),
        ]
    )
