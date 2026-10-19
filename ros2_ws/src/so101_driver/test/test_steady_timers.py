"""Unit test: the driver schedules on the steady clock (#112).

On WSL2 the system clock steps back about 2 s every 30 s: timers on the ROS clock pause until it catches up.
Message stamps stay on the ROS clock.
"""

import rclpy
from rclpy.clock import ClockType

from so101_driver.driver_node import DriverNode


def test_every_timer_runs_on_the_steady_clock():
    rclpy.init(args=["--ros-args", "-p", "expose_ground_truth:=true"])  # its timer too
    try:
        node = DriverNode()
        try:
            clocks = [timer.clock.clock_type for timer in node.timers]
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()
    assert clocks == [ClockType.STEADY_TIME] * 3  # control, /joint_states, ground truth
