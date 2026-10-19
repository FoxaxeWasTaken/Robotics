"""Unit test: the brain schedules on the steady clock (#112).

On WSL2 the system clock steps back about 2 s every 30 s: timers on the ROS clock pause until it catches up.
Message stamps stay on the ROS clock; the motion time is checked by test_clock_step_launch.py.
"""

import rclpy
from rclpy.clock import ClockType

from so101_brain.brain_node import BrainNode


def test_every_timer_runs_on_the_steady_clock():
    rclpy.init()
    try:
        node = BrainNode()
        try:
            clocks = [timer.clock.clock_type for timer in node.timers]
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()
    assert clocks == [ClockType.STEADY_TIME] * 3  # /end_effector_pose, FSM tick and stream, /brain_state
