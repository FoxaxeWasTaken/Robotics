"""shutdown.py - shutdown move of the driver (docs/project/03 §8).

disconnect() cuts the torque of the real arm, which then falls: the driver first brings it slowly to its rest
pose. Direct backend calls, no ROS: on SIGINT the context is already down.
"""

import time

from so101_driver.limiter import step_towards


def move_to(robot, setpoint, goal, rate_hz, velocity, timeout, tolerance):
    """Moves the joints of `goal` (rad) from `setpoint` through the speed limiter; the others stay put.

    Stops once each of them is within `tolerance` (rad) of the goal, or after `timeout` (s).
    Returns (elapsed s, largest remaining error in rad). Bus errors propagate.
    """
    start = time.monotonic()
    while True:
        obs = robot.get_observation()
        error = max(abs(obs[f"{name}.pos"] - value) for name, value in goal.items())
        elapsed = time.monotonic() - start
        if error <= tolerance or elapsed >= timeout:
            return elapsed, error
        setpoint = step_towards(setpoint, goal, 1.0 / rate_hz, velocity, 0.0)
        robot.send_action({f"{name}.pos": value for name, value in setpoint.items()})
        time.sleep(1.0 / rate_hz)
