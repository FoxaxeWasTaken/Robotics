"""limiter.py - speed limiter of the driver (ADR-023), pure Python.

Safety net, idle in nominal operation (the brain already sends smooth trajectories): it stops a
`ros2 topic pub` from jumping a joint by 2 rad in one tick.
"""

from so101_common.robot import ARM_JOINT_NAMES


def step_towards(
    setpoint: dict, goal: dict, dt: float, max_velocity: float, max_gripper_velocity: float
) -> dict:
    """Moves each joint of `setpoint` towards `goal` by at most its max velocity x dt.

    Arm joints in rad (max_velocity in rad/s), the gripper in % (max_gripper_velocity in %/s).
    """
    out = {}
    for name, target in goal.items():
        max_step = (max_gripper_velocity if name == "gripper" else max_velocity) * dt
        out[name] = setpoint[name] + max(-max_step, min(max_step, target - setpoint[name]))
    return out


def max_relative_target(arm: float, gripper: float) -> dict:
    """LeRobot's `max_relative_target` per motor (07 §2): arm joints in rad, the gripper in %.

    A single value would apply the arm's 0.2 to the gripper's percent too, and freeze it.
    """
    return {**dict.fromkeys(ARM_JOINT_NAMES, arm), "gripper": gripper}
