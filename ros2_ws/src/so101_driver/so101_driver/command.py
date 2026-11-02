"""Merge of a /joint_command into the driver goal (docs/project/03 §4, C-14, C-15), and the setpoint it
publishes on /joint_setpoint (#200). Pure, no ROS."""

import math

from so101_common.robot import ARM_JOINT_NAMES, JOINT_LIMITS, JOINT_NAMES, pct_to_rad


def apply_command(goal: dict, names: list, positions: list) -> tuple[dict, str | None]:
    """New goal for a command, or the unchanged goal and the reason it was rejected.

    goal: arm joints in rad, gripper in %. No name = JOINT_NAMES order. A size mismatch or any
    non-finite value rejects the whole message; unknown names are ignored; arm joints are clamped
    to the URDF limits, the gripper to 0-100 %. Joints not named keep their goal (partial command).
    """
    names = list(names) or list(JOINT_NAMES)
    if len(names) != len(positions):
        return goal, f"{len(names)} names, {len(positions)} positions"
    if not all(math.isfinite(p) for p in positions):
        return goal, "non-finite position"
    new = dict(goal)
    for name, value in zip(names, positions, strict=True):
        if name not in goal:
            continue
        lo, hi = (0.0, 100.0) if name == "gripper" else JOINT_LIMITS[name]
        new[name] = min(max(float(value), lo), hi)
    return new, None


def setpoint_positions(setpoint: dict) -> list[float]:
    """/joint_setpoint positions: JOINT_NAMES order, rad, the gripper converted from % (what RSP expects)."""
    return [*(setpoint[name] for name in ARM_JOINT_NAMES), pct_to_rad(setpoint["gripper"])]
