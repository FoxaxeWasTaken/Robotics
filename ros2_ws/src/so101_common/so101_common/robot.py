"""robot.py - single source of truth of the SO-ARM101 (ADR-003).

Joint names, joint limits, gripper % <-> rad conversion and frame names, defined once for the
driver, the brain and the tests. Pure Python on purpose: no rclpy, no backend.
"""

from types import MappingProxyType

ARM_JOINT_NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
JOINT_NAMES = (*ARM_JOINT_NAMES, "gripper")  # URDF chain order: the order of an unnamed /joint_command

# URDF limits (rad), so101_description/urdf/so101.urdf; test_urdf.py checks them against the file.
JOINT_LIMITS = MappingProxyType(
    {
        "shoulder_pan": (-1.91986, 1.91986),
        "shoulder_lift": (-1.74533, 1.74533),
        "elbow_flex": (-1.69, 1.69),
        "wrist_flex": (-1.65806, 1.65806),
        "wrist_roll": (-2.74385, 2.84121),  # asymmetric
        "gripper": (-0.174533, 1.74533),
    }
)

# LeRobot convention: 0 % = closed (lower limit), 100 % = open (upper limit).
GRIPPER_RANGE_RAD = JOINT_LIMITS["gripper"]

BASE_FRAME = "base_link"
TCP_FRAME = "gripper_frame_link"
CAMERA_FRAME = "external_cam_optical_frame"  # not in the URDF: published by the calibration (ADR-013)


def pct_to_rad(pct: float) -> float:
    """Gripper opening in % (clamped to 0-100) -> joint angle in rad."""
    lo, hi = GRIPPER_RANGE_RAD
    return lo + min(max(pct, 0.0), 100.0) / 100.0 * (hi - lo)


def rad_to_pct(rad: float) -> float:
    """Gripper joint angle in rad -> opening in %, clamped to 0-100."""
    lo, hi = GRIPPER_RANGE_RAD
    return min(max((rad - lo) / (hi - lo) * 100.0, 0.0), 100.0)
