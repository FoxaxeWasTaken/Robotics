"""The drop pose of phase 2 (docs/project/phase2/P2-02 §2): the joints that hold a token above a column of an
edge-on grid, the jaws normal to the grid. Pure, no ROS. kinematics.py is unchanged: release_pose composes its
ik_pos and ik_point.

The target is the token's centre, not the TCP: the token rests on the fixed jaw, 3.15 mm off the TCP. The pose
documented first (roll ±90°, the TCP on the slot plane) dropped 0 token of 28 [MES-64]. Where to aim across
the slot (the layout's plane offset and the 1 mm of P2-02 §2.2) is the caller's: release_pose puts the centre
on the target it gets.
"""

import numpy as np

from so101_brain.grasp_planner import tilted_down

# rad: +90° and the URDF wrist's 2.8° (wrist_roll rpy 0.0487), the jaws normal to the arm plane [MES-64]
ROLL = 1.6195
# Approach tilted from vertical, outwards: it solves all 7 columns; bare ik_pos flips the tool [MES-64]
TILT_DEG = 70.0
TOKEN_IN_TCP = np.array([-0.00315, 0.0, -0.001])  # m: the held token's centre in gripper_frame_link [MES-64]
SEED_TAIL = (-0.4, 0.0, 1.0)  # lift, elbow, wrist_flex of the default seed: bent (q = 0 is singular)


def release_pose(kin, target, q_seed=None) -> np.ndarray | None:
    """Joints that put the held token's centre at `target` (base_link, m), wrist_roll at ROLL and the approach
    tilted TILT_DEG; None if the IK finds none (unreachable, or only folded onto the arm).

    ik_pos with the roll locked gives the pan, so the arm plane and the token's offset in it; twice, since the
    pan moves the offset. Then ik_point tilts the approach in that plane, seeded with the solution.
    """
    target = np.asarray(target, dtype=float)
    azimuth = kin.arm_azimuth(target)
    seed = np.array([-azimuth, *SEED_TAIL, ROLL]) if q_seed is None else np.asarray(q_seed, dtype=float)
    for _ in range(2):
        p = target - _release_axes(azimuth) @ TOKEN_IN_TCP
        q = kin.ik_pos(p, q_seed=seed, roll=ROLL)
        if q is None:
            return None
        azimuth, seed = -q[0], q  # pan = -azimuth (04 §3.1)
    return kin.ik_point(p, tilted_down(TILT_DEG, azimuth), q_seed=seed, roll=ROLL)


def _release_axes(azimuth) -> np.ndarray:
    """The TCP's axes at the release, as columns: x the jaws' closing axis across the arm plane, z the
    approach tilted outwards in it."""
    z = tilted_down(TILT_DEG, azimuth)
    x = np.array([np.sin(azimuth), -np.cos(azimuth), 0.0])
    return np.column_stack([x, np.cross(z, x), z])
