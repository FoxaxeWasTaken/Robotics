"""Forward and inverse kinematics of the arm (docs/project/04 §1-§3, §8-§9, ADR-008). Pure, no ROS.

Façade over ikpy (Pinocchio possible later). Arm joints only: 5 values in ARM_JOINT_NAMES order, in rad;
the gripper is not in the chain. Positions in base_link, in m.
"""

import itertools
import warnings

import numpy as np
from ikpy.chain import Chain

from so101_common.robot import ARM_JOINT_NAMES

PAN_AXIS_X = 0.0388  # m: the shoulder_pan axis is vertical at (x, y) = (0.0388, 0) in base_link [MES-06]
MAX_POSITION_ERROR = 1e-3  # m: every IK solution is checked by FK, worse ones are rejected (04 §8)
SEED_TAILS = ((-0.3, 0.3, 1.2, 0.0), (0.5, -0.5, 1.5, 0.0), (-1.0, 1.0, 0.5, 0.0))  # 04 §3.1, after pan
# When those fail (target past the pan range, |azimuth| > 1.92 rad, or a folded arm): both pan families
# (facing the target, or turned by pi and reaching back), shoulder and elbow both ways. 0 failures in 2000
# random configurations instead of 307; the forward targets still solve with the first seed [MES-36].
FALLBACK_TAILS = tuple(
    (lift, elbow, wrist, 0.0)
    for lift, elbow, wrist in itertools.product((-1.2, 1.2), (-1.2, 1.2), (0.0, 1.2))
)


class Kinematics:
    def __init__(self, urdf_path: str):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")  # ikpy: "fixed joint with an axis" in the course URDF
            chain = Chain.from_urdf_file(
                urdf_path, base_elements=["base_link"], active_links_mask=[False, *[True] * 5, False]
            )
        names = [link.name for link in chain.links]
        # ikpy follows the first child joint: the chain must end at the TCP, not at the moving jaw (04 §8).
        if names[1:] != [*ARM_JOINT_NAMES, "gripper_frame_joint"]:
            raise ValueError(f"unexpected ikpy chain from {urdf_path}: {names}")
        self._chain = chain
        # The chain's own bounds: the ones scipy checks the seed against.
        self._low, self._high = np.array([link.bounds for link in chain.links[1:6]]).T

    def fk(self, q) -> np.ndarray:
        """4x4 pose of gripper_frame_link (TCP) in base_link."""
        return self._chain.forward_kinematics(self._full(q))

    def arm_azimuth(self, p) -> float:
        """Azimuth of a point seen from the pan axis (not from the origin), in rad."""
        return float(np.arctan2(p[1], p[0] - PAN_AXIS_X))

    def ik_pos(self, p, q_seed=None) -> np.ndarray | None:
        """Joints that put the TCP at p (orientation free), or None.

        Seeds in order: q_seed, pan = -azimuth (04 §3.1), then the fallback grid.
        """
        p = np.asarray(p, dtype=float)
        azimuth = self.arm_azimuth(p)
        behind = (-azimuth + 2 * np.pi) % (2 * np.pi) - np.pi  # pan turned by pi, wrapped to (-pi, pi]
        seeds = [np.array([-azimuth, *tail]) for tail in SEED_TAILS]
        seeds += [np.array([pan, *tail]) for pan in (-azimuth, behind) for tail in FALLBACK_TAILS]
        if q_seed is not None:
            seeds.insert(0, np.asarray(q_seed, dtype=float))
        for seed in seeds:
            # Measured joints can sit past a URDF bound: unclipped, scipy raises "x0 is infeasible".
            start = self._full(np.clip(seed, self._low, self._high))
            q = self._chain.inverse_kinematics(target_position=p, initial_position=start)[1:6]
            if np.linalg.norm(self.fk(q)[:3, 3] - p) <= MAX_POSITION_ERROR:
                return q
        return None

    @staticmethod
    def _full(q) -> np.ndarray:
        """ikpy's vector: the fixed base and TCP links around the 5 arm joints."""
        return np.concatenate(([0.0], np.asarray(q, dtype=float), [0.0]))
