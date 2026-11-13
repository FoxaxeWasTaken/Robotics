"""Forward and inverse kinematics of the arm (docs/project/04 §1-§3, §8-§9, ADR-008). Pure, no ROS.

Façade over ikpy (Pinocchio possible later). Arm joints only: 5 values in ARM_JOINT_NAMES order, in rad;
the gripper is not in the chain. Positions in base_link, in m. Orientations are those of the URDF TCP frame
(gripper_frame_link), never the MuJoCo site (rotated by 90°): +z = approach axis, the jaws close along +x
(the jaw hinge lies along y).
"""

import itertools
import warnings

import numpy as np
from ikpy.chain import Chain

from so101_common.robot import ARM_JOINT_NAMES

PAN_AXIS_X = 0.0388  # m: the shoulder_pan axis is vertical at (x, y) = (0.0388, 0) in base_link [MES-06]
MAX_POSITION_ERROR = 1e-3  # m: every IK solution is checked by FK, worse ones are rejected (04 §8)
MAX_ANGLE_ERROR = np.radians(2.0)  # rad: same check for the approach axis and the full orientation
SEED_TAILS = ((-0.3, 0.3, 1.2, 0.0), (0.5, -0.5, 1.5, 0.0), (-1.0, 1.0, 0.5, 0.0))  # 04 §3.1, after pan
# When those fail (target past the pan range, |azimuth| > 1.92 rad, or a folded arm): both pan families
# (facing the target, or turned by pi and reaching back), shoulder and elbow both ways. 0 failures in 2000
# random configurations instead of 307; the forward targets still solve with the first seed [MES-36].
FALLBACK_TAILS = tuple(
    (lift, elbow, wrist, 0.0)
    for lift, elbow, wrist in itertools.product((-1.2, 1.2), (-1.2, 1.2), (0.0, 1.2))
)
# Self-collision (#145): ikpy knows no collision volumes. Over 30,000 random poses, MuJoCo found no contact
# between the arm's bodies while the elbow folds at most 1.55 rad and the lower arm, wrist and gripper (the
# polyline elbow -> wrist_flex -> wrist_roll -> TCP, 6 points per segment) stay out of a vertical cylinder of
# 9 cm around the pan axis below 12 cm [MES-43]. Margin for the real arm's calibration: 0.05 rad, 1 cm on the
# radius, 0.5 cm on the height (PARK's elbow sits at 13.0 cm, 8.7 cm from the axis: PARK stays clear).
MAX_ELBOW_FOLD = 1.50  # rad
ELBOW = ARM_JOINT_NAMES.index("elbow_flex")
SHOULDER_RADIUS = 0.10  # m, around the pan axis
SHOULDER_HEIGHT = 0.125  # m, above base_link


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
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            # Same chain, wrist_roll inactive: ikpy keeps it at its initial value (ik_point with a roll).
            self._chain_fixed_roll = Chain.from_urdf_file(
                urdf_path, base_elements=["base_link"], active_links_mask=[False, *[True] * 4, False, False]
            )
        # The chain's own bounds: the ones scipy checks the seed against.
        self._low, self._high = np.array([link.bounds for link in chain.links[1:6]]).T

    def fk(self, q) -> np.ndarray:
        """4x4 pose of gripper_frame_link (TCP) in base_link."""
        return self._chain.forward_kinematics(self._full(q))

    def link_frames(self, q) -> list:
        """4x4 poses in base_link of the 5 arm links, shoulder to gripper: the frames the URDF gives each
        link's meshes in (clearance.py)."""
        return self._chain.forward_kinematics(self._full(q), full_kinematics=True)[1:6]

    def arm_azimuth(self, p) -> float:
        """Azimuth of a point seen from the pan axis (not from the origin), in rad."""
        return float(np.arctan2(p[1], p[0] - PAN_AXIS_X))

    def ik_pos(self, p, q_seed=None, roll=None) -> np.ndarray | None:
        """Joints that put the TCP at p (orientation free), or None.

        Seeds in order: q_seed, pan = -azimuth (04 §3.1), then the fallback grid. With `roll`, wrist_roll
        stays at that value: the roll barely moves the TCP (8 mm off its axis), and left free the solver may
        spin it by up to 4 rad for a 6 cm lift, seconds of motion (04 §8).
        """
        p = np.asarray(p, dtype=float)
        if roll is not None:
            return self._solve(self._chain_fixed_roll, self._seeds(p, q_seed, roll), p, None, None)
        azimuth = self.arm_azimuth(p)
        behind = (-azimuth + 2 * np.pi) % (2 * np.pi) - np.pi  # pan turned by pi, wrapped to (-pi, pi]
        seeds = self._seeds(p, q_seed)
        seeds += [np.array([pan, *tail]) for pan in (-azimuth, behind) for tail in FALLBACK_TAILS]
        return self._solve(self._chain, seeds, p, None, None)

    def ik_point(self, p, approach, q_seed=None, roll=None) -> np.ndarray | None:
        """Joints that put the TCP at p with its +z along `approach` (roll free), or None.

        With `roll`, wrist_roll stays at that value and the position is solved with it: the TCP sits 8 mm off
        the roll axis (04 §8). Seeds: q_seed, then pan = -azimuth (04 §3.1); no fallback grid (grasps only).
        Without `roll`, the seed's roll is tried first: the rotation about the approach is free, keeping the
        roll keeps the motion short (the planner's free grasp and drop, 04 §5).
        """
        a = np.asarray(approach, dtype=float) / np.linalg.norm(approach)
        if roll is not None:
            ok = self._low[4] <= roll <= self._high[4]
            return (
                self._solve(self._chain_fixed_roll, self._seeds(p, q_seed, roll), p, a, "Z") if ok else None
            )
        if q_seed is not None:
            seeds = self._seeds(p, q_seed, roll=q_seed[4])
            if (q := self._solve(self._chain_fixed_roll, seeds, p, a, "Z")) is not None:
                return q
        return self._solve(self._chain, self._seeds(p, q_seed), p, a, "Z")

    def ik_pose(self, p, R, q_seed=None) -> np.ndarray | None:
        """Joints that put the TCP at p with the orientation R (3x3, in base_link), or None. As ik_point."""
        return self._solve(self._chain, self._seeds(p, q_seed), p, np.asarray(R, dtype=float), "all")

    def _seeds(self, p, q_seed, roll=None) -> list:
        """q_seed, then pan = -azimuth (04 §3.1); with `roll`, every seed holds it, clipped to its bounds."""
        azimuth = self.arm_azimuth(p)
        seeds = [np.array([-azimuth, *tail]) for tail in SEED_TAILS]
        if q_seed is not None:
            seeds.insert(0, np.array(q_seed, dtype=float))
        if roll is not None:
            for seed in seeds:
                seed[4] = np.clip(roll, self._low[4], self._high[4])
        return seeds

    def self_collides(self, q) -> bool:
        """The arm folded onto its shoulder or base (#145, 04 §8): elbow past MAX_ELBOW_FOLD, or a point of
        the lower arm, wrist or gripper inside the shoulder cylinder. Pure geometry of the chain."""
        if q[ELBOW] > MAX_ELBOW_FOLD:
            return True
        frames = self._chain.forward_kinematics(self._full(q), full_kinematics=True)
        joints = [frame[:3, 3] for frame in frames[3:]]  # elbow, wrist_flex, wrist_roll, TCP
        t = np.linspace(0.0, 1.0, 6)[:, None]
        points = np.concatenate([a + t * (b - a) for a, b in itertools.pairwise(joints)])
        near_axis = np.hypot(points[:, 0] - PAN_AXIS_X, points[:, 1]) < SHOULDER_RADIUS
        return bool(np.any(near_axis & (points[:, 2] < SHOULDER_HEIGHT)))

    def _solve(self, chain, seeds, p, orientation, mode) -> np.ndarray | None:
        """First solution from the seeds that FK confirms and that does not fold the arm onto itself, or None:
        never an unconverged or self-colliding one (04 §8)."""
        p = np.asarray(p, dtype=float)
        for seed in seeds:
            # Measured joints can sit past a URDF bound: unclipped, scipy raises "x0 is infeasible".
            start = self._full(np.clip(seed, self._low, self._high))
            q = chain.inverse_kinematics(
                target_position=p,
                target_orientation=orientation,
                orientation_mode=mode,
                initial_position=start,
            )[1:6]
            if self._confirms(q, p, orientation, mode) and not self.self_collides(q):
                return q
        return None

    def _confirms(self, q, p, orientation, mode) -> bool:
        pose = self.fk(q)
        if np.linalg.norm(pose[:3, 3] - p) > MAX_POSITION_ERROR:
            return False
        if mode == "Z":
            return _angle(pose[:3, 2], orientation) <= MAX_ANGLE_ERROR
        if mode == "all":  # angle of the rotation between the two frames
            return (
                np.arccos(np.clip((np.trace(pose[:3, :3].T @ orientation) - 1.0) / 2.0, -1.0, 1.0))
                <= MAX_ANGLE_ERROR
            )
        return True

    @staticmethod
    def _full(q) -> np.ndarray:
        """ikpy's vector: the fixed base and TCP links around the 5 arm joints."""
        return np.concatenate(([0.0], np.asarray(q, dtype=float), [0.0]))


def _angle(u, v) -> float:
    return float(np.arccos(np.clip(np.dot(u, v) / (np.linalg.norm(u) * np.linalg.norm(v)), -1.0, 1.0)))
