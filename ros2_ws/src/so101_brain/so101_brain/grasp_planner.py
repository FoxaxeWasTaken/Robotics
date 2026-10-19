"""Grasp planner (docs/project/04 §5, ADR-009). Pure, no ROS.

From the cube and box positions, the joint configurations of a pick & place. Every configuration comes from
the Kinematics façade, which checks each IK solution by FK and rejects unconverged ones: a plan never holds
an unverified solution. Failure gives a reason usable as an FSM error cause.
"""

from dataclasses import dataclass

import numpy as np

UP = np.array([0.0, 0.0, 1.0])


@dataclass(frozen=True)
class GraspPlan:
    pre_grasp: np.ndarray  # pregrasp_backoff_m back along the approach axis, same orientation
    grasp: np.ndarray  # TCP at the cube center
    lift: np.ndarray  # TCP lift_dz_m above the cube center, orientation free
    drop: np.ndarray  # TCP drop_tcp_z_m above the box floor, the gripper as vertical as reachable
    tilt_deg: float  # approach tilt from vertical, in the arm plane, tool tip outwards
    aligned: bool  # jaws perpendicular to a face; False: free roll


class GraspPlanner:
    """Defaults: the brain's grasp.* parameters (06 §6)."""

    def __init__(
        self,
        kinematics,
        tilts_deg=(0.0, 10.0, 20.0, 30.0),
        pregrasp_backoff_m=0.04,
        lift_dz_m=0.10,
        drop_tcp_z_m=0.09,
    ):
        self._kin = kinematics
        self._tilts_deg = tuple(tilts_deg)
        self._backoff = pregrasp_backoff_m
        self._lift_dz = lift_dz_m
        self._drop_z = drop_tcp_z_m

    def plan(self, cube_position, cube_yaw, box_position, q_current):
        """(GraspPlan, None), or (None, reason): "unreachable", "lift_unreachable" or "drop_unreachable".

        Positions in base_link (m); cube_yaw in rad, 90°-symmetric, None if unknown (free grasp only);
        q_current: the measured arm joints, first IK seed.
        """
        cube = np.asarray(cube_position, dtype=float)
        found = self._grasp(cube, cube_yaw, np.asarray(q_current, dtype=float))
        if found is None:
            return None, "unreachable"
        pre_grasp, grasp, tilt_deg, aligned = found
        lift = self._kin.ik_pos(cube + self._lift_dz * UP, q_seed=pre_grasp, roll=pre_grasp[4])
        if lift is None:
            return None, "lift_unreachable"
        drop = self._drop(np.asarray(box_position, dtype=float) + self._drop_z * UP, lift)
        if drop is None:
            return None, "drop_unreachable"
        return GraspPlan(pre_grasp, grasp, lift, drop, tilt_deg, aligned), None

    def _grasp(self, cube, yaw, q_current):
        """Smallest tilt first; at each tilt, jaws aligned to a face then free roll (04 §5).

        Of the 4 faces, the one with the shortest first motion (its largest joint change), not the first
        found: from the rest pose, the first found turned the wrist by 2.4-2.7 rad in half the scenes.
        """
        azimuth = self._kin.arm_azimuth(cube)
        for tilt_deg in self._tilts_deg:
            approach = tilted_down(tilt_deg, azimuth)
            back = cube - self._backoff * approach
            candidates = []
            for k in range(4) if yaw is not None else ():
                R = aligned_orientation(approach, yaw + k * np.pi / 2)
                grasp = self._kin.ik_pose(cube, R, q_seed=q_current)
                pre_grasp = None if grasp is None else self._kin.ik_pose(back, R, q_seed=grasp)
                if pre_grasp is not None:
                    candidates.append((pre_grasp, grasp))
            if candidates:
                pre_grasp, grasp = min(candidates, key=lambda c: np.max(np.abs(c[0] - q_current)))
                return pre_grasp, grasp, tilt_deg, True
            grasp = self._kin.ik_point(cube, approach, q_seed=q_current)
            pre_grasp = (
                None if grasp is None else self._kin.ik_point(back, approach, q_seed=grasp, roll=grasp[4])
            )
            if pre_grasp is not None:
                return pre_grasp, grasp, tilt_deg, False
        return None

    def _drop(self, p, lift):
        """The gripper as vertical as reachable, smallest tilt first: tilted 62-68°, the cube stayed hooked on
        a jaw inside one of its tunnels (2/40 in the closed loop, 2.4b).

        The lift's roll kept: turning it costs seconds of motion (up to 4 rad), a 30° tilt does not. With the
        roll fixed, the approach can only tilt in the arm's own plane (azimuth -pan, the TCP sits 8 mm off
        it), so the position-only solution gives the pan first. Free roll next, position only last.
        """
        q = self._kin.ik_pos(p, q_seed=lift, roll=lift[4])
        if q is None:
            return self._kin.ik_pos(p, q_seed=lift)
        for roll, azimuth in ((q[4], -q[0]), (None, self._kin.arm_azimuth(p))):
            for tilt_deg in self._tilts_deg:
                d = self._kin.ik_point(p, tilted_down(tilt_deg, azimuth), q_seed=q, roll=roll)
                if d is not None:
                    return d
        return q


def tilted_down(tilt_deg, azimuth) -> np.ndarray:
    """Approach axis tilted from vertical in the arm plane, tool tip outwards (04 §4)."""
    t = np.radians(tilt_deg)
    return np.array([np.sin(t) * np.cos(azimuth), np.sin(t) * np.sin(azimuth), -np.cos(t)])


def aligned_orientation(approach, face_yaw) -> np.ndarray:
    """TCP orientation: +z along the approach, the jaws (+x) along the face normal at face_yaw, projected
    perpendicular to the approach (exact for a vertical approach)."""
    x = np.array([np.cos(face_yaw), np.sin(face_yaw), 0.0])
    x -= approach * (x @ approach)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(approach, x), approach])
