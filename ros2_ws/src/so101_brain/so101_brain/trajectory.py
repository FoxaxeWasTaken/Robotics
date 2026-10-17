"""Minimum-jerk joint trajectories (docs/project/04 §6, ADR-010). Pure, no ROS.

Arm joints only, in rad: the gripper (in %, its own speed) is commanded on its own (06 §5),
so it never stretches the duration of an arm motion.
"""

from dataclasses import dataclass

import numpy as np

PEAK_VELOCITY = 1.875  # max of ds/dτ for s(τ) = 10τ³ - 15τ⁴ + 6τ⁵


@dataclass(frozen=True)
class MinJerk:
    q_start: np.ndarray
    q_goal: np.ndarray
    duration: float  # s

    def sample(self, t: float) -> np.ndarray:
        """Position at time t (s) from the start; before 0 the start, after the end the goal."""
        tau = 1.0 if self.duration <= 0.0 else min(max(t / self.duration, 0.0), 1.0)
        if tau >= 1.0:
            return self.q_goal.copy()
        s = tau**3 * (10.0 - 15.0 * tau + 6.0 * tau**2)
        return self.q_start + s * (self.q_goal - self.q_start)


def min_jerk(q_start, q_goal, v_max: float, t_min: float) -> MinJerk:
    """Quintic from q_start to q_goal; the joint that moves most reaches v_max at mid-motion."""
    q_start, q_goal = np.asarray(q_start, dtype=float), np.asarray(q_goal, dtype=float)
    longest = float(np.max(np.abs(q_goal - q_start), initial=0.0))
    return MinJerk(q_start, q_goal, max(t_min, longest * PEAK_VELOCITY / v_max))
