"""late.py - late timer ticks of the driver (#176), pure Python.

The control step and /joint_states share one executor: anything that holds it shows as a tick that came
late. The driver warns beyond LATE_WARN_S, before the brain stops on stale /joint_states.
"""

LATE_WARN_S = 0.25  # half the brain's max_joint_state_age_s (brain.yaml, 0.5 s)


class TickClock:
    """The delay of each tick of a periodic callback, beyond its period."""

    def __init__(self, period_s: float):
        self.period_s = period_s
        self._last = None

    def late(self, now: float) -> float:
        """How much later than one period after the previous tick this one came, s (0 for the first)."""
        late = 0.0 if self._last is None else max(0.0, now - self._last - self.period_s)
        self._last = now
        return late
