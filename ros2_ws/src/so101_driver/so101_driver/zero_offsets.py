"""zero_offsets.py - the real arm seen in URDF angles (docs/project/07 §4, MES-30, #196).

LeRobot's zero is the middle of each joint's recorded range, not the URDF's: the arm held at the URDF zero
reads these offsets. The driver wraps the backend after the provided _connect_arm (ADR-032).
"""


class ZeroOffsets:
    """The backend, with each joint's offset (rad) subtracted from what it reads and added to what it is sent.

    Joints without an offset (the gripper, in %) pass unchanged; every other attribute is the backend's.
    """

    def __init__(self, robot, offsets: dict):
        self._robot, self._offsets = robot, dict(offsets)

    def _shift(self, values: dict, sign: float) -> dict:
        return {
            key: value + sign * self._offsets.get(key.removesuffix(".pos"), 0.0)
            for key, value in values.items()
        }

    def get_observation(self) -> dict:
        return self._shift(self._robot.get_observation(), -1.0)

    def send_action(self, action: dict) -> dict:
        return self._robot.send_action(self._shift(action, 1.0))

    def __getattr__(self, name):
        return getattr(self._robot, name)
