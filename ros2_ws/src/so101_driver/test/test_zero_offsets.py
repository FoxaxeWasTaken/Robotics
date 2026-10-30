"""The real arm seen in URDF angles (#196, 07 §4, MES-30): LeRobot's zero is the middle of each recorded
range, not the URDF's; the driver subtracts each joint's offset from what it reads and adds it to what it
sends."""

import pytest

from so101_driver.zero_offsets import ZeroOffsets

OFFSETS = {"shoulder_pan": -0.028, "elbow_flex": 0.241}


class Backend:
    """The motors' side: positions in LeRobot's angles."""

    def __init__(self):
        self.positions = {"shoulder_pan.pos": 0.0, "elbow_flex.pos": 0.241, "gripper.pos": 40.0}
        self.sent = None
        self.config = "the backend's config"

    def get_observation(self):
        return dict(self.positions)

    def send_action(self, action):
        self.sent = action
        return action


def test_a_reading_is_the_lerobot_angle_minus_the_offset():
    arm = ZeroOffsets(Backend(), OFFSETS)
    assert arm.get_observation() == pytest.approx(
        {"shoulder_pan.pos": 0.028, "elbow_flex.pos": 0.0, "gripper.pos": 40.0}
    )


def test_a_command_is_the_urdf_angle_plus_the_offset():
    backend = Backend()
    ZeroOffsets(backend, OFFSETS).send_action({"elbow_flex.pos": 0.0, "gripper.pos": 100.0})
    assert backend.sent == pytest.approx({"elbow_flex.pos": 0.241, "gripper.pos": 100.0})


def test_a_command_of_what_was_read_keeps_the_motors_where_they_are():
    backend = Backend()
    arm = ZeroOffsets(backend, OFFSETS)
    arm.send_action(arm.get_observation())
    assert backend.sent == pytest.approx(backend.positions)


def test_everything_else_is_the_backend_s():
    assert ZeroOffsets(Backend(), OFFSETS).config == "the backend's config"
