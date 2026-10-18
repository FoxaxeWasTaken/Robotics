"""Unit tests of the sim tools (docs/project/03 §7): the scene frame the ground truth is published in."""

import numpy as np
import pytest
from so101_sim import SO101Sim


@pytest.fixture(scope="module")
def sim():
    sim = SO101Sim()
    sim.connect()
    yield sim
    sim.disconnect()


def test_the_mujoco_world_is_base_link(sim):
    """The ground truth is published in base_link as read from MuJoCo: the robot base sits at the origin."""
    np.testing.assert_array_equal(sim.data.body("base").xpos, [0.0, 0.0, 0.0])
    np.testing.assert_array_equal(sim.data.body("base").xquat, [1.0, 0.0, 0.0, 0.0])
