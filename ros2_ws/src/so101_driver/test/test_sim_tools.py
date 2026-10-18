"""Unit tests of the sim tools (docs/project/03 §7): the scene frame the ground truth is published in,
and the drop-box sampler of /sim/randomize, checked against the scene's camera."""

import itertools

import numpy as np
import pytest
from so101_sim import SO101Sim

from so101_driver.randomize import MIN_CUBE_DISTANCE, sample_box_xy

PAN_AXIS_X = 0.0388  # m: shoulder_pan axis in base_link (04 §1.1)
PAN_MAX = 1.91986  # rad: URDF limit of shoulder_pan
BOX_HALF, BOX_HEIGHT = 0.052, 0.05  # m: outer half-width and height of the drop_box walls (scene)
MARGIN_PX = 5


@pytest.fixture(scope="module")
def sim():
    sim = SO101Sim()
    sim.connect()
    yield sim
    sim.disconnect()


def project(T, K, p):
    """World point -> pixel (u, v). MuJoCo camera frame: x right, y up, looking along -z."""
    c = T[:3, :3].T @ (np.asarray(p) - T[:3, 3])
    return K[0, 2] + K[0, 0] * c[0] / -c[2], K[1, 2] - K[1, 1] * c[1] / -c[2]


def cube_positions(rng, n):
    """Drawn like the provided randomize_object(): r 0.18-0.30 m, azimuth -50° to 60°."""
    r, a = rng.uniform(0.18, 0.30, n), np.radians(rng.uniform(-50.0, 60.0, n))
    return np.column_stack([r * np.cos(a), r * np.sin(a)])


def test_1000_boxes_are_reachable_fully_visible_and_clear_of_the_cube(sim):
    T, K = sim.get_camera_extrinsics(), sim.get_camera_intrinsics()
    width, height = sim.camera_width, sim.camera_height
    rng = np.random.default_rng(0)
    for cube in cube_positions(np.random.default_rng(1), 1000):
        box = sample_box_xy(rng, cube)
        assert np.linalg.norm(box - cube) >= MIN_CUBE_DISTANCE
        assert 0.18 <= np.linalg.norm(box) <= 0.30  # reachable at drop height (04 §4)
        assert abs(np.arctan2(box[1], box[0] - PAN_AXIS_X)) <= PAN_MAX - 0.1  # pan range, with margin
        for sx, sy, z in itertools.product((-1, 1), (-1, 1), (0.0, BOX_HEIGHT)):
            u, v = project(T, K, (box[0] + sx * BOX_HALF, box[1] + sy * BOX_HALF, z))
            assert MARGIN_PX <= u <= width - MARGIN_PX
            assert MARGIN_PX <= v <= height - MARGIN_PX


def test_a_seed_gives_the_same_sequence():
    cube = np.array([0.24, -0.1])

    def boxes(seed):
        rng = np.random.default_rng(seed)
        return [sample_box_xy(rng, cube) for _ in range(20)]

    np.testing.assert_array_equal(boxes(3), boxes(3))
    assert not np.allclose(boxes(3), boxes(4))


def test_the_mujoco_world_is_base_link(sim):
    """The ground truth is published in base_link as read from MuJoCo: the robot base sits at the origin."""
    np.testing.assert_array_equal(sim.data.body("base").xpos, [0.0, 0.0, 0.0])
    np.testing.assert_array_equal(sim.data.body("base").xquat, [1.0, 0.0, 0.0, 0.0])
