"""Unit tests of the kinematics façade (docs/project/04 §1-§3, §8). No ROS, no sim: FK against MuJoCo is
checked in so101_bringup (sim-backed tests, C-05)."""

import itertools
import time
from pathlib import Path

import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory
from hypothesis import given, settings
from hypothesis import strategies as st

from so101_brain.kinematics import Kinematics
from so101_common.robot import ARM_JOINT_NAMES, JOINT_LIMITS

URDF = Path(get_package_share_directory("so101_description")) / "urdf" / "so101.urdf"
LOW = np.array([JOINT_LIMITS[n][0] for n in ARM_JOINT_NAMES])
HIGH = np.array([JOINT_LIMITS[n][1] for n in ARM_JOINT_NAMES])
configurations = st.tuples(*(st.floats(lo, hi) for lo, hi in zip(LOW, HIGH, strict=True))).map(np.array)


@pytest.fixture(scope="module")
def kin():
    return Kinematics(str(URDF))


def tcp(kin, q):
    return kin.fk(q)[:3, 3]


def test_tcp_at_zero_is_the_stretched_arm(kin):
    np.testing.assert_allclose(tcp(kin, np.zeros(5)), [0.391, 0.0, 0.226], atol=1e-3)  # 04 §1.1


def test_azimuth_is_measured_from_the_pan_axis(kin):
    assert kin.arm_azimuth([0.0388 + 0.1, 0.1, 0.05]) == pytest.approx(np.pi / 4)
    assert kin.arm_azimuth([0.0388 + 0.2, 0.0, 0.05]) == pytest.approx(0.0)
    assert kin.arm_azimuth([0.0388 + 0.1, -0.1, 0.05]) == pytest.approx(-np.pi / 4)


@settings(max_examples=200, deadline=None)
@given(configurations)
def test_ik_reaches_every_reachable_position(kin, q):
    p = tcp(kin, q)
    solution = kin.ik_pos(p)
    assert solution is not None
    assert np.linalg.norm(tcp(kin, solution) - p) <= 1e-3
    assert np.all(solution >= LOW - 1e-9)
    assert np.all(solution <= HIGH + 1e-9)


def test_a_seed_past_the_bounds_does_not_crash(kin):
    # Real arm at rest: elbow 1.7007 > 1.69; sim overshoot on wrist_roll. Unclipped: "x0 is infeasible".
    seed = np.array([0.0, 0.0, 1.7007, 0.0, 3.0])
    assert kin.ik_pos([0.22, 0.05, 0.10], q_seed=seed) is not None


def test_an_unreachable_position_gives_none(kin):
    assert kin.ik_pos([1.0, 0.0, 0.0]) is None


DOWN = np.array([0.0, 0.0, -1.0])


def angle(u, v):
    return float(np.arccos(np.clip(np.dot(u, v) / np.linalg.norm(u) / np.linalg.norm(v), -1.0, 1.0)))


def grasp_targets(n, seed=0):
    """In the cube's randomization area (r 0.18-0.30 m, azimuth -50° to 60°), z 2-6 cm."""
    rng = np.random.default_rng(seed)
    r, a = rng.uniform(0.18, 0.30, n), np.radians(rng.uniform(-50.0, 60.0, n))
    return np.column_stack([r * np.cos(a), r * np.sin(a), rng.uniform(0.02, 0.06, n)])


def reachable_by_search(kin, p, approach):
    """Is there a solution at all? 100 extra seeds over the shoulder, elbow and wrist ranges."""
    pan = -kin.arm_azimuth(p)
    grid = itertools.product(np.linspace(-1.6, 1.6, 5), np.linspace(-1.5, 1.5, 5), np.linspace(-1.5, 1.5, 4))
    return any(kin.ik_point(p, approach, q_seed=np.array([pan, *seed, 0.0])) is not None for seed in grid)


def test_ik_point_points_down_at_95_percent_of_the_reachable_low_targets(kin):
    times, found, unreachable = [], 0, 0
    for p in grasp_targets(200):
        start = time.perf_counter()
        q = kin.ik_point(p, DOWN)
        times.append(time.perf_counter() - start)
        if q is not None:
            found += 1
            pose = kin.fk(q)
            assert np.linalg.norm(pose[:3, 3] - p) <= 1e-3
            assert angle(pose[:3, 2], DOWN) <= np.radians(2.0)  # TCP +z of the URDF frame = approach
        elif not reachable_by_search(kin, p, DOWN):
            unreachable += 1  # far and high: a vertical approach cannot reach it (04 §4)
    rate = found / (200 - unreachable)
    print(
        f"ik_point vertical: {found}/200 found, {unreachable} with no vertical solution at all: "
        f"{rate * 100:.1f} % of the reachable ones, p95 {np.percentile(times, 95) * 1000:.0f} ms"
    )
    assert rate >= 0.95  # MES-21: 97 %


def test_ik_point_with_a_fixed_roll_keeps_it_and_resolves_the_position(kin):
    p = np.array([0.22, 0.05, 0.03])
    q = kin.ik_point(p, DOWN, roll=0.7)
    assert q is not None
    assert q[4] == pytest.approx(0.7)
    pose = kin.fk(q)
    assert np.linalg.norm(pose[:3, 3] - p) <= 1e-3  # the TCP sits 8 mm off the roll axis (04 §8)
    assert angle(pose[:3, 2], DOWN) <= np.radians(2.0)


def test_ik_pose_reaches_aligned_grasp_poses(kin):
    """Approach down, jaws (TCP x) along a horizontal direction: the planner's ALIGNED attempt."""
    rng = np.random.default_rng(1)
    solved = 0
    for p in grasp_targets(50, seed=1):
        yaw = rng.uniform(-np.pi, np.pi)
        x = np.array([np.cos(yaw), np.sin(yaw), 0.0])
        R = np.column_stack([x, np.cross(DOWN, x), DOWN])
        q = kin.ik_pose(p, R)
        if q is not None:
            solved += 1
            pose = kin.fk(q)
            assert np.linalg.norm(pose[:3, 3] - p) <= 1e-3
            assert angle(pose[:3, 0], x) <= np.radians(2.0)
            assert angle(pose[:3, 2], DOWN) <= np.radians(2.0)
    print(f"ik_pose aligned and vertical: {solved * 2} % of 50")
    assert solved > 0


def test_ik_pose_finds_the_pose_of_a_nearby_configuration(kin):
    q = np.array([0.3, -0.4, 0.6, 1.1, 0.5])
    pose = kin.fk(q)
    solution = kin.ik_pose(pose[:3, 3], pose[:3, :3], q_seed=q + 0.05)
    assert solution is not None
    assert np.linalg.norm(tcp(kin, solution) - pose[:3, 3]) <= 1e-3


def test_unreachable_orientations_give_none(kin):
    assert kin.ik_point([0.5, 0.0, 0.05], DOWN) is None
    assert kin.ik_pose([0.5, 0.0, 0.05], np.diag([1.0, -1.0, -1.0])) is None


def test_an_impossible_orientation_at_a_reachable_position_gives_none(kin):
    """Position reachable, approach pointing up: the solver may converge on the position only."""
    p, up = np.array([0.22, 0.0, 0.05]), np.array([0.0, 0.0, 1.0])
    assert kin.ik_pos(p) is not None
    assert kin.ik_point(p, up) is None
    assert kin.ik_pose(p, np.eye(3)) is None  # TCP +z up


def test_ik_pos_with_a_fixed_roll_keeps_it(kin):
    """The roll barely moves the TCP (8 mm off its axis): left free, the solver may spin it for nothing."""
    q = np.array([0.2, -0.3, 0.5, 1.2, 1.0])
    p = tcp(kin, q) + np.array([0.0, 0.0, 0.06])  # a lift
    solution = kin.ik_pos(p, q_seed=q, roll=q[4])
    assert solution[4] == pytest.approx(q[4])
    assert np.linalg.norm(tcp(kin, solution) - p) <= 1e-3


def test_the_fk_check_rejects_a_solution_off_the_orientation(kin, monkeypatch):
    """The FK check decides, not the solver: a solution right on the position only is never returned."""
    q = np.array([0.2, -0.3, 0.5, 1.2, 0.0])  # its approach is far from vertical-up
    for chain in (kin._chain, kin._chain_fixed_roll):
        monkeypatch.setattr(chain, "inverse_kinematics", lambda **_: kin._full(q))
    p, up = tcp(kin, q), np.array([0.0, 0.0, 1.0])
    assert kin.ik_pos(p) is not None  # the position is right
    assert kin.ik_point(p, up) is None
    assert kin.ik_point(p, up, roll=0.0) is None
    assert kin.ik_pose(p, np.eye(3)) is None
