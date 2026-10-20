"""Unit tests of the grasp planner (docs/project/04 §5, ADR-009). Pure: no ROS, no sim."""

import time
from pathlib import Path

import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory

from so101_brain.fsm import Params
from so101_brain.grasp_planner import GraspPlanner, aligned_orientation
from so101_brain.kinematics import Kinematics

URDF = Path(get_package_share_directory("so101_description")) / "urdf" / "so101.urdf"
REST = np.zeros(5)  # q_current: the startup pose
BOX = np.array([0.05, -0.25, 0.0])  # drop_box of the provided scene
DOWN = np.array([0.0, 0.0, -1.0])
UP = -DOWN


@pytest.fixture(scope="module")
def kin():
    return Kinematics(str(URDF))


@pytest.fixture(scope="module")
def planner(kin):
    return GraspPlanner(kin)


def angle(u, v):
    return float(np.arccos(np.clip(np.dot(u, v) / np.linalg.norm(u) / np.linalg.norm(v), -1.0, 1.0)))


def scenes(n, seed=0):
    """Cube as the provided randomize_object() draws it (r 0.18-0.30 m, azimuth -50° to 60°, yaw 0-90°),
    box in the area of /sim/randomize (r 0.18-0.30 m, azimuth -90° to -10°, 03 §7)."""
    rng = np.random.default_rng(seed)
    for _ in range(n):
        r, a = rng.uniform(0.18, 0.30), np.radians(rng.uniform(-50.0, 60.0))
        rb, ab = rng.uniform(0.18, 0.30), np.radians(rng.uniform(-90.0, -10.0))
        cube = np.array([r * np.cos(a), r * np.sin(a), 0.025])
        yield cube, rng.uniform(0.0, np.pi / 2), np.array([rb * np.cos(ab), rb * np.sin(ab), 0.0])


def check(kin, plan, cube, yaw, box, backoff=0.04, lift_dz=0.10, drop_z=0.09, q_current=REST):
    """Every configuration of the plan, verified by FK."""
    grasp = kin.fk(plan.grasp)
    approach = grasp[:3, 2]
    assert np.linalg.norm(grasp[:3, 3] - cube) <= 1e-3
    assert abs(np.degrees(angle(approach, DOWN)) - plan.tilt_deg) <= 2.0
    outward = cube[:2] - [0.0388, 0.0]  # tilted in the arm plane, tool tip outwards (04 §4)
    assert plan.tilt_deg == 0 or np.dot(approach[:2], outward) > 0
    assert np.linalg.norm(kin.fk(plan.pre_grasp)[:3, 3] - (grasp[:3, 3] - backoff * approach)) <= 1e-3
    assert np.linalg.norm(kin.fk(plan.lift)[:3, 3] - (cube + lift_dz * UP)) <= 1e-3
    drop = kin.fk(plan.drop)
    assert np.linalg.norm(drop[:3, 3] - (box + drop_z * UP)) <= 1e-3
    # Gripper as vertical as reachable: tilted 62-68°, the cube stayed hooked on a jaw in its tunnel (2.4b).
    assert np.degrees(angle(drop[:3, 2], DOWN)) <= 32.0
    # The wrist roll only turns when the jaws need it: lift and drop keep it, a free grasp keeps the current.
    assert plan.lift[4] == pytest.approx(plan.pre_grasp[4])
    assert plan.drop[4] == pytest.approx(plan.lift[4])
    assert plan.aligned or plan.grasp[4] == pytest.approx(q_current[4])
    if plan.aligned:  # jaws (TCP x) along a face normal (yaw + k·90°) projected perpendicular to the approach
        normals = [
            np.array([np.cos(yaw + k * np.pi / 2), np.sin(yaw + k * np.pi / 2), 0.0]) for k in range(4)
        ]
        assert min(angle(grasp[:3, 0], n - approach * (n @ approach)) for n in normals) <= np.radians(2.0)


def test_plans_98_percent_of_200_scenes_under_1_s_p95(kin, planner):
    times, found, aligned, tilts, turns = [], 0, 0, [], []
    for cube, yaw, box in scenes(200):
        start = time.perf_counter()
        plan, _ = planner.plan(cube, yaw, box, REST)
        times.append(time.perf_counter() - start)
        if plan is not None:
            check(kin, plan, cube, yaw, box)
            found, aligned = found + 1, aligned + plan.aligned
            tilts.append(plan.tilt_deg)
            if plan.aligned:
                turns.append(abs(plan.grasp[4] - REST[4]))  # the face closest to the current roll
    per_tilt = dict(zip(*np.unique(tilts, return_counts=True), strict=True))
    print(
        f"plans: {found}/200 ({aligned} aligned), tilts {per_tilt}, "
        f"median {np.median(times) * 1000:.0f} ms, p95 {np.percentile(times, 95) * 1000:.0f} ms; "
        f"aligned wrist turn: median {np.degrees(np.median(turns)):.0f}°, "
        f"p95 {np.degrees(np.percentile(turns, 95)):.0f}°, max {np.degrees(max(turns)):.0f}°"
    )
    assert found >= 196
    assert np.percentile(times, 95) < 1.0
    assert np.degrees(np.median(turns)) <= 80.0  # the closest face (61°); the first one found: 100°


def test_a_cube_out_of_reach_gives_no_plan_and_the_reason(planner):
    assert planner.plan(np.array([0.5, 0.0, 0.025]), 0.0, BOX, REST) == (None, "unreachable")


def test_a_cube_out_of_reach_fails_within_half_the_plan_timeout(planner):
    """The slowest answer: every tilt and face fails (1.7 s, 2.4 s on the CI runner). Within PLAN's
    timeout with a 2x margin, else the brain reports ERROR:timeout:PLAN, not ERROR:unreachable (06 §3.1)."""
    park = np.array(Params().park_pose)  # q_current in PLAN
    start = time.perf_counter()
    assert planner.plan(np.array([0.5, 0.0, 0.025]), 0.3, BOX, park) == (None, "unreachable")
    assert time.perf_counter() - start < Params().plan_timeout_s / 2


def test_a_box_out_of_reach_gives_no_plan_and_the_reason(planner):
    assert planner.plan(np.array([0.22, 0.05, 0.025]), 0.0, np.array([0.5, 0.0, 0.0]), REST) == (
        None,
        "drop_unreachable",
    )


def test_an_unknown_yaw_gives_a_free_grasp(kin, planner):
    cube = np.array([0.22, 0.05, 0.025])
    plan, reason = planner.plan(cube, None, BOX, REST)
    assert reason is None
    assert not plan.aligned
    check(kin, plan, cube, None, BOX)


def test_every_parameter_is_injected(kin):
    planner = GraspPlanner(kin, tilts_deg=(30.0,), pregrasp_backoff_m=0.06, lift_dz_m=0.05, drop_tcp_z_m=0.12)
    cube = np.array([0.22, 0.05, 0.025])
    plan, _ = planner.plan(cube, 0.3, BOX, REST)
    assert plan.tilt_deg == 30.0
    check(kin, plan, cube, 0.3, BOX, backoff=0.06, lift_dz=0.05, drop_z=0.12)


def test_the_jaws_close_along_the_face_normal():
    """The cube's 90° symmetry hides a jaw axis off by 90°: the convention is checked here (TCP x = jaws)."""
    R = aligned_orientation(DOWN, 0.3)
    np.testing.assert_allclose(R[:, 0], [np.cos(0.3), np.sin(0.3), 0.0], atol=1e-12)
    np.testing.assert_allclose(R[:, 2], DOWN)
    assert np.linalg.det(R) == pytest.approx(1.0)  # a rotation, right-handed
