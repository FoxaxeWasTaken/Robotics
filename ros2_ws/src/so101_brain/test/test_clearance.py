"""How near a box the arm passes (clearance.py, #260): the arm as points on the hulls of the URDF's collision
meshes, their distance to boxes given in a frame, and the smallest one along the joint lines of a move's
steps. No ROS, no sim: the model is checked against MuJoCo in so101_bringup (test_connect4_clearance.py)."""

from pathlib import Path

import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory

from so101_brain import clearance
from so101_brain.drop import Step
from so101_brain.kinematics import Kinematics

URDF = Path(get_package_share_directory("so101_description")) / "urdf" / "so101.urdf"
BENT = np.array([0.0, -0.4, 0.6, 0.8, 0.0])  # the arm ahead of its base, along +x


@pytest.fixture(scope="module")
def kin():
    return Kinematics(str(URDF))


@pytest.fixture(scope="module")
def arm():
    return clearance.Arm(URDF)


def box(centre, half) -> tuple:
    """(the frame of a box centred there, its corners in that frame)."""
    T = np.eye(4)
    T[:3, 3] = centre
    return T, np.array([[-np.asarray(half, dtype=float), np.asarray(half, dtype=float)]])


def test_the_distance_of_a_point_to_a_box_is_to_its_face_its_corner_or_negative_inside():
    T, boxes = box([0.0, 0.0, 0.0], [0.01, 0.01, 0.01])
    assert clearance.distance(np.array([[0.05, 0.0, 0.0]]), T, boxes) == pytest.approx(0.04)
    assert clearance.distance(np.array([[0.04, 0.05, 0.0]]), T, boxes) == pytest.approx(0.05)  # an edge
    assert clearance.distance(np.array([[0.0, 0.0, 0.004]]), T, boxes) == pytest.approx(-0.006)
    assert clearance.distance(np.array([[0.05, 0.0, 0.0], [0.0, 0.03, 0.0]]), T, boxes) == pytest.approx(0.02)


def test_the_boxes_are_given_in_a_frame_that_stands_anywhere():
    T, boxes = box([0.2, 0.1, 0.0], [0.01, 0.01, 0.01])
    turn = np.radians(90.0)
    T[:3, :3] = [[np.cos(turn), -np.sin(turn), 0], [np.sin(turn), np.cos(turn), 0], [0, 0, 1]]
    boxes[0, 1, 0] = 0.05  # 5 cm long along the frame's x: base_link's y
    assert clearance.distance(np.array([[0.2, 0.2, 0.0]]), T, boxes) == pytest.approx(0.05)
    two = np.array([*boxes, [[0.0, -0.01, 0.1], [0.01, 0.01, 0.2]]])  # the nearest of several boxes
    assert clearance.distance(np.array([[0.2, 0.1, 0.25]]), T, two) == pytest.approx(0.05)


def test_the_arms_points_cover_every_moving_link_from_the_shoulder_to_both_jaws(kin, arm):
    points = arm.points(kin, BENT, 100.0)
    frames = kin.link_frames(BENT)
    assert len(frames) == 5
    for frame in (*frames, kin.fk(BENT)):  # each link's own frame, and the TCP between the jaws
        assert np.linalg.norm(points - frame[:3, 3], axis=1).min() < 0.03
    assert 1000 < len(points) < 6000  # every 8 mm of the hulls: enough for a millimetre, cheap enough
    assert points[:, 2].min() > 0.0  # the base, which never moves, is not in it


def test_a_box_the_lower_arm_goes_through_is_inside_and_one_ahead_of_the_jaws_is_as_far_as_it_stands(
    kin, arm
):
    frames = kin.link_frames(BENT)
    middle = (frames[2][:3, 3] + frames[3][:3, 3]) / 2  # between the elbow and the wrist
    assert clearance.distance(arm.points(kin, BENT, 0.0), *box(middle, [0.03, 0.03, 0.03])) < -0.005
    reach = arm.points(kin, BENT, 0.0)[:, 0].max()
    for gap in (0.05, 0.04):  # a wall ahead of the arm: the distance is the gap, to the millimetre
        wall = box([reach + gap + 0.01, 0.0, 0.2], [0.01, 0.5, 0.5])
        assert clearance.distance(arm.points(kin, BENT, 0.0), *wall) == pytest.approx(gap, abs=1e-9)


def test_the_moving_jaw_follows_the_opening(kin, arm):
    shut, wide = arm.points(kin, BENT, 0.0), arm.points(kin, BENT, 100.0)
    apart = np.linalg.norm(wide[:, None, :] - shut[None, :, :], axis=2).min(axis=1)
    assert apart.max() > 0.03  # the open jaw's tip is 3 cm from anything of the closed gripper
    assert np.isclose(apart, 0.0).sum() > 1000  # the rest of the arm did not move


def test_the_moving_jaw_shuts_beside_the_tool_point_and_swings_11_cm_aside_when_open(kin, arm):
    """In the tool's frame the jaws close along x: the fixed jaw on +x, the moving one on -x."""
    tcp = kin.fk(BENT)

    def in_tool(opening):
        return (arm.points(kin, BENT, opening) - tcp[:3, 3]) @ tcp[:3, :3]

    shut, wide = in_tool(0.0), in_tool(100.0)
    for points, nearest, furthest in (
        (shut, (-0.025, -0.018), (-0.045, -0.030)),
        (wide, (-0.015, -0.008), (-0.12, -0.10)),
    ):
        near = points[np.linalg.norm(points, axis=1) < 0.03]
        assert nearest[0] < near[:, 0].min() < nearest[1]  # what stands on -x within 3 cm of the tool point
        assert furthest[0] < points[:, 0].min() < furthest[1]  # the moving jaw's far side, or its tip
        assert 0.012 < near[:, 0].max() < 0.020  # the fixed jaw, which does not move
    assert np.abs(shut[:, 1]).max() < 0.06  # nothing of the arm far off its own plane, the jaw neither
    jaw = np.linalg.norm(wide - shut, axis=1) > 1e-9  # the points that moved with the opening: the jaw's
    extents = {  # of the jaw in the tool's frame, m: (its low corner, its high corner)
        "shut": ((-0.038, -0.024, -0.084), (-0.002, 0.024, 0.004)),
        "wide": ((-0.111, -0.024, -0.089), (-0.018, 0.024, -0.065)),
    }
    for name, points in (("shut", shut[jaw]), ("wide", wide[jaw])):
        low, high = extents[name]
        assert np.allclose(points.min(axis=0), low, atol=0.003), (name, points.min(axis=0))
        assert np.allclose(points.max(axis=0), high, atol=0.003), (name, points.max(axis=0))


def test_a_steps_distance_is_the_smallest_over_its_line_every_3_hundredths_of_a_radian(kin, arm):
    start, goal = np.array([0.25, -0.4, 0.6, 0.8, 0.0]), np.array([0.6, -0.4, 0.6, 0.8, 0.0])
    reach = arm.points(kin, goal, 0.0)[:, 1].max()  # the pan alone turns the arm towards +y
    post = box([0.2, reach + 0.03, 0.2], [0.5, 0.01, 0.5])  # a wall 2 cm past the goal: nearest at its end
    count = int(np.ceil(0.35 / clearance.STEP))  # 12 poses for the 0.35 rad the pan turns
    on_the_line = [
        clearance.distance(arm.points(kin, start + (goal - start) * k / count, 0.0), *post)
        for k in range(1, count + 1)
    ]
    assert on_the_line[-1] == min(on_the_line) > 0.0  # the goal itself is judged, and nothing beyond it
    ((name, gap),) = clearance.along(arm, kin, start, [Step("turn", q=goal)], 0.0, *post)
    assert (name, gap) == ("turn", pytest.approx(min(on_the_line), abs=1e-12))
    beyond = clearance.distance(arm.points(kin, start + (goal - start) * (count + 1) / count, 0.0), *post)
    assert beyond < gap  # one pose further would have said nearer
    # a post the jaws pass on the way: the smallest of those 12 poses, not of a finer or a coarser line
    tcp = kin.fk((start + goal) / 2)[:3, 3]
    passed = box([tcp[0] + 0.05, tcp[1], tcp[2]], [0.005, 0.005, 0.2])
    on_the_line = [
        clearance.distance(arm.points(kin, start + (goal - start) * k / count, 0.0), *passed)
        for k in range(1, count + 1)
    ]
    assert 0 < np.argmin(on_the_line) < count - 1  # nearest in between
    ((_, gap),) = clearance.along(arm, kin, start, [Step("turn", q=goal)], 0.0, *passed)
    assert gap == pytest.approx(min(on_the_line), abs=1e-12)
    finer = min(
        clearance.distance(arm.points(kin, start + (goal - start) * k / 100, 0.0), *passed)
        for k in range(101)
    )
    assert finer < gap - 1e-6


def test_a_move_is_judged_along_its_joint_lines_not_at_its_ends(kin, arm):
    left, right = np.array([0.6, -0.4, 0.6, 0.8, 0.0]), np.array([-0.6, -0.4, 0.6, 0.8, 0.0])
    tcp = kin.fk(BENT)[:3, 3]
    post = box([tcp[0], 0.0, tcp[2]], [0.01, 0.01, 0.2])  # where the jaws pass half-way
    for q in (left, right):
        assert clearance.distance(arm.points(kin, q, 0.0), *post) > 0.05
    steps = [Step("up", q=left + np.array([0, -0.1, 0, 0, 0])), Step("back", q=left), Step("turn", q=right)]
    gaps = dict(clearance.along(arm, kin, left, steps, 0.0, *post))
    assert list(gaps) == ["up", "back", "turn"]  # each step's own smallest distance, in their order
    assert gaps["turn"] < 0.0
    assert min(gaps["up"], gaps["back"]) > 0.05


def test_a_step_that_opens_the_jaws_is_judged_with_them_open(kin, arm):
    wide = arm.points(kin, BENT, 100.0)
    shut = arm.points(kin, BENT, 0.0)
    apart = np.linalg.norm(wide[:, None, :] - shut[None, :, :], axis=2).min(axis=1)
    tip = wide[np.argmax(apart)]  # the open jaw's tip
    beside = box(tip, [0.004, 0.004, 0.004])
    still = [Step("wait", q=BENT.copy())]
    assert dict(clearance.along(arm, kin, BENT, still, 0.0, *beside))["wait"] > 0.01
    gaps = dict(clearance.along(arm, kin, BENT, [Step("open", gripper=100.0), *still], 0.0, *beside))
    # a step without an arm goal is judged where the arm stands, and the jaws stay as it left them
    assert gaps["open"] == pytest.approx(clearance.distance(wide, *beside))
    assert gaps["wait"] == gaps["open"] < 0.0
