"""How near a volume the arm passes on a move (#260, docs/project/phase2/P2-02 §2.5). Pure, no ROS.

ikpy knows no volumes (kinematics.self_collides is a calibrated cylinder around the shoulder). Here the arm is
a cloud of points on the convex hulls of the URDF's collision meshes (hulls.py), the hulls MuJoCo collides
with: within 1.4 mm of MuJoCo's own distance on 99 % of 68,000 poses, 3.6 mm at most [MES-78]. The volume is
a few boxes in a frame, the grid's from the pose the camera measures and its own dimensions. A move is judged
along the joint lines of its steps, not at their ends: the line to PARK crosses the grid's post [MES-69].
"""

import numpy as np

from so101_brain import hulls
from so101_common.robot import pct_to_rad

SPACING = 0.008  # m between two points of a hull: a box's corner between them is seen 1 mm late at most
STEP = 0.03  # rad between two poses judged on a joint line: 12 mm at the jaws, arm stretched
# m: a move that passes nearer the grid is refused. Above the 3.6 mm the model may say too much, under the
# 11 mm a game's moves keep [MES-78]
MARGIN = 0.005


def distance(points, T, boxes) -> float:
    """m between the nearest of `points` (base_link) and the nearest of `boxes`, each (low corner, high
    corner) in the frame T; negative inside one, by its depth under the nearest face."""
    local = (np.asarray(points) - T[:3, 3]) @ T[:3, :3]
    best = np.inf
    for low, high in boxes:
        out = np.maximum(low - local, local - high)  # per axis: how far outside, negative between the faces
        gaps = np.linalg.norm(np.maximum(out, 0.0), axis=1) + np.minimum(out.max(axis=1), 0.0)
        best = min(best, float(gaps.min()))
    return best


def along(arm, kin, q, steps, gripper, T, boxes):
    """Yields (the step's name, m) for each step of a move from the joints `q`: the smallest distance to the
    boxes on its joint line, judged every STEP, the jaws at `gripper` % until a step commands them. A step
    without an arm goal is judged where the arm stands."""
    q = np.asarray(q, dtype=float)
    for step in steps:
        gripper = gripper if step.gripper is None else step.gripper
        goal = q if step.q is None else np.asarray(step.q, dtype=float)
        count = max(1, int(np.ceil(np.max(np.abs(goal - q)) / STEP)))
        yield (
            step.name,
            min(
                distance(arm.points(kin, q + (goal - q) * k / count, gripper), T, boxes)
                for k in range(1, count + 1)
            ),
        )
        q = goal


class Arm:
    """The arm's moving links as points, each link's in its own frame: the URDF's collision meshes
    (hulls.py)."""

    def __init__(self, urdf_path, spacing=SPACING):
        self._links, self._jaw, self._jaw_joint = hulls.load(urdf_path, spacing)

    def points(self, kin, q, gripper) -> np.ndarray:
        """Every point in base_link, the arm at the joints `q` and the jaws open at `gripper` %."""
        frames = kin.link_frames(q)
        c, s = np.cos(pct_to_rad(gripper)), np.sin(pct_to_rad(gripper))
        jaw = frames[-1] @ self._jaw_joint @ [[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
        clouds = zip([*frames, jaw], [*self._links, self._jaw], strict=True)
        return np.vstack([points @ frame[:3, :3].T + frame[:3, 3] for frame, points in clouds])
