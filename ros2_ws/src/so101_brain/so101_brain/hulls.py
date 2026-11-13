"""hulls.py - the arm's links as clouds of points, from the URDF's collision meshes (clearance.py, #260).
Pure, no ROS.

Each collision mesh of a link becomes points every `spacing` on its convex hull, the hull MuJoCo collides
with, in the link's own frame. Apart from clearance.py so that mutation testing covers that one alone: a
mutant of the mesh loading builds clouds of gigabytes (docs/project/08 §1); the clouds are checked against
MuJoCo's own distances in so101_bringup (test_connect4_clearance.py) [MES-78].
"""

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull

from so101_common.robot import ARM_JOINT_NAMES

STL = np.dtype([("normal", "<f4", 3), ("corners", "<f4", (3, 3)), ("attribute", "<u2")])  # a binary STL facet


def load(urdf_path, spacing) -> tuple:
    """(the points of the 5 arm links, shoulder to gripper; the moving jaw's; the jaw joint's 4x4 origin in
    the gripper link) from the URDF and the meshes installed beside it."""
    root = ET.parse(urdf_path).getroot()
    meshes = Path(urdf_path).parent.parent / "meshes"
    child = {joint.get("name"): joint.find("child").get("link") for joint in root.findall("joint")}
    clouds = {
        link.get("name"): np.vstack([_hull(part, meshes, spacing) for part in link.findall("collision")])
        for link in root.findall("link")
        if link.find("collision") is not None
    }
    jaw = root.find("joint[@name='gripper']")
    if jaw.find("axis").get("xyz").split() != ["0", "0", "1"]:
        raise ValueError(f"{urdf_path}: the gripper joint is expected to turn about its z")
    return (
        [clouds[child[name]] for name in ARM_JOINT_NAMES],
        clouds[child["gripper"]],
        _origin(jaw.find("origin")),
    )


def _origin(node) -> np.ndarray:
    """The 4x4 transform of a URDF <origin>: xyz, then roll, pitch, yaw about the fixed axes."""
    T = np.eye(4)
    if node is not None:
        T[:3, 3] = [float(v) for v in node.get("xyz", "0 0 0").split()]
        (cr, sr), (cp, sp), (cy, sy) = (
            (np.cos(a), np.sin(a)) for a in map(float, node.get("rpy", "0 0 0").split())
        )
        T[:3, :3] = [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    return T


def _hull(collision, meshes, spacing) -> np.ndarray:
    """Points every `spacing` on the convex hull of one collision mesh, in its link's frame."""
    name = collision.find("geometry/mesh").get("filename").rsplit("/", 1)[-1]
    facets = np.frombuffer((meshes / name).read_bytes(), dtype=STL, offset=84)  # 80 of header, the count
    T = _origin(collision.find("origin"))
    vertices = facets["corners"].reshape(-1, 3).astype(float) @ T[:3, :3].T + T[:3, 3]
    hull = ConvexHull(vertices)
    points = [vertices[hull.vertices]]
    for a, b, c in vertices[hull.simplices]:  # a large flat face has no vertex inside: a lattice on it
        n = int(np.ceil(max(np.linalg.norm(b - a), np.linalg.norm(c - a), np.linalg.norm(c - b)) / spacing))
        if n > 1:
            i, j = np.meshgrid(np.arange(n + 1), np.arange(n + 1))
            inside = i + j <= n
            points.append(a + np.outer(i[inside] / n, b - a) + np.outer(j[inside] / n, c - a))
    points = np.vstack(points)
    _, kept = np.unique(np.round(points / spacing).astype(int), axis=0, return_index=True)  # one per cell
    return points[kept]
