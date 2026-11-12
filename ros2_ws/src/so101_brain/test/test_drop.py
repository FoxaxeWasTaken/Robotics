"""The drop pose of phase 2 (drop.py, phase2/P2-02 §2): on the 7 columns of the layout of P2-02 §7, the held
token's centre on the target, the jaws normal to the grid, the approach tilted 70°. No ROS, no sim."""

import time
from pathlib import Path

import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory

from so101_brain.drop import ROLL, TILT_DEG, TOKEN_IN_TCP, release_pose
from so101_brain.kinematics import PAN_AXIS_X, Kinematics

URDF = Path(get_package_share_directory("so101_description")) / "urdf" / "so101.urdf"
# The layout of P2-02 §7 (so101_connect4/config/connect4_grid.yaml): azimuth -40°, columns 0.14 m + 37 mm
# apart, slot entries 25 cm above a table 10 cm below base_link, a 30 mm depth guide; the token's bottom 8 mm
# above the guide (P2-02 §2.3). Across the grid, the token centre 4.85 mm off the pan axis on the side of the
# TCP line (MES-64: the TCP 8 mm off the axis, the token 3.15 mm from it), as e7_cycle.py dropped 42 of 42.
AZIMUTH = np.radians(-40.0)
ALONG = np.array([np.cos(AZIMUTH), np.sin(AZIMUTH), 0.0])
NORMAL = np.array([-np.sin(AZIMUTH), np.cos(AZIMUTH), 0.0])  # the grid's normal
TOKEN_Z = -0.10 + 0.25 + 0.030 + 0.008 + 0.017  # table, entries, guide, clearance, token radius
COLUMNS = [
    np.array([PAN_AXIS_X, 0.0, TOKEN_Z]) + (0.14 + 0.037 * c) * ALONG - 0.00485 * NORMAL for c in range(7)
]


class CountingKinematics(Kinematics):
    """Counts the solutions _solve rejects as folded onto the arm."""

    rejected = 0

    def self_collides(self, q):
        folded = super().self_collides(q)
        self.rejected += folded
        return folded


@pytest.fixture(scope="module")
def kin():
    return CountingKinematics(str(URDF))


def token_centre(kin, q):
    pose = kin.fk(q)
    return pose[:3, 3] + pose[:3, :3] @ TOKEN_IN_TCP


@pytest.mark.parametrize("column", range(7))
def test_each_column_gets_the_token_on_target_jaws_normal_to_the_grid_tilted_70(kin, column):
    kin.rejected = 0
    start = time.perf_counter()
    q = release_pose(kin, COLUMNS[column])
    elapsed = time.perf_counter() - start
    print(f"column {column}: {elapsed * 1000:.0f} ms")  # 10 to 36 ms in MES-64
    assert q is not None
    pose = kin.fk(q)
    assert np.linalg.norm(token_centre(kin, q) - COLUMNS[column]) < 1e-3
    jaw_error = np.degrees(np.arccos(min(1.0, abs(pose[:3, 0] @ NORMAL))))
    assert jaw_error < 0.2
    tilt = np.degrees(np.arccos(np.clip(-pose[2, 2], -1.0, 1.0)))  # bare ik_pos flips the tool: 114-149°
    assert tilt == pytest.approx(TILT_DEG, abs=2.0)
    assert q[4] == pytest.approx(ROLL)
    assert kin.rejected == 0


def test_the_seed_does_not_change_the_pose(kin):
    """From HANDOVER's candidate (P2-02 §7) as from the default bent seed: the same joints."""
    handover = np.array([-1.0, -0.6, 0.6, 1.1, 1.57])
    for target in (COLUMNS[0], COLUMNS[6]):
        np.testing.assert_allclose(release_pose(kin, target, handover), release_pose(kin, target), atol=0.02)


@pytest.mark.parametrize(
    "target", [(0.7, 0.0, 0.2), (PAN_AXIS_X + 0.02, 0.0, 0.05)], ids=["far", "on-the-base"]
)
def test_an_unreachable_target_gives_none(kin, target):
    assert release_pose(kin, np.array(target)) is None
