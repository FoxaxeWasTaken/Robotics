"""IK timings, a nightly trend (#60, 08 §2): measured, no threshold (CI runners are too noisy for a gate)."""

from pathlib import Path

import pytest
from ament_index_python.packages import get_package_share_directory

from so101_brain.kinematics import Kinematics

URDF = Path(get_package_share_directory("so101_description")) / "urdf" / "so101.urdf"
Q = [0.2, 0.3, 0.6, 1.0, 0.0]  # a bent pose over the work area: its FK pose is reachable by construction


@pytest.fixture(scope="module")
def kin():
    return Kinematics(str(URDF))


def test_ik_pos(benchmark, kin):
    assert benchmark(kin.ik_pos, kin.fk(Q)[:3, 3]) is not None


def test_ik_pose(benchmark, kin):
    T = kin.fk(Q)
    assert benchmark(kin.ik_pose, T[:3, 3], T[:3, :3]) is not None
