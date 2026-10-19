"""FK of so101_brain against the MuJoCo model. Sim-backed, so here and not in so101_brain (C-05 guard)."""

from pathlib import Path

import mujoco
import numpy as np
from ament_index_python.packages import get_package_share_directory
from so101_sim import SO101Sim

from so101_brain.kinematics import Kinematics
from so101_common.robot import ARM_JOINT_NAMES, JOINT_LIMITS

URDF = Path(get_package_share_directory("so101_description")) / "urdf" / "so101.urdf"


def test_fk_matches_the_mujoco_gripperframe_site():
    sim = SO101Sim()
    sim.connect()
    kin = Kinematics(str(URDF))
    rng = np.random.default_rng(0)
    worst = 0.0
    for _ in range(50):
        q = np.array([rng.uniform(*JOINT_LIMITS[name]) for name in ARM_JOINT_NAMES])
        for name, value in zip(ARM_JOINT_NAMES, q, strict=True):
            sim.data.qpos[sim.model.joint(name).qposadr[0]] = value
        mujoco.mj_forward(sim.model, sim.data)
        # Positions only: the MuJoCo site is rotated 90° from the URDF TCP frame (04 §1.1).
        worst = max(worst, float(np.linalg.norm(kin.fk(q)[:3, 3] - sim.data.site("gripperframe").xpos)))
    print(f"worst FK vs MuJoCo gripperframe over 50 configurations: {worst * 1000:.4f} mm")
    assert worst < 1e-5  # 0.01 mm (MES-04: 0.004 mm)
