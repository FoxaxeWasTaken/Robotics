"""The planning worker of brain_node (06 §4): its own process and GraspPlanner, the plan back intact."""

import os
from pathlib import Path

import numpy as np
from ament_index_python.packages import get_package_share_directory

from so101_brain.brain_node import planner_pool, timed_plan
from so101_brain.fsm import Params
from so101_brain.grasp_planner import GraspPlanner
from so101_brain.kinematics import Kinematics

URDF = str(Path(get_package_share_directory("so101_description")) / "urdf" / "so101.urdf")
CUBE, BOX = np.array([0.22, 0.05, 0.025]), np.array([0.05, -0.25, 0.0])


def test_the_planner_runs_in_its_own_process_and_its_plan_comes_back():
    pool = planner_pool(URDF, {"lift_dz_m": 0.12})
    try:
        assert pool.submit(os.getpid).result(timeout=30) != os.getpid()  # no GIL shared with the tick
        request = (CUBE, 0.3, BOX, np.array(Params().park_pose))
        (plan, reason), seconds = pool.submit(timed_plan, *request).result(timeout=30)
    finally:
        pool.shutdown()
    expected, _ = GraspPlanner(Kinematics(URDF), lift_dz_m=0.12).plan(*request)
    assert reason is None
    assert seconds > 0.0
    for name in ("pre_grasp", "grasp", "lift", "drop"):
        np.testing.assert_allclose(getattr(plan, name), getattr(expected, name))
