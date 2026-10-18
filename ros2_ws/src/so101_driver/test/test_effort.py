"""Normalized effort from the sim (docs/project/03 §5, ADR-016): the grasp signal. SO101Sim in-process."""

import mujoco
import numpy as np
import pytest
from so101_sim import SO101Sim

from so101_common.robot import JOINT_NAMES
from so101_driver.effort import normalized_effort

TUNNEL_YAW = [np.cos(np.pi / 4), 0.0, 0.0, np.sin(np.pi / 4)]  # cube turned 90°: the jaws enter a tunnel


@pytest.fixture
def sim():
    s = SO101Sim()
    s.connect()
    yield s
    s.disconnect()


def drive(sim, gripper_pct, seconds):
    """Holds the arm still and drives the gripper, 20 ms of sim time per tick as in the driver."""
    action = sim.get_observation() | {"gripper.pos": gripper_pct}
    for _ in range(round(seconds / 0.02)):
        sim.send_action(action)
        sim.get_observation()


def test_one_bounded_value_per_joint(sim):
    effort = normalized_effort(sim.model, sim.data, JOINT_NAMES)
    assert len(effort) == len(JOINT_NAMES)
    assert all(-1.0 <= e <= 1.0 for e in effort)


def test_actuators_are_found_by_name(sim):
    drive(sim, 0.0, 0.5)
    effort = dict(zip(JOINT_NAMES, normalized_effort(sim.model, sim.data, JOINT_NAMES), strict=True))
    reordered = list(reversed(JOINT_NAMES))
    assert normalized_effort(sim.model, sim.data, reordered) == [effort[n] for n in reordered]


def test_gripper_closing_on_nothing_has_no_load(sim):
    drive(sim, 0.0, 1.0)
    gripper = normalized_effort(sim.model, sim.data, ["gripper"])[0]
    print(f"gripper effort, closed on nothing: {gripper:.3f}")
    assert abs(gripper) < 0.02


def test_gripper_holding_the_cube_reports_a_positive_load(sim):
    drive(sim, 100.0, 1.0)
    sim.model.opt.gravity[:] = 0  # test only: the cube stays between the open jaws while they close
    cube = sim.model.joint("pickup_object_free").qposadr[0]
    sim.data.qpos[cube : cube + 3] = sim.data.site("gripperframe").xpos  # test-only qpos write
    sim.data.qpos[cube + 3 : cube + 7] = TUNNEL_YAW
    sim.data.qvel[:] = 0
    mujoco.mj_forward(sim.model, sim.data)
    drive(sim, 0.0, 1.0)
    gripper = normalized_effort(sim.model, sim.data, ["gripper"])[0]
    print(f"gripper effort, holding the cube: {gripper:.3f} (MES-10: 0.12-0.19)")
    assert gripper > 0.06  # the brain's grasp threshold (06 §3)
