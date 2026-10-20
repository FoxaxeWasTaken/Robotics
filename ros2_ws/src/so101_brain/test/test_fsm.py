"""Unit tests of the pick & place state machine (docs/project/06 §3.1, ADR-015). Pure: a fake clock, arm,
camera and planner; no ROS, no sim. One test per row of 06 §3.1, /brain/stop from every state."""

import re

import numpy as np
import pytest

from so101_brain.fsm import Detection, Fsm, Inputs, Params, State
from so101_brain.grasp_planner import GraspPlan
from so101_brain.trajectory import min_jerk

DT = 0.02  # s: the 50 Hz tick
EPS = 1e-6
CUBE = np.array([0.22, 0.05, 0.025])
BOX = np.array([0.05, -0.25, 0.0])
IN_BOX = BOX + np.array([0.01, -0.02, 0.025])
PARK = np.array(Params().park_pose)
PLAN = GraspPlan(np.full(5, 0.1), np.full(5, 0.2), np.full(5, 0.3), np.full(5, 0.4), 10.0, True)
HOLDING, EMPTY = 0.15, 0.0  # normalized gripper effort, cube in the jaws or not [MES-35]
GOALS = {  # first goal of each motion state
    "WAIT_TARGET": PARK,
    "APPROACH": PLAN.pre_grasp,
    "DESCEND": PLAN.grasp,
    "LIFT": PLAN.pre_grasp,
    "TRANSPORT": PLAN.drop,
    "HOME": PARK,
    "HOME_RETRY": PARK,
}


class World:
    """Fake arm and gripper (follow every command at once, unless blocked), camera (10 Hz, frames alternately
    offset by `jitter` along x) and planner (answers at the next tick)."""

    def __init__(self, **params):
        self.fsm = Fsm(Params(**params))
        self.now = 0.0
        self.q = np.zeros(5)
        self.effort = HOLDING
        self.cube, self.box = CUBE, BOX  # where the camera sees them; None: out of view
        self.jitter = 0.0
        self.joints = True  # the driver publishes /joint_states
        self.blocked = False
        self.gripper = 0.0  # measured, %; None: not in /joint_states
        self.gripper_blocked = False
        self.answer = (PLAN, None)  # the planner's answer; None: it never answers
        self.joints_received = -np.inf
        self.seen = {"cube": None, "box": None}  # last detection of each, as the node stores them
        self.frames = 0
        self.pending = None
        self.requests, self.commands, self.grippers, self.transitions = [], [], [], []
        self.command_states = []  # /brain_state when each command was sent
        self.reached = []  # (state left, q) at each transition

    def step(self, start=False, stop=False):
        if self.joints:
            self.joints_received = self.now
        if self.now >= self.frames * 0.1 - EPS:
            offset = np.array([self.jitter * (-1) ** self.frames, 0.0, 0.0])
            for name, yaw in (("cube", 0.3 + 0.001 * self.frames), ("box", None)):
                if getattr(self, name) is not None:
                    self.seen[name] = Detection(getattr(self, name) + offset, self.now, yaw)
            self.frames += 1
        before = self.fsm.label
        out = self.fsm.step(
            self.now,
            Inputs(
                q=self.q.copy(),
                joints_received=self.joints_received,
                effort=self.effort,
                cube=self.seen["cube"],
                box=self.seen["box"],
                start=start,
                stop=stop,
                plan=self.pending,
                gripper=self.gripper,
            ),
        )
        assert out.state == self.fsm.label
        if out.plan_request is not None:
            self.requests.append(out.plan_request)
            self.pending = self.answer
        elif self.fsm.state is not State.PLAN:
            self.pending = None
        if out.command is not None:
            self.commands.append(out.command.copy())
            self.command_states.append(out.state)
            if not self.blocked:
                self.q = out.command.copy()
        if out.gripper is not None:
            self.grippers.append((out.state, out.gripper))
            if not self.gripper_blocked:
                self.gripper = out.gripper
        if out.transition is not None:
            self.transitions.append(out.transition)
            self.reached.append((before, self.q.copy()))
        self.now += DT
        return out

    def run_until(self, label, within=60.0):
        """Steps until /brain_state is `label`; returns the time since the state before it was entered, s."""
        start = self.now
        while self.fsm.label != label:
            assert self.now - start < within, f"no {label} in {within} s: {self.transitions[-3:]}"
            self.step()
        return self.now - start

    def states(self):
        return [t.split(" → ")[1].split(" (")[0] for t in self.transitions]


def reach(label, **params):
    """A world whose FSM has just entered `label` (ERROR: ERROR:unreachable)."""
    world = World(**params)
    if label == "STOPPED":
        world.step(stop=True)
        return world
    if label == "IDLE":
        return world
    if label == "ERROR":
        world.answer, label = (None, "unreachable"), "ERROR:unreachable"
    if label in ("RETRY", "HOME_RETRY"):
        world.effort = EMPTY  # the grasp fails
    world.step(start=True)
    if label == "DONE":
        world.run_until("RELEASE")
        world.cube = IN_BOX
    world.run_until(label)
    return world


def near(q, goal):
    """Reached: within the reach tolerance, so not exactly at the goal."""
    np.testing.assert_allclose(q, goal, atol=1e-6)


def duration(q_start, goal):
    return min_jerk(q_start, goal, Params().v_max, Params().min_motion_s).duration


# --- IDLE ---


def test_idle_waits_for_start_and_commands_nothing():
    world = World()
    for _ in range(100):
        world.step()
    assert world.fsm.label == "IDLE"
    assert not world.commands and not world.grippers
    world.step(start=True)
    assert world.fsm.label == "WAIT_TARGET"


def test_start_without_joint_states_is_an_error():
    world = World()
    world.joints = False
    world.step(start=True)
    assert world.fsm.label == "ERROR:no_joint_states"
    assert not world.commands


def test_start_is_ignored_while_busy_and_the_fsm_says_so():
    world = reach("APPROACH")
    assert not world.fsm.accepts_start
    world.step(start=True)
    assert world.fsm.label == "APPROACH"
    assert World().fsm.accepts_start and reach("ERROR").fsm.accepts_start


# --- WAIT_TARGET ---


def test_wait_target_parks_then_locks_the_mean_of_5_consistent_detections():
    world = World()
    world.jitter = 0.002  # frames 4 mm apart
    world.step(start=True)
    world.run_until("PLAN")
    np.testing.assert_allclose(world.commands[0], np.zeros(5))  # from the measured joints
    near(world.reached[-1][1], PARK)
    [(cube, yaw, box, q)] = world.requests
    offsets = [world.jitter * (-1) ** k for k in range(world.frames - 5, world.frames)]
    np.testing.assert_allclose(cube, CUBE + np.array([np.mean(offsets), 0.0, 0.0]), atol=1e-12)
    np.testing.assert_allclose(box, BOX + np.array([np.mean(offsets), 0.0, 0.0]), atol=1e-12)
    assert yaw == pytest.approx(0.3 + 0.001 * (world.frames - 1))  # the newest
    np.testing.assert_allclose(q, PARK)  # the measured joints: the planner's first seed


def test_detections_during_the_park_motion_do_not_count():
    world = World()
    world.cube = CUBE + np.array([0.003, 0.0, 0.0])  # within the spread: only the filter keeps it out
    world.step(start=True)
    while not np.allclose(world.q, PARK):
        world.step()
    world.cube = None  # out of view a moment: the last detection received stays the one taken in motion
    for _ in range(10):
        world.step()
    world.cube = CUBE
    world.run_until("PLAN")
    np.testing.assert_allclose(world.requests[0][0], CUBE)


def test_detections_more_than_5_mm_apart_never_lock():
    world = World(wait_target_s=3.0)
    world.jitter = 0.003  # frames 6 mm apart
    world.step(start=True)
    elapsed = world.run_until("ERROR:object_lost")
    assert abs(elapsed - (duration(np.zeros(5), PARK) + 3.0)) <= 2 * DT
    assert not world.requests


def test_a_cube_that_left_the_view_is_not_locked():
    world = World(wait_target_s=3.0)
    world.box = None
    world.step(start=True)
    while not np.allclose(world.q, PARK):
        world.step()
    for _ in range(40):  # 0.8 s: 8 detections of the cube, none of the box yet
        world.step()
    world.cube = None
    for _ in range(25):  # out of view for 0.5 s
        world.step()
    world.box = BOX
    world.run_until("ERROR:object_lost")
    assert not world.requests


def test_a_park_motion_that_never_ends_is_a_timeout():
    world = World()
    world.blocked = True
    world.step(start=True)
    elapsed = world.run_until("ERROR:timeout:WAIT_TARGET")
    assert abs(elapsed - (duration(np.zeros(5), PARK) + 1.0)) <= 2 * DT


# --- PLAN ---


def test_plan_asks_the_planner_once_then_approaches():
    world = reach("PLAN")
    world.run_until("APPROACH")
    assert len(world.requests) == 1


def test_no_plan_is_an_error_with_the_planner_reason_and_no_command():
    world = reach("PLAN")
    world.answer = (None, "drop_unreachable")
    world.pending = world.answer
    sent = len(world.commands)
    world.run_until("ERROR:drop_unreachable", within=DT)
    for _ in range(50):
        world.step()
    assert len(world.commands) == sent  # never "move anyway" (06 §10)


def test_a_silent_planner_is_a_timeout():
    world = World(plan_timeout_s=1.0)
    world.answer = None
    world.step(start=True)
    world.run_until("PLAN")
    assert abs(world.run_until("ERROR:timeout:PLAN") - 1.0) <= 2 * DT


# --- motions: APPROACH, DESCEND, LIFT, TRANSPORT, HOME ---


def test_approach_opens_the_gripper_and_starts_from_the_measured_joints():
    world = reach("PLAN")
    world.q = PARK + 0.05  # the arm lagged
    world.answer = world.pending = (PLAN, None)
    world.run_until("APPROACH")
    assert world.grippers[-1] == ("APPROACH", 100.0)
    np.testing.assert_allclose(world.commands[-1], PARK + 0.05)
    first = len(world.commands) - 1
    world.run_until("DESCEND")
    near(world.reached[-1][1], PLAN.pre_grasp)
    steps = np.abs(np.diff(world.commands[first:], axis=0)).max(axis=1)
    assert steps.max() / DT <= Params().v_max * 1.01


@pytest.mark.parametrize("opened, ends", [(50.0, False), (96.9, False), (97.1, True)])
def test_approach_also_waits_for_the_gripper_open_within_3_pct(opened, ends):
    world = reach("PLAN")
    world.gripper, world.gripper_blocked = opened, True
    world.run_until("APPROACH")
    for _ in range(int(duration(world.q, PLAN.pre_grasp) / DT) + 10):  # the arm reached, still in the margin
        world.step()
    assert (world.fsm.label == "DESCEND") is ends
    world.gripper_blocked = False
    world.gripper = 100.0
    world.run_until("DESCEND", within=0.1)


def test_a_gripper_that_does_not_open_is_a_timeout():
    world = reach("PLAN")
    world.gripper_blocked = True
    world.run_until("ERROR:timeout:APPROACH")
    assert "gripper 0 %" in world.transitions[-1]


def test_an_unknown_gripper_position_leaves_the_arm_alone_to_decide():
    world = reach("PLAN")
    world.gripper, world.gripper_blocked = None, True
    world.run_until("DESCEND")


def test_descend_moves_to_the_grasp():
    world = reach("DESCEND")
    world.run_until("GRASP")
    near(world.reached[-1][1], PLAN.grasp)


def test_lift_goes_back_to_the_pre_grasp_then_up():
    world = reach("LIFT")
    world.run_until("TRANSPORT")
    near(world.reached[-1][1], PLAN.lift)
    assert any(np.allclose(c, PLAN.pre_grasp) for c in world.commands)


def test_an_object_lost_during_the_lift_retries():
    world = reach("LIFT")
    world.effort = EMPTY
    world.run_until("RETRY")
    near(world.reached[-1][1], PLAN.lift)  # judged once lifted


def test_transport_moves_to_the_drop():
    world = reach("TRANSPORT")
    world.run_until("RELEASE")
    near(world.reached[-1][1], PLAN.drop)


def test_home_parks_then_verifies():
    world = reach("HOME")
    world.run_until("VERIFY")
    near(world.reached[-1][1], PARK)


@pytest.mark.parametrize("state", GOALS)
def test_a_motion_not_reached_at_T_plus_1_s_is_a_timeout_and_freezes_the_arm(state):
    world = reach(state)
    world.blocked = True
    entry = world.q.copy()
    elapsed = world.run_until(f"ERROR:timeout:{state}")
    assert abs(elapsed - (duration(entry, GOALS[state]) + 1.0)) <= 2 * DT
    np.testing.assert_allclose(world.commands[-1], entry)  # frozen: the measured joints


# --- GRASP ---


def test_grasp_closes_and_lifts_once_the_effort_holds_after_0_8_s():
    world = reach("GRASP")
    assert world.grippers[-1] == ("GRASP", 0.0)
    assert abs(world.run_until("LIFT") - 0.8) <= DT + EPS


def test_an_empty_gripper_at_1_5_s_retries():
    world = reach("GRASP")
    world.effort = EMPTY
    assert abs(world.run_until("RETRY") - 1.5) <= DT + EPS


def test_the_effort_threshold_is_injected():
    world = reach("GRASP", effort_threshold=0.2)  # HOLDING is 0.15
    world.run_until("RETRY", within=1.5 + DT)


# --- RELEASE, VERIFY, DONE ---


def test_release_opens_the_gripper_for_0_5_s():
    world = reach("RELEASE")
    assert world.grippers[-1] == ("RELEASE", 100.0)
    assert abs(world.run_until("HOME") - 0.5) <= DT + EPS


def test_a_cube_seen_in_the_box_is_done_then_idle():
    world = reach("DONE")
    world.step()
    assert world.fsm.label == "IDLE"
    assert world.states() == [
        "WAIT_TARGET", "PLAN", "APPROACH", "DESCEND", "GRASP", "LIFT", "TRANSPORT", "RELEASE", "HOME",
        "VERIFY", "DONE", "IDLE",
    ]  # fmt: skip
    assert [g for _, g in world.grippers] == [100.0, 0.0, 100.0]


@pytest.mark.parametrize("seen", [CUBE, BOX + np.array([0.01, 0.08, 0.025])], ids=["table", "beside"])
def test_a_cube_still_on_the_table_retries(seen):
    world = reach("VERIFY")
    world.cube = seen
    assert world.run_until("RETRY") <= 0.1 + DT


def test_a_cube_not_seen_for_2_s_is_done_with_a_warning():
    world = reach("VERIFY")
    world.cube = None  # its last detection, taken before VERIFY, does not count
    assert abs(world.run_until("DONE") - 2.0) <= DT + EPS
    assert "not seen" in world.transitions[-1]


# --- RETRY, HOME_RETRY ---


def test_retry_opens_the_gripper_and_parks_for_a_new_attempt():
    world = reach("RETRY")
    assert world.grippers[-1] == ("RETRY", 100.0)
    world.step()
    assert world.fsm.label == "HOME_RETRY"
    world.run_until("WAIT_TARGET")
    near(world.reached[-1][1], PARK)
    assert "attempt 2/3" in world.transitions[-1]


def test_a_retry_locks_the_target_on_new_detections_only():
    world = reach("HOME_RETRY")
    world.cube = CUBE + np.array([0.003, 0.0, 0.0])  # moved by the failed grasp, within the lock spread
    world.run_until("PLAN")
    np.testing.assert_allclose(world.requests[-1][0], world.cube)


@pytest.mark.parametrize("retries", [1, 3])
def test_the_last_failed_attempt_is_grasp_failed(retries):
    world = World(max_retries=retries)
    world.effort = EMPTY
    world.step(start=True)
    world.run_until("ERROR:grasp_failed", within=60.0 * retries)
    assert world.states().count("RETRY") == retries
    assert world.states().count("GRASP") == retries


# --- STOPPED, ERROR ---


@pytest.mark.parametrize("state", list(State))
def test_stop_from_every_state_freezes_then_idles(state):
    world = reach(state)
    q = world.q.copy()
    out = world.step(stop=True)
    assert out.state == "STOPPED"
    np.testing.assert_allclose(out.command, q)  # the measured position as the target
    world.step()
    assert world.fsm.label == "IDLE"
    world.step(start=True)
    assert world.fsm.label == "WAIT_TARGET"


ACTIVE = ["WAIT_TARGET", "PLAN", "APPROACH", "DESCEND", "GRASP", "LIFT", "TRANSPORT", "RELEASE", "HOME"]


@pytest.mark.parametrize("state", [*ACTIVE, "VERIFY", "HOME_RETRY"])
def test_joint_states_lost_in_a_cycle_stop_the_commands(state):
    world = reach(state, max_joint_state_age_s=0.2)
    world.answer = None  # PLAN keeps waiting
    world.pending = None if state == "PLAN" else world.pending
    world.blocked = True  # the motions keep running
    world.cube = None  # VERIFY keeps waiting
    world.joints = False
    lost = world.joints_received
    world.run_until("ERROR:no_joint_states", within=0.3)
    assert world.now - DT - lost <= 0.2 + DT + EPS
    for _ in range(50):
        world.step()
    assert "ERROR:no_joint_states" not in world.command_states  # commands stop


@pytest.mark.parametrize("state", ["IDLE", "ERROR"])
def test_joint_states_are_not_needed_at_rest(state):
    world = reach(state)
    world.joints = False
    for _ in range(50):
        world.step()
    assert world.fsm.label == ("IDLE" if state == "IDLE" else "ERROR:unreachable")


def test_a_new_cycle_gets_every_attempt_again():
    world = World(max_retries=2)
    world.effort = EMPTY
    world.step(start=True)
    world.run_until("ERROR:grasp_failed")
    world.step(start=True)
    world.step(start=True)
    world.run_until("ERROR:grasp_failed")
    assert world.states().count("GRASP") == 4


def test_start_rearms_an_error_into_idle():
    world = reach("ERROR")
    world.step(start=True)
    assert world.fsm.label == "IDLE"
    world.step(start=True)
    assert world.fsm.label == "WAIT_TARGET"


def test_the_time_in_idle_counts_from_the_first_step():
    world = World()
    world.now = 1000.0  # a steady clock counts from the boot, not from the node's start
    for _ in range(10):
        world.step()
    world.step(start=True)
    assert world.transitions[-1].endswith(", 0.20 s in IDLE)")


def test_each_transition_logs_the_reason_and_the_time_in_the_old_state():
    world = reach("GRASP")
    world.run_until("LIFT")
    log = r"GRASP → LIFT \(holding: effort 0\.150, 0\.8[02] s in GRASP\)"  # 0.82: float rounding
    assert re.fullmatch(log, world.transitions[-1])
