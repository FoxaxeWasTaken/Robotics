"""Unit tests of the pick & place state machine (docs/project/06 §3.1, ADR-015). Pure: a fake clock, arm,
camera and planner; no ROS, no sim. One test per row of 06 §3.1, /brain/stop from every state."""

import dataclasses
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
        self.cube_yaw = True  # False: perception gives no yaw
        self.joints = True  # the driver publishes /joint_states
        self.blocked = False
        self.gripper = 0.0  # measured, %; None: not in /joint_states
        self.gripper_blocked = False
        self.gripper_floor = None  # the jaws stop at this opening, %: the cube between them; None: no cube
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
            for name, yaw in (("cube", 0.3 + 0.001 * self.frames if self.cube_yaw else None), ("box", None)):
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
                self.gripper = (
                    out.gripper if self.gripper_floor is None else max(out.gripper, self.gripper_floor)
                )
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
    assert not world.commands
    assert not world.grippers
    world.step(start=True)
    assert world.fsm.label == "WAIT_TARGET"


def test_start_without_joint_states_is_an_error():
    world = World()
    world.joints = False
    world.step(start=True)
    assert world.fsm.label == "ERROR:no_joint_states"
    assert "(/brain/start without /joint_states, " in world.transitions[-1]
    assert not world.commands


def test_start_is_ignored_while_busy_and_the_fsm_says_so():
    world = reach("APPROACH")
    assert not world.fsm.accepts_start
    world.step(start=True)
    assert world.fsm.label == "APPROACH"
    assert World().fsm.accepts_start
    assert reach("ERROR").fsm.accepts_start


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


def test_a_cube_locked_in_the_box_is_done_without_any_motion():
    # #143: a second /brain/start without /sim/randomize; grasping it closed the jaws on the box wall too.
    world = World()
    world.box = np.array([0.0512, -0.2534, 0.0])  # 4 decimals: the log rounds to 3
    world.cube = world.box + np.array([0.0123, -0.0217, 0.025])
    world.step(start=True)
    world.run_until("DONE")
    assert world.states() == ["WAIT_TARGET", "DONE"]
    where = f"cube {np.round(world.cube, 3)}, box {np.round(world.box, 3)}"
    assert reasons(world)[-1] == f"cube already in the box: {where}"
    assert world.requests == []  # no plan asked
    assert set(world.command_states) == {"WAIT_TARGET"}  # the park motion only
    assert world.grippers == []


@pytest.mark.parametrize("offset", [(0.051, 0.0), (0.0, -0.051)], ids=["x", "y"])
def test_a_cube_just_outside_the_box_footprint_is_still_planned(offset):
    world = World()
    world.cube = BOX + np.array([*offset, 0.025])  # box_half_size_m = 0.05
    world.step(start=True)
    world.run_until("PLAN")


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
    assert "(no target locked 3 s after PARK, " in world.transitions[-1]
    assert "ERROR:object_lost" not in world.command_states  # parked, nothing moves: nothing to freeze
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
    assert "(no plan, " in world.transitions[-1]
    for _ in range(50):
        world.step()
    assert len(world.commands) == sent  # never "move anyway" (06 §10)


def test_a_silent_planner_is_a_timeout():
    world = World(plan_timeout_s=1.0)
    world.answer = None
    world.step(start=True)
    world.run_until("PLAN")
    assert abs(world.run_until("ERROR:timeout:PLAN") - 1.0) <= 2 * DT
    assert "(no plan in 1 s, " in world.transitions[-1]


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
    assert "(not reached at T + 1 s: joint error 0.000 rad, gripper 0 %, " in world.transitions[-1]


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
    assert "(object lost during the lift: effort 0.000, " in world.transitions[-1]


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
    assert "(empty gripper: effort 0.000, " in world.transitions[-1]


def test_the_effort_threshold_is_injected():
    world = reach("GRASP", effort_threshold=0.2)  # HOLDING is 0.15
    world.run_until("RETRY", within=1.5 + DT)


# The real arm (#69, MES-56): no effort in /joint_states, so the cube is held when it stops the jaws at least
# hold_gap_pct above their goal; close_pct sits just below the cube's contact opening.
REAL_GRASP = {"close_pct": 20.0, "hold_gap_pct": 3.0}


def real_grasp(floor):
    """A world on the real arm's grasp parameters, no effort, the jaws stopped at `floor` (None: no cube)."""
    world = World(**REAL_GRASP)
    world.effort = EMPTY
    world.gripper_floor = floor
    world.step(start=True)
    world.run_until("GRASP")
    return world


@pytest.mark.parametrize("floor", [23.0, 26.0])  # hold_gap_pct above the goal, or more
def test_jaws_stopped_by_the_cube_hold_it_without_any_effort(floor):
    world = real_grasp(floor)
    assert world.grippers[-1] == ("GRASP", 20.0)
    assert abs(world.run_until("LIFT") - 0.8) <= DT + EPS
    assert f"(holding: effort 0.000, gripper {floor:.1f} %, " in world.transitions[-1]


@pytest.mark.parametrize(
    "floor", [None, 22.9]
)  # the jaws reach their goal, or stop under hold_gap_pct above it
def test_jaws_closing_to_their_goal_are_an_empty_gripper(floor):
    world = real_grasp(floor)
    assert abs(world.run_until("RETRY") - 1.5) <= DT + EPS
    assert "(empty gripper: effort 0.000, gripper " in world.transitions[-1]


def test_an_unknown_gripper_opening_never_holds():
    world = real_grasp(floor=26.0)
    world.gripper_blocked, world.gripper = True, None  # no gripper in /joint_states
    world.run_until("RETRY", within=1.5 + DT)


def test_a_cube_slipping_out_of_the_jaws_during_the_lift_retries():
    world = real_grasp(floor=26.0)
    world.run_until("LIFT")
    world.gripper_floor = None  # the jaws close on nothing
    world.gripper = REAL_GRASP["close_pct"]
    world.run_until("RETRY")
    near(world.reached[-1][1], PLAN.lift)  # judged once lifted
    assert "(object lost during the lift: effort 0.000, gripper 20.0 %, " in world.transitions[-1]


def test_a_cube_still_in_the_jaws_after_the_lift_is_transported():
    world = real_grasp(floor=26.0)
    world.run_until("TRANSPORT")
    assert "(lifted: effort 0.000, gripper 26.0 %, " in world.transitions[-1]


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
    assert "(attempt 1/3 failed, " in world.transitions[-1]
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
    assert f"({retries} failed attempts, " in world.transitions[-1]
    assert world.states().count("GRASP") == retries


# --- STOPPED, ERROR ---


@pytest.mark.parametrize("state", list(State))
def test_stop_from_every_state_freezes_then_idles(state):
    world = reach(state)
    q = world.q.copy()
    out = world.step(stop=True)
    assert out.state == "STOPPED"
    assert "(/brain/stop, " in out.transition
    np.testing.assert_allclose(out.command, q)  # the measured position as the target
    world.step()
    assert world.fsm.label == "IDLE"
    assert "(stopped, " in world.transitions[-1]
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
    age = world.now - DT - lost
    assert f"(no /joint_states for {age:.2f} s, " in world.transitions[-1]
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
    assert "(/brain/start: re-armed, " in world.transitions[-1]
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
    # 0.82: float rounding
    log = r"GRASP → LIFT \(holding: effort 0\.150, gripper 0\.0 %, 0\.8[02] s in GRASP\)"
    assert re.fullmatch(log, world.transitions[-1])


# --- The reasons logged and the target lock, completed by mutation testing (#59, MES-42) ---


def reasons(world):
    """The reason of each transition, out of "old → new (reason, 1.23 s in old)"."""
    return [t.split(" (", 1)[1].rsplit(", ", 1)[0] for t in world.transitions]


def park(world):
    """Steps until WAIT_TARGET has reached PARK: detections count from the next frame."""
    while not np.allclose(world.q, PARK):
        world.step()
    world.step()


@pytest.mark.parametrize("aligned", [True, False], ids=["aligned", "free_roll"])
def test_a_full_cycle_logs_the_reason_of_each_transition(aligned):
    """06 §7: the operator reads the cycle in these lines. Positions with 4 decimals: the log rounds to 3."""
    world = World()
    world.cube, world.box = np.array([0.2234, 0.0512, 0.025]), np.array([0.0512, -0.2534, 0.0])
    world.answer = (dataclasses.replace(PLAN, aligned=aligned), None)
    world.step(start=True)
    world.run_until("RELEASE")
    world.cube = world.box + np.array([0.01, -0.02, 0.025])
    world.run_until("IDLE")
    cube, yaw, box, _ = world.requests[0]
    assert reasons(world) == [
        "/brain/start",
        f"target locked: cube {np.round(cube, 3)}, yaw {yaw:.2f} rad, box {np.round(box, 3)}",
        f"plan: tilt 10°, {'aligned' if aligned else 'free roll'}",
        "pre-grasp reached",
        "grasp pose reached",
        f"holding: effort {HOLDING:.3f}, gripper 0.0 %",
        f"lifted: effort {HOLDING:.3f}, gripper 0.0 %",
        "above the box",
        "released",
        "PARK reached",
        "cube seen in the box",
        "cycle done",
    ]


def test_a_target_without_yaw_logs_it_unknown():
    world = World()
    world.cube_yaw = False
    world.step(start=True)
    world.run_until("PLAN")
    assert ", yaw unknown, " in world.transitions[-1]
    assert world.requests[0][1] is None


def test_a_cube_still_on_the_table_logs_where():
    world = reach("VERIFY")
    world.cube = np.array([0.2234, 0.0512, 0.025])
    world.run_until("RETRY")
    assert "(cube still on the table at [0.223 0.051 0.025], " in world.transitions[-1]


@pytest.mark.parametrize("name", ["cube", "box"])
def test_an_outlier_right_after_park_leaves_the_window(name):
    """Only the last 5 detections count: a glitch right after PARK must not block the lock for good."""
    world = World()
    good = getattr(world, name)
    setattr(world, name, good + np.array([0.03, 0.0, 0.0]))
    world.step(start=True)
    park(world)
    for _ in range(round(0.25 / DT)):  # 2 or 3 outlier frames
        world.step()
    setattr(world, name, good)
    world.run_until("PLAN", within=2.0)
    np.testing.assert_allclose(world.requests[0][0 if name == "cube" else 2], good, atol=1e-12)


def test_the_lock_needs_the_newest_detection_fresh_not_the_older_ones():
    """4 cube detections, a gap longer than max_detection_age_s (0.5 s), then one more: it locks on it."""
    world = World()
    world.cube = None
    world.step(start=True)
    park(world)
    world.cube = CUBE
    first = world.frames
    while world.frames < first + 4:
        world.step()
    world.cube = None
    for _ in range(round(0.7 / DT)):
        world.step()
    world.cube = CUBE
    seen = world.frames
    while world.frames == seen:
        world.step()
    assert world.fsm.label == "PLAN"


def test_box_detections_count_while_the_cube_is_out_of_view():
    """Each object has its own history: box frames seen alone still fill the box's."""
    world = World(max_detection_age_s=1.0)
    world.cube = None
    world.step(start=True)
    park(world)
    for _ in range(round(0.6 / DT)):  # 6 box frames, no cube
        world.step()
    world.cube, world.box = CUBE, None  # the box out of view while the cube's 5 frames come
    world.run_until("PLAN", within=1.0)
    np.testing.assert_allclose(world.requests[0][2], BOX)


def test_joint_states_exactly_max_age_old_are_still_fresh():
    """06 §3.1: stale means older than max_joint_state_age_s (0.5 s), the age itself is fresh."""
    out = Fsm().step(10.0, Inputs(q=np.zeros(5), joints_received=9.5, start=True))
    assert out.state == "WAIT_TARGET"
