"""Pick & place state machine (docs/project/06 §3, ADR-015, ADR-017). Pure, no ROS.

brain_node calls step(now, inputs) at 50 Hz and sends what it returns. The clock is injected: `now` (s, the
node's steady clock) drives every wait and timeout, and the ages come from the reception times on the same
clock, never from time.time(). Motions are min-jerk trajectories from the measured joints (06 §5), sampled
at each step. The planner runs outside: PLAN asks once (Outputs.plan_request) and gets its answer back in
Inputs.plan. One transition at most per step, so every state shows on /brain_state.
"""

from collections import deque
from dataclasses import dataclass
from enum import StrEnum

import numpy as np

from so101_brain.trajectory import min_jerk


class State(StrEnum):
    IDLE = "IDLE"
    WAIT_TARGET = "WAIT_TARGET"
    PLAN = "PLAN"
    APPROACH = "APPROACH"
    DESCEND = "DESCEND"
    GRASP = "GRASP"
    LIFT = "LIFT"
    TRANSPORT = "TRANSPORT"
    RELEASE = "RELEASE"
    HOME = "HOME"
    VERIFY = "VERIFY"
    RETRY = "RETRY"
    HOME_RETRY = "HOME_RETRY"
    DONE = "DONE"
    STOPPED = "STOPPED"
    ERROR = "ERROR"


# Out of a cycle: /joint_states may be missing there (driver not started yet, or stopped).
AT_REST = {State.IDLE, State.DONE, State.STOPPED, State.ERROR}
# States that end when their motion is reached (06 §3.1).
AFTER_MOTION = {
    State.APPROACH: (State.DESCEND, "pre-grasp reached"),
    State.DESCEND: (State.GRASP, "grasp pose reached"),
    State.TRANSPORT: (State.RELEASE, "above the box"),
    State.HOME: (State.VERIFY, "PARK reached"),
}


@dataclass(frozen=True)
class Params:
    """06 §3.1 and §6: brain_node passes the ones it declares (brain.yaml), the others are the table's."""

    park_pose: tuple = (1.9, -1.7, 1.5, 0.7, 0.0)  # rad: off the workspace view, no self-contact [MES-39]
    v_max: float = 1.0  # rad/s
    min_motion_s: float = 0.5  # 04 §6
    reach_tol: float = 0.01  # rad, every arm joint
    gripper_tol_pct: float = 3.0  # APPROACH also waits for the gripper this close to open_pct
    motion_margin_s: float = 1.0  # not reached at T + margin -> ERROR:timeout:<state>
    open_pct: float = 100.0
    close_pct: float = 0.0
    effort_threshold: float = 0.06  # gripper closing force above it: cube held [MES-10, MES-35, MES-48]
    target_lock_n: int = 5
    target_lock_spread_m: float = 0.005  # the n detections within this distance of each other
    max_detection_age_s: float = 0.5
    max_joint_state_age_s: float = 0.5
    max_retries: int = 3  # failed attempts before ERROR:grasp_failed
    wait_target_s: float = 10.0  # from PARK reached
    plan_timeout_s: float = 6.0  # slowest answer, a cube out of reach: 1.7 s, 2.4 s on CI [MES-40]
    grasp_wait_s: float = 0.8
    grasp_timeout_s: float = 1.5
    release_wait_s: float = 0.5
    verify_timeout_s: float = 2.0
    box_half_size_m: float = 0.05  # cube center within the box footprint (08 §3)


@dataclass(frozen=True)
class Detection:
    position: np.ndarray  # m, base_link
    received: float  # s, on the step clock: gives its age, and tells a new message from the same one
    yaw: float | None = None  # rad, cube only


@dataclass(frozen=True)
class Inputs:
    q: np.ndarray | None = None  # measured arm joints, rad (ARM_JOINT_NAMES order)
    joints_received: float = -np.inf  # s, reception of the last /joint_states
    effort: float = 0.0  # gripper closing force, normalized (ADR-016)
    cube: Detection | None = None  # last /object_position, however old
    box: Detection | None = None  # last /drop_box_position
    start: bool = False  # /brain/start since the last step
    stop: bool = False  # /brain/stop since the last step
    plan: tuple | None = None  # the planner's (GraspPlan | None, reason) once done, else None
    gripper: float | None = None  # measured gripper opening, %; None if /joint_states has none


@dataclass(frozen=True)
class Outputs:
    state: str  # for /brain_state: "WAIT_TARGET", "ERROR:object_lost"...
    command: np.ndarray | None = None  # arm joints to send now, rad
    gripper: float | None = None  # gripper goal to send now, %
    plan_request: tuple | None = None  # (cube, yaw, box, q): run planner.plan(*plan_request)
    transition: str | None = None  # "old → new (reason, 1.23 s in old)" when the state changed


class Fsm:
    def __init__(self, params: Params | None = None):
        self.p = params = params or Params()
        self.state = State.IDLE
        self.cause = None  # of the ERROR
        self._since = None  # when the state was entered; IDLE: at the first step
        self._motion = None  # running trajectory: (MinJerk, start time)
        self._goals = []  # the state's motions still to run (LIFT: pre-grasp, then lift)
        self._cubes = deque(maxlen=params.target_lock_n)
        self._boxes = deque(maxlen=params.target_lock_n)
        self._parked_at = None  # WAIT_TARGET: PARK reached, detections count from then
        self._target = None  # locked (cube, yaw, box)
        self._plan = None
        self._failures = 0

    @property
    def label(self) -> str:
        return f"ERROR:{self.cause}" if self.state is State.ERROR else str(self.state)

    @property
    def accepts_start(self) -> bool:
        """/brain/start: a new cycle from IDLE, a re-arm from ERROR; rejected otherwise (C-35)."""
        return self.state in (State.IDLE, State.ERROR)

    def step(self, now, inputs) -> Outputs:
        self._now, self._in, self._out = now, inputs, {}
        if self._since is None:
            self._since = now
        if inputs.stop:
            self._go(State.STOPPED, "/brain/stop")
        elif self.state not in AT_REST and self._stale():
            age = now - inputs.joints_received
            self._go(State.ERROR, f"no /joint_states for {age:.2f} s", "no_joint_states")
        else:
            self._tick()
        return Outputs(state=self.label, **self._out)

    def _tick(self):
        p, i, t = self.p, self._in, self._now - self._since
        match self.state:
            case State.IDLE if i.start and self._stale():
                self._go(State.ERROR, "/brain/start without /joint_states", "no_joint_states")
            case State.IDLE if i.start:
                self._failures = 0
                self._go(State.WAIT_TARGET, "/brain/start")
            case State.ERROR if i.start:
                self._go(State.IDLE, "/brain/start: re-armed")
            case State.STOPPED:
                self._go(State.IDLE, "stopped")
            case State.DONE:
                self._go(State.IDLE, "cycle done")
            case State.WAIT_TARGET:
                self._wait_target()
            case State.PLAN if i.plan is not None and i.plan[0] is None:
                self._go(State.ERROR, "no plan", i.plan[1])
            case State.PLAN if i.plan is not None:
                self._plan = i.plan[0]
                fit = "aligned" if self._plan.aligned else "free roll"
                self._go(State.APPROACH, f"plan: tilt {self._plan.tilt_deg:g}°, {fit}")
            case State.PLAN if t > p.plan_timeout_s:
                self._go(State.ERROR, f"no plan in {p.plan_timeout_s:g} s", "timeout:PLAN")
            case State.APPROACH | State.DESCEND | State.TRANSPORT | State.HOME:
                if self._follow():
                    self._go(*AFTER_MOTION[self.state])
            case State.GRASP if t >= p.grasp_wait_s and i.effort > p.effort_threshold:
                self._go(State.LIFT, f"holding: effort {i.effort:.3f}")
            case State.GRASP if t >= p.grasp_timeout_s:
                self._go(State.RETRY, f"empty gripper: effort {i.effort:.3f}")
            case State.LIFT:
                if self._follow():
                    if i.effort > p.effort_threshold:
                        self._go(State.TRANSPORT, f"lifted, effort {i.effort:.3f}")
                    else:
                        self._go(State.RETRY, f"object lost during the lift: effort {i.effort:.3f}")
            case State.RELEASE if t >= p.release_wait_s:
                self._go(State.HOME, "released")
            case State.VERIFY:
                self._verify(t)
            case State.RETRY:
                self._go(State.HOME_RETRY, f"attempt {self._failures}/{p.max_retries} failed")
            case State.HOME_RETRY:
                if self._follow():
                    if self._failures >= p.max_retries:
                        self._go(State.ERROR, f"{self._failures} failed attempts", "grasp_failed")
                    else:
                        self._go(
                            State.WAIT_TARGET, f"PARK reached, attempt {self._failures + 1}/{p.max_retries}"
                        )

    def _go(self, state, reason, cause=None):
        """Transition: the log line, then the entry action of the new state (06 §3.1)."""
        old, spent, moving = self.label, self._now - self._since, self._motion is not None
        self.state, self.cause, self._since = state, cause, self._now
        self._motion, self._goals = None, []
        self._out["transition"] = f"{old} → {self.label} ({reason}, {spent:.2f} s in {old})"
        p = self.p
        match state:
            case State.WAIT_TARGET:
                self._parked_at = None
                self._move(p.park_pose)
            case State.PLAN:
                cube, yaw, box = self._target
                self._out["plan_request"] = (cube, yaw, box, self._in.q.copy())
            case State.APPROACH:
                self._out["gripper"] = p.open_pct
                self._move(self._plan.pre_grasp)
            case State.DESCEND:
                self._move(self._plan.grasp)
            case State.GRASP:
                self._out["gripper"] = p.close_pct
            case State.LIFT:
                self._move(self._plan.pre_grasp, self._plan.lift)
            case State.TRANSPORT:
                self._move(self._plan.drop)
            case State.RELEASE:
                self._out["gripper"] = p.open_pct
            case State.HOME | State.HOME_RETRY:
                self._move(p.park_pose)
            case State.RETRY:
                self._out["gripper"] = p.open_pct
                self._failures += 1
            case State.STOPPED:
                self._freeze()
            case State.ERROR if moving:  # an error out of a motion: never "move anyway" (06 §10)
                self._freeze()

    def _stale(self) -> bool:
        return self._now - self._in.joints_received > self.p.max_joint_state_age_s

    def _freeze(self):
        """The measured position as the target; nothing if it is not fresh (commands stop)."""
        if not self._stale():
            self._out["command"] = np.array(self._in.q, dtype=float)

    def _move(self, *goals):
        self._goals = [np.asarray(goal, dtype=float) for goal in goals[1:]]
        self._start_motion(np.asarray(goals[0], dtype=float))

    def _start_motion(self, goal):
        """From the measured joints, not the last command: nothing jumps if the arm lagged (06 §5)."""
        trajectory = min_jerk(self._in.q, goal, self.p.v_max, self.p.min_motion_s)
        self._motion = (trajectory, self._now)
        self._out["command"] = trajectory.sample(0.0)

    def _follow(self) -> bool:
        """Streams the state's motions; True once the last is reached. Not reached at T + margin: ERROR.

        APPROACH also waits for the open gripper: DESCEND must not bring closed jaws onto the cube."""
        trajectory, start = self._motion
        t = self._now - start
        self._out["command"] = trajectory.sample(t)
        error = float(np.max(np.abs(self._in.q - trajectory.q_goal)))
        gripper = self._in.gripper if self.state is State.APPROACH else None
        opened = gripper is None or abs(gripper - self.p.open_pct) <= self.p.gripper_tol_pct
        if t >= trajectory.duration and error < self.p.reach_tol and opened:
            if not self._goals:
                self._motion = None
                return True
            self._start_motion(self._goals.pop(0))
        elif t > trajectory.duration + self.p.motion_margin_s:
            reason = f"not reached at T + {self.p.motion_margin_s:g} s: joint error {error:.3f} rad"
            if not opened:
                reason += f", gripper {gripper:.0f} %"
            self._go(State.ERROR, reason, f"timeout:{self.state}")
        return False

    def _wait_target(self):
        """PARK first; then the target from the detections received there (arm out of the view, ADR-017)."""
        if self._parked_at is None:
            if self._follow():
                self._parked_at = self._now
                self._cubes.clear()
                self._boxes.clear()
            return
        for history, detection in ((self._cubes, self._in.cube), (self._boxes, self._in.box)):
            if detection is None or detection.received < self._parked_at:
                continue
            if not history or detection.received > history[-1].received:
                history.append(detection)
        target = self._lock()
        if target is not None:
            self._target = target
            cube, yaw, box = target
            if self._in_box(cube, box):  # grasping it would close the jaws on the box wall too (#143)
                where = f"cube {np.round(cube, 3)}, box {np.round(box, 3)}"
                self._go(State.DONE, f"cube already in the box: {where}")
                return
            yaw = "unknown" if yaw is None else f"{yaw:.2f} rad"
            self._go(
                State.PLAN, f"target locked: cube {np.round(cube, 3)}, yaw {yaw}, box {np.round(box, 3)}"
            )
        elif self._now - self._parked_at > self.p.wait_target_s:
            self._go(State.ERROR, f"no target locked {self.p.wait_target_s:g} s after PARK", "object_lost")

    def _lock(self):
        """(cube, yaw, box): the mean of the last n detections of each, when they lie within the spread of
        each other and the newest is younger than max_detection_age_s; the newest cube yaw. None until then.

        The newest only: perception publishes valid detections only, and a dropped frame must not reset the
        lock; the spread already rejects a cube that moved."""
        means = []
        for history in (self._cubes, self._boxes):
            if len(history) < self.p.target_lock_n:
                return None
            if self._now - history[-1].received >= self.p.max_detection_age_s:
                return None
            P = np.array([d.position for d in history])
            if np.max(np.linalg.norm(P[:, None] - P[None], axis=-1)) >= self.p.target_lock_spread_m:
                return None
            means.append(P.mean(axis=0))
        return means[0], self._cubes[-1].yaw, means[1]

    def _in_box(self, cube, box) -> bool:
        """The cube's center within the box footprint (08 §3): a placed cube, for VERIFY and for the lock."""
        return bool(np.all(np.abs(cube[:2] - box[:2]) < self.p.box_half_size_m))

    def _verify(self, t):
        """A cube seen since PARK: in the box footprint -> DONE, elsewhere -> RETRY; unseen -> DONE."""
        cube = self._in.cube
        if cube is not None and cube.received > self._since:
            if self._in_box(cube.position, self._target[2]):
                self._go(State.DONE, "cube seen in the box")
            else:
                self._go(State.RETRY, f"cube still on the table at {np.round(cube.position, 3)}")
        elif t > self.p.verify_timeout_s:
            self._go(
                State.DONE, f"warning: cube not seen for {self.p.verify_timeout_s:g} s, assumed in the box"
            )
