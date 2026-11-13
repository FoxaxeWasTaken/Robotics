"""The drop of phase 2 (docs/project/phase2/P2-02 §2): the joints that hold a token above a column of an
edge-on grid, the jaws normal to the grid (release_pose), and one move of /brain/drop or /brain/handover as
steps (drop_steps, handover_steps) that Move runs tick by tick, as fsm.Fsm does a cycle; pick_steps is the
move of /brain/pick, a token taken at a station of the dispenser with the same pose (ADR-041). Pure, no ROS.
kinematics.py and fsm.py are unchanged: release_pose composes ik_pos and ik_point, Move takes fsm.Params.

The target is the token's centre, not the TCP: the token rests on the fixed jaw, 3.15 mm off the TCP. The pose
documented first (roll ±90°, the TCP on the slot plane) dropped 0 token of 28 [MES-64]. Where to aim across
the slot (the layout's plane offset and the 1 mm of P2-02 §2.2) is the caller's: release_pose puts the centre
on the target it gets.
"""

from dataclasses import dataclass

import numpy as np

from so101_brain.grasp_planner import tilted_down
from so101_brain.trajectory import min_jerk

# rad: +90° and the URDF wrist's 2.8° (wrist_roll rpy 0.0487), the jaws normal to the arm plane [MES-64]
ROLL = 1.6195
# Approach tilted from vertical, outwards: it solves all 7 columns; bare ik_pos flips the tool [MES-64]
TILT_DEG = 70.0
TOKEN_IN_TCP = np.array([-0.00315, 0.0, -0.001])  # m: the held token's centre in gripper_frame_link [MES-64]
SEED_TAIL = (-0.4, 0.0, 1.0)  # lift, elbow, wrist_flex of the default seed: bent (q = 0 is singular)
PRE_DROP = 0.025  # m above the release: height first, then the column; 30 mm: column 6 at its reach [MES-63]
LIFT_STEP = 0.005  # m between the lift's waypoints: one joint move swept a jaw into the guide [MES-63]
DESCENT_V = 0.3  # rad/s, from the pre-drop to the release; the motion's v_max if that is lower
# The legs back once the jaws are empty: this many times the motion's v_max, so 1.4 rad/s on the sim, under
# the driver's limiter (1.5) [MES-73]. A ratio: an overlay that slows the arm slows these legs too.
EMPTY_SPEEDUP = 1.4
NO_TOKEN = "no_token"  # Move's result when the jaws closed on nothing
PICK_ABOVE = 0.040  # m above a station's token: the jaws pass over the station's walls [MES-75]
PICK_STEP = 0.010  # m between the waypoints of the way down and up: 16 of them every 5 mm took 8 s
PICK_OPEN_PCT = 40.0  # the jaws on the way down into a station [MES-75]
# m along the jaws' axis. On the way down the fixed jaw stays PICK_ASIDE further from the token than the
# pose that holds one: there it passed 0.8 mm from the token's face, the slot leaves the token 1.5 mm of play
# and the camera places a station within 1.5 mm, so it landed on a token and stopped. Then the arm moves
# sideways until the fixed jaw is where it holds a token in the slot's middle: a token closed on sits
# PICK_SEAT nearer the fixed jaw than TOKEN_IN_TCP says, and lifted there it rubbed the rail's wall [MES-77]
PICK_ASIDE = 0.0025
PICK_SEAT = 0.0008
AT_HANDOVER = 0.05  # rad: every arm joint this close to HANDOVER is there; a drop or a pick starts there
HANDOVER_RISE = 0.040  # m straight up before a handover from elsewhere: out of a station [MES-77]
# rad, by the step that meets a slot: how far the slot's plane may stand from the arm's there. The arm holds
# a token in its own vertical plane; a token dropped 1.5 deg past it on the fixed jaw's side fell beside its
# column, 8 deg the other way it fell in; at a station the jaws touched the stop from 6 deg on [MES-79]
EDGE_ON = {"descent": np.radians((-6.0, 1.0)), "seat": np.radians((-4.0, 4.0))}
# Where the arm waits beside a dispenser, in the dispenser's frame: the tool's place (m) and its tilt from
# vertical at HANDOVER, the dispenser where the layout puts it. The camera sees the whole grid [MES-65]
WAIT_IN_DISPENSER = (-0.081, -0.052, 0.069)
WAIT_TILT_DEG = 27.0


def release_pose(kin, target, q_seed=None) -> np.ndarray | None:
    """Joints that put the held token's centre at `target` (base_link, m), wrist_roll at ROLL and the approach
    tilted TILT_DEG; None if the IK finds none (unreachable, or only folded onto the arm).

    ik_pos with the roll locked gives the pan, so the arm plane and the token's offset in it; twice, since the
    pan moves the offset. Then ik_point tilts the approach in that plane, seeded with the solution.
    """
    target = np.asarray(target, dtype=float)
    azimuth = kin.arm_azimuth(target)
    seed = np.array([-azimuth, *SEED_TAIL, ROLL]) if q_seed is None else np.asarray(q_seed, dtype=float)
    for _ in range(2):
        p = target - _release_axes(azimuth) @ TOKEN_IN_TCP
        q = kin.ik_pos(p, q_seed=seed, roll=ROLL)
        if q is None:
            return None
        azimuth, seed = -q[0], q  # pan = -azimuth (04 §3.1)
    return kin.ik_point(p, tilted_down(TILT_DEG, azimuth), q_seed=seed, roll=ROLL)


def _release_axes(azimuth) -> np.ndarray:
    """The TCP's axes at the release, as columns: x the jaws' closing axis across the arm plane, z the
    approach tilted outwards in it."""
    z = tilted_down(TILT_DEG, azimuth)
    x = np.array([np.sin(azimuth), -np.cos(azimuth), 0.0])
    return np.column_stack([x, np.cross(z, x), z])


def off_plane(T, q) -> float:
    """rad between the vertical plane a slot stands in, the x-z plane of the frame T (base_link), and the
    arm's at the joints `q`: 0 when the arm meets the slot edge-on."""
    angle = np.arctan2(T[1, 0], T[0, 0]) + q[0]  # pan = -azimuth (04 §3.1)
    return float((angle + np.pi) % (2 * np.pi) - np.pi)


def edge_on(T, steps) -> bool:
    """Whether a drop's or a pick's steps meet the slot of the frame T edge-on, within EDGE_ON."""
    step = next(step for step in steps if step.name in EDGE_ON)
    low, high = EDGE_ON[step.name]
    return bool(low <= off_plane(T, step.q) <= high)


def wait_pose(kin, T_base_dispenser, seed) -> np.ndarray | None:
    """The joints the arm waits at beside a dispenser that stands at T_base_dispenser, as the camera
    measures it: `seed` (connect4.handover_pose) where the layout puts the dispenser, and the same place
    beside it elsewhere. None when the IK finds none."""
    p = (T_base_dispenser @ [*WAIT_IN_DISPENSER, 1.0])[:3]
    q = kin.ik_pos(p, q_seed=seed, roll=seed[4])  # the pan first: the tool tilts in the arm's plane
    return None if q is None else kin.ik_point(p, tilted_down(WAIT_TILT_DEG, -q[0]), q_seed=q, roll=seed[4])


@dataclass(frozen=True)
class Step:
    """One step of a move. It ends once its arm goal is reached, `wait` has passed and, with `held_by`, the
    token is held."""

    name: str
    q: np.ndarray | None = None  # arm goal, rad: a min-jerk motion from the measured joints
    v: float | None = None  # its peak speed, rad/s; None: the motion's v_max
    gripper: float | None = None  # %, sent as the step starts
    wait: float = 0.0  # s, its shortest time
    held_by: float | None = None  # s: the token is held by then (judged from `wait` on), or the move ends
    otherwise: tuple = ()  # the steps after the jaws opened again on nothing: the way out, then no_token


@dataclass(frozen=True)
class Out:
    command: np.ndarray | None = None  # arm joints for /joint_command
    gripper: float | None = None  # %
    result: str | None = None  # None: running; "": done; else why it ended (no_token, timeout:<step>)


def drop_steps(kin, target, handover, p) -> list | None:
    """The steps of one move (P2-02 §2.3), from `handover` with a token between the open jaws: close and
    check the grip, rise to the pre-drop's height, turn to the column, descend, open, lift straight up, come
    back. `target`: the token's centre at the release, base_link. `p`: fsm.Params. None if the pre-drop or
    the release is out of reach: every pose is solved here, before anything moves."""
    handover, target = np.asarray(handover, dtype=float), np.asarray(target, dtype=float)
    above = np.array([0.0, 0.0, PRE_DROP])
    pre = release_pose(kin, target + above, handover)
    release = None if pre is None else release_pose(kin, target, pre)
    if release is None:
        return None
    high = np.array([handover[0], *pre[1:]])  # HANDOVER is below the grid's top: up before the pan turns
    last = round(PRE_DROP / LIFT_STEP)
    lifts, seed = [], release
    for k in range(1, last):
        q = release_pose(kin, target + np.array([0.0, 0.0, k * LIFT_STEP]), seed)
        if q is not None:  # no pose there: on to the next waypoint, the pre-drop is the last
            lifts.append(Step(f"lift_{k}", q=q))
            seed = q
    return [
        Step("close", gripper=p.close_pct, wait=p.grasp_wait_s, held_by=p.grasp_timeout_s),
        Step("rise", q=high),
        Step("column", q=pre),
        Step("descent", q=release, v=min(DESCENT_V, p.v_max)),
        Step("release", gripper=p.open_pct, wait=p.release_wait_s),
        *lifts,
        Step(f"lift_{last}", q=pre),
        Step("clear", q=high, v=EMPTY_SPEEDUP * p.v_max),
        Step("handover", q=handover, v=EMPTY_SPEEDUP * p.v_max),
    ]


def pick_steps(kin, target, handover, p) -> list | None:
    """The steps of one pick at the dispenser (ADR-041), from `handover` with empty jaws: above the station,
    straight down beside the token with the jaws part open, sideways until the fixed jaw is seated on it,
    close and check the grip, straight up, back to `handover` with the token. `target`: the centre of the
    token that waits at the station, in the middle of its slot, base_link. Closed on nothing, the jaws open
    and the arm comes back the same way before the move says no_token. None if any pose of the way is out of
    reach: the way down has no waypoint to spare, a jaw passes 3 mm from the dispenser [MES-75]."""
    handover, target = np.asarray(handover, dtype=float), np.asarray(target, dtype=float)
    jaws = _release_axes(kin.arm_azimuth(target))[:, 0]  # from the token to the fixed jaw
    count = round(PICK_ABOVE / PICK_STEP)

    def way(aside, heights, seed):
        """The poses at these heights above the station, `aside` along the jaws' axis; None if one is out
        of reach."""
        poses = []
        for k in heights:
            seed = release_pose(kin, target + aside * jaws + np.array([0.0, 0.0, k * PICK_STEP]), seed)
            if seed is None:
                return None
            poses.append(seed)
        return poses

    down = way(PICK_ASIDE, range(count, -1, -1), handover)  # from above the station down to it
    up = down and way(-PICK_SEAT, range(count + 1), down[-1])  # seated, then up
    if not up:
        return None
    back = (
        *(Step(f"lift_{k}", q=q) for k, q in enumerate(up[1:], start=1)),
        Step("handover", q=handover),
    )
    return [
        Step("above", q=down[0], gripper=PICK_OPEN_PCT),
        *(Step(f"down_{k}", q=q) for k, q in enumerate(down[1:], start=1)),
        Step("seat", q=up[0]),
        Step("close", gripper=p.close_pct, wait=p.grasp_wait_s, held_by=p.grasp_timeout_s, otherwise=back),
        *back,
    ]


def handover_steps(handover, p, kin=None, q=None) -> list:
    """The jaws open, the arm to `handover`: where it waits, and the camera sees the whole grid. With `kin`
    and the joints `q`, away from `handover`, the arm first rises HANDOVER_RISE straight up, the tool's
    orientation kept: a pick cut short may have left it down in a station of the dispenser, and one joint
    motion from there sweeps the stop [MES-77]. A waypoint without a pose ends the rise there."""
    handover, rise = np.asarray(handover, dtype=float), []
    if q is not None and np.max(np.abs(np.asarray(q, dtype=float) - handover)) > AT_HANDOVER:
        pose, seed = kin.fk(q), q
        for k in range(1, round(HANDOVER_RISE / PICK_STEP) + 1):
            seed = kin.ik_pose(pose[:3, 3] + [0.0, 0.0, k * PICK_STEP], pose[:3, :3], q_seed=seed)
            if seed is None:
                break
            rise.append(Step(f"rise_{k}", q=seed))
    steps = [*rise, Step("handover", q=handover)]
    steps[0] = Step(steps[0].name, q=steps[0].q, gripper=p.open_pct)
    return steps


class Move:
    """Runs steps one after the other. step() is called at each tick with the measured joints, and returns
    what to command; its result ends the move. The motion's rules are the cycle's (fsm.Fsm._follow, _held):
    min-jerk from the measured joints, reached within the tolerances, a timeout at T + the margin."""

    def __init__(self, steps, p):
        self._steps, self._p = list(steps), p
        self._started = None  # when the running step started, s
        self._trajectory = None  # its motion
        self._result = ""  # what the move ends with once its steps are done
        self.seconds = {}  # of each step ended, by name; lift_1 to lift_5 as one "lift"

    @property
    def name(self) -> str:
        return self._steps[0].name if self._steps else ""

    def _end(self, now):
        name = self._steps[0].name.partition("_")[0]
        self.seconds[name] = self.seconds.get(name, 0.0) + now - self._started
        self._started = None

    def _start(self, step, now, q, sent):
        """The step starts on this tick, its motion from the measured joints: what the gripper is sent."""
        p = self._p
        self._started = now
        speed = p.v_max if step.v is None else step.v
        self._trajectory = None if step.q is None else min_jerk(q, step.q, speed, p.min_motion_s)
        return sent if step.gripper is None else step.gripper

    def _progress(self, t, q) -> tuple[bool, bool]:
        """(reached, late) of the running step's motion at `t`; a step without motion is reached."""
        if self._trajectory is None:
            return True, False
        p, duration = self._p, self._trajectory.duration
        errors = np.abs(q - self._trajectory.q_goal)
        reached = t >= duration and errors[:4].max() < p.reach_tol and errors[4] < p.reach_tol_roll
        return reached, not reached and t > duration + p.motion_margin_s

    def _holds(self, step, effort, gripper) -> bool:
        """A step that must hold a token does: the closing effort, or the jaws' gap from their command."""
        p = self._p
        return step.held_by is None or (
            effort > p.effort_threshold or (gripper is not None and gripper - p.close_pct >= p.hold_gap_pct)
        )

    def step(self, now, q, effort, gripper) -> Out:
        """One tick: `q` the measured arm joints, `effort` the gripper's closing effort, `gripper` its
        opening in % or None."""
        sent = command = None  # a step that ends still sends its goal, as the cycle does
        while self._steps:
            step = self._steps[0]
            if self._started is None:
                sent = self._start(step, now, q, sent)
            t = now - self._started
            if self._trajectory is not None:
                command = self._trajectory.sample(t)
            reached, late = self._progress(t, q)
            held = self._holds(step, effort, gripper)
            if late:
                self._end(now)
                return Out(command, sent, f"timeout:{step.name}")
            if reached and held and t >= step.wait:
                self._end(now)
                self._steps.pop(0)  # the next step starts on this tick
            elif not held and t >= step.held_by:
                # Nothing held: the jaws open again, and the result waits for them. A token given right after
                # the reply needs them open.
                self._end(now)
                reopen = Step("reopen", gripper=self._p.open_pct, wait=self._p.release_wait_s)
                self._steps = [reopen, *step.otherwise]
                self._result = NO_TOKEN
            else:
                return Out(command, sent)
        return Out(command, sent, self._result)
