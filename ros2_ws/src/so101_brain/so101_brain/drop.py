"""The drop of phase 2 (docs/project/phase2/P2-02 §2): the joints that hold a token above a column of an
edge-on grid, the jaws normal to the grid (release_pose), and one move of /brain/drop or /brain/handover as
steps (drop_steps, handover_steps) that Move runs tick by tick, as fsm.Fsm does a cycle. Pure, no ROS.
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
DESCENT_V = 0.3  # rad/s, from the pre-drop to the release
NO_TOKEN = "no_token"  # Move's result when the jaws closed on nothing


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
        Step("descent", q=release, v=DESCENT_V),
        Step("release", gripper=p.open_pct, wait=p.release_wait_s),
        *lifts,
        Step(f"lift_{last}", q=pre),
        Step("clear", q=high),
        Step("handover", q=handover),
    ]


def handover_steps(handover, p) -> list:
    """The jaws open, the arm to `handover`: where a token is given and the camera sees the whole grid."""
    return [Step("handover", q=np.asarray(handover, dtype=float), gripper=p.open_pct)]


class Move:
    """Runs steps one after the other. step() is called at each tick with the measured joints, and returns
    what to command; its result ends the move. The motion's rules are the cycle's (fsm.Fsm._follow, _held):
    min-jerk from the measured joints, reached within the tolerances, a timeout at T + the margin."""

    def __init__(self, steps, p):
        self._steps, self._p = list(steps), p
        self._started = None  # when the running step started, s
        self._trajectory = None  # its motion
        self._result = ""  # what the move ends with once its steps are done

    @property
    def name(self) -> str:
        return self._steps[0].name if self._steps else ""

    def step(self, now, q, effort, gripper) -> Out:
        """One tick: `q` the measured arm joints, `effort` the gripper's closing effort, `gripper` its
        opening in % or None."""
        sent = command = None  # a step that ends still sends its goal, as the cycle does
        while self._steps:
            step, p = self._steps[0], self._p
            if self._started is None:
                self._started, sent = now, step.gripper if step.gripper is not None else sent
                speed = p.v_max if step.v is None else step.v
                self._trajectory = None if step.q is None else min_jerk(q, step.q, speed, p.min_motion_s)
            t = now - self._started
            reached = True
            if self._trajectory is not None:
                command = self._trajectory.sample(t)
                errors = np.abs(q - self._trajectory.q_goal)
                reached = (
                    t >= self._trajectory.duration
                    and errors[:4].max() < p.reach_tol
                    and errors[4] < p.reach_tol_roll
                )
                if not reached and t > self._trajectory.duration + p.motion_margin_s:
                    return Out(command, sent, f"timeout:{step.name}")
            held = step.held_by is None or (
                effort > p.effort_threshold
                or (gripper is not None and gripper - p.close_pct >= p.hold_gap_pct)
            )
            if reached and held and t >= step.wait:
                self._steps.pop(0)
                self._started = None
                continue  # the next step starts on this tick
            if not held and t >= step.held_by:
                # Nothing held: the jaws open again, and the result waits for them. A token given right after
                # the reply needs them open.
                self._steps, self._result = (
                    [Step("reopen", gripper=p.open_pct, wait=p.release_wait_s)],
                    NO_TOKEN,
                )
                self._started = None
                continue
            return Out(command, sent)
        return Out(command, sent, self._result)
