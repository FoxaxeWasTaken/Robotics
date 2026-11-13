#!/usr/bin/env python3
"""brain_node.py - intelligence of the arm: kinematics and the pick & place state machine.

Never talks to the backend: reads /joint_states, commands through /joint_command. ROS glue only (06 §2):
callbacks store the last message with its reception time, a 50 Hz tick feeds them to fsm.Fsm.step and sends
what it returns; the services answer at once, their effect shows on /brain_state (06 §4, ADR-017). PLAN runs
the grasp planner in a worker process, which never touches a ROS object; the tick polls it.
Publishes the TCP pose from forward kinematics. In IDLE, /go_to_target solves a position IK and returns at
once, the tick streams the min-jerk trajectory (04 §6).
In game mode (connect4.enabled, phase2/P2-02 §2.3), /brain/drop, /brain/pick and /brain/handover run one move
of drop.Move in the same slot of the tick, in IDLE, and reply at its end; fsm.py does not know them. Where the
grid and the dispenser stand comes from the camera (/connect4/grid_pose, /connect4/dispenser_pose): a move is
solved, then judged against them before anything moves (P2-02 §2.5).
"""

import inspect
import multiprocessing
import os
import signal
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
import rclpy
import rclpy.node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped
from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from so101_brain import clearance, drop
from so101_brain.fsm import Detection, Fsm, Inputs, Params, State
from so101_brain.grasp_planner import GraspPlanner
from so101_brain.kinematics import Kinematics
from so101_brain.trajectory import min_jerk
from so101_common.robot import ARM_JOINT_NAMES, BASE_FRAME, JOINT_LIMITS, rad_to_pct
from so101_interfaces.srv import Drop, GoToTarget

END_EFFECTOR_POSE_HZ = 30.0  # docs/project/01 §3.1
STREAM_HZ = 50.0  # the FSM tick, /joint_command during a motion (04 §6, 06 §4)
STATE_HZ = 1.0  # /brain_state, also on each transition (01 §3.1)
# Startup-only parameters (06 §6), values in so101_bringup/config/brain.yaml: name: (fsm.Params field,
# description, min, max), the default is the field's; fsm.park_pose and the planner's grasp.* are apart.
PARAMETERS = {
    "motion.v_max_rad_s": ("v_max", "Peak arm joint speed of a motion, rad/s", 0.05, 1.5),
    "motion.reach_tol_rad": (
        "reach_tol",
        "Target reached when every arm joint but wrist_roll is this close, rad",
        0.001,
        0.1,
    ),
    "motion.reach_tol_roll_rad": ("reach_tol_roll", "... and wrist_roll this close, rad", 0.001, 0.5),
    "motion.min_duration_s": ("min_motion_s", "Shortest min-jerk motion, s (04 §6)", 0.1, 5.0),
    "motion.timeout_margin_s": (
        "motion_margin_s",
        "Not reached at T + this margin: timeout, s (04 §6)",
        0.1,
        10.0,
    ),
    "grasp.open_pct": ("open_pct", "Gripper open, %", 0.0, 100.0),
    "grasp.close_pct": ("close_pct", "Gripper closed on the cube, %", 0.0, 100.0),
    "grasp.gripper_tol_pct": (
        "gripper_tol_pct",
        "APPROACH waits for the measured gripper this close to open, %",
        0.5,
        20.0,
    ),
    "grasp.effort_threshold": (
        "effort_threshold",
        "Normalized gripper closing force above which the cube is held (ADR-016)",
        0.0,
        1.0,
    ),
    "grasp.hold_gap_pct": (
        "hold_gap_pct",
        "Or the jaws stopped this far above close_pct: the cube is held (the real arm, #69), %",
        0.5,
        50.0,
    ),
    "fsm.target_lock_n": ("target_lock_n", "Consistent detections that lock the target", 1, 50),
    "fsm.target_lock_spread_m": (
        "target_lock_spread_m",
        "The locking detections lie within this distance of each other, m",
        0.0005,
        0.05,
    ),
    "fsm.max_detection_age_s": (
        "max_detection_age_s",
        "The newest locking detection is younger than this, s",
        0.05,
        5.0,
    ),
    "fsm.max_joint_state_age_s": (
        "max_joint_state_age_s",
        "/joint_states older than this during a cycle: ERROR:no_joint_states, s",
        0.05,
        5.0,
    ),
    "fsm.max_retries": ("max_retries", "Failed attempts before ERROR:grasp_failed", 1, 10),
    "fsm.wait_target_s": (
        "wait_target_s",
        "No target locked this long after PARK: ERROR:object_lost, s",
        1.0,
        120.0,
    ),
    "fsm.plan_timeout_s": ("plan_timeout_s", "No plan within this time: ERROR:timeout:PLAN, s", 1.0, 60.0),
    "fsm.grasp_wait_s": ("grasp_wait_s", "GRASP: closing time before the grasp is judged, s", 0.1, 5.0),
    "fsm.grasp_timeout_s": (
        "grasp_timeout_s",
        "GRASP: still not held (effort, jaws gap): RETRY, s",
        0.2,
        10.0,
    ),
    "fsm.release_wait_s": ("release_wait_s", "RELEASE: opening time before HOME, s", 0.1, 5.0),
    "fsm.verify_timeout_s": (
        "verify_timeout_s",
        "VERIFY: no detection this long: DONE with a warning, s",
        0.5,
        10.0,
    ),
    "fsm.box_half_size_m": (
        "box_half_size_m",
        "In the box: the cube center within this distance of the box center, per axis, m (08 §3)",
        0.01,
        0.2,
    ),
}
HANDOVER = (-0.8, -0.6, 0.6, 1.1, 1.57)  # default of connect4.handover_pose [MES-65]
MARKERS = (("pre_grasp", (1.0, 1.0, 0.0)), ("grasp", (0.0, 1.0, 0.0)), ("drop", (0.0, 0.0, 1.0)))  # 06 §7


_PLANNER = None  # in the worker process (start_worker)


def planner_pool(urdf, params):
    """The planning worker, started now: one process with its own GraspPlanner(**params).

    A process, not a thread: the planner holds the GIL, and a worker thread delayed the 50 Hz tick to 27-29 ms
    at p99 during PLAN [MES-40]. forkserver, not fork: the node is multithreaded (DDS)."""
    pool = ProcessPoolExecutor(
        max_workers=1,
        mp_context=multiprocessing.get_context("forkserver"),
        initializer=start_worker,
        initargs=(urdf, params),
    )
    pool.submit(os.getpid).result()  # the first PLAN does not wait for the start
    return pool


def start_worker(urdf, params):
    global _PLANNER
    signal.signal(signal.SIGINT, signal.SIG_IGN)  # Ctrl-C stops the node, and the node the worker
    _PLANNER = GraspPlanner(Kinematics(urdf), **params)


def timed_plan(*request):
    """In the worker: the planner's answer, (GraspPlan | None, reason), and its duration in s."""
    start = time.monotonic()
    return _PLANNER.plan(*request), time.monotonic() - start


class BrainNode(rclpy.node.Node):
    """Brain of the arm: kinematics, then pick & place."""

    def __init__(self):
        super().__init__("so101_brain")
        urdf = Path(get_package_share_directory("so101_description")) / "urdf" / "so101.urdf"
        self._kin = Kinematics(str(urdf))
        # Last /joint_states: (arm joints in ARM_JOINT_NAMES order, stamp, reception time, gripper effort,
        # gripper opening in % or None).
        self._joints = None
        self._cube = self._box = None  # last /object_position, /drop_box_position (fsm.Detection)
        self._start = self._stop = False  # service requests, for the next tick
        self._motion = None  # running /go_to_target trajectory: (MinJerk, start time in s)
        d = Params()
        values = {
            field: self._declare(name, getattr(d, field), description, low, high)
            for name, (field, description, low, high) in PARAMETERS.items()
        }
        park = self._declare_pose(
            "fsm.park_pose", d.park_pose, "PARK, the arm out of the camera view: 5 arm joints in rad"
        )
        self._fsm = Fsm(Params(park_pose=park, **values))
        self._p = self._fsm.p  # also /go_to_target's motion: v_max, min_motion_s, reach_tol, motion_margin_s
        planner = self._declare_planner()
        self._game = self.declare_parameter(
            "connect4.enabled",
            False,
            ParameterDescriptor(
                description="Game mode: /brain/drop, /brain/pick, /brain/handover", read_only=True
            ),
        ).value
        self._handover = np.array(
            self._declare_pose(
                "connect4.handover_pose",
                HANDOVER,
                "HANDOVER, where a token is given and the camera sees the whole grid: 5 arm joints in rad",
            )
        )
        boxes = self.declare_parameter(
            "connect4.grid_boxes",
            rclpy.Parameter.Type.DOUBLE_ARRAY,
            ParameterDescriptor(
                description="The grid's volume, in the frame of /connect4/grid_pose: per box, its low corner "
                "then its high corner, m; a move that passes nearer than clearance.MARGIN is refused",
                read_only=True,
            ),
        ).value
        self._boxes = np.array(boxes or [], dtype=float).reshape(-1, 6).reshape(-1, 2, 3)
        if np.any(self._boxes[:, 0] >= self._boxes[:, 1]):
            raise ValueError(f"connect4.grid_boxes {boxes}: each box is its low corner, then its high corner")
        self._arm = clearance.Arm(urdf) if self._game and len(self._boxes) else None
        # The parameter: where the arm waits while the camera has seen no dispenser, then the IK's seed
        self._wait_seed = self._handover
        self._grid = self._dispenser = None  # where the camera sees them: 4x4 frames in base_link
        self._solved = -np.inf  # when a service call last ended its solve (_refusal)
        self._move = None  # the running /brain/drop, /brain/pick or /brain/handover (_begin)
        self._pick_cut = False  # the last move was a pick that did not end well (_srv_handover)
        # ROS ignores a key no node declares: a typo in brain_real.yaml would keep the sim's value (#57).
        if unknown := sorted(set(self._parameter_overrides) - set(self._parameters)):  # rclpy: no public API
            raise ValueError(f"Unknown parameters {unknown}: a typo in a config file? (06 §6)")
        self._worker = planner_pool(str(urdf), planner)  # its own process (06 §4)
        self._planning = None  # the worker's future of the last PLAN
        self._locked = None  # (cube, yaw) of the last PLAN: the cube marker
        self._plan_ticks = []  # tick periods while in PLAN, s (C-33: planning never blocks the tick)
        self._last_tick = None

        self.create_subscription(JointState, "/joint_states", self._cb_joint_states, 10)
        self.create_subscription(PoseStamped, "/object_position", self._cb_object, 10)
        self.create_subscription(PoseStamped, "/drop_box_position", self._cb_box, 10)
        self._pub_pose = self.create_publisher(PoseStamped, "/end_effector_pose", 10)
        self._pub_command = self.create_publisher(JointState, "/joint_command", 10)
        self._pub_markers = self.create_publisher(MarkerArray, "/brain/markers", 1)  # 01 §3.1
        latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self._pub_state = self.create_publisher(String, "/brain_state", latched)  # late subscribers: 01 §3.1
        # Timers, motion time and ages on the steady clock: a step back of the system clock (WSL2: ~2 s every
        # 30 s [MES-37]) would pause the timers, rewind the trajectory and hide a dead driver. Stamps stay on
        # the ROS clock.
        self._steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / END_EFFECTOR_POSE_HZ, self._publish_end_effector_pose, clock=self._steady)
        self.create_timer(1.0 / STREAM_HZ, self._tick, clock=self._steady)
        self.create_timer(1.0 / STATE_HZ, self._publish_state, clock=self._steady)
        self.create_service(GoToTarget, "/go_to_target", self._srv_go_to_target)
        self.create_service(Trigger, "/brain/start", self._srv_start)
        self.create_service(Trigger, "/brain/stop", self._srv_stop)
        if self._game:
            # Their own group: they reply at the end of the move, and a pending callback of the default group
            # froze the tick [MES-64]. Reentrant: a request during a move is refused at once, not queued.
            group = ReentrantCallbackGroup()
            self.create_service(Drop, "/brain/drop", self._srv_drop, callback_group=group)
            self.create_service(Trigger, "/brain/handover", self._srv_handover, callback_group=group)
            self.create_service(Drop, "/brain/pick", self._srv_pick, callback_group=group)
            for name in ("grid", "dispenser"):  # latched, as the board reader publishes them
                self.create_subscription(
                    PoseStamped,
                    f"/connect4/{name}_pose",
                    lambda msg, name=name: self._cb_frame(name, msg),
                    latched,
                )
        self._publish_state()

        self.get_logger().info("Brain node ready.")

    def _declare(self, name, default, description, low, high):
        """Read-only float or int parameter with its range; out of range, the node stops at startup."""
        descriptor = ParameterDescriptor(description=description, read_only=True)
        if isinstance(default, int):
            descriptor.integer_range = [IntegerRange(from_value=low, to_value=high, step=1)]
        else:
            descriptor.floating_point_range = [FloatingPointRange(from_value=low, to_value=high)]
        return self.declare_parameter(name, default, descriptor).value

    def _declare_pose(self, name, default, description):
        """Read-only arm pose, 5 joints within the URDF limits; otherwise the node stops at startup."""
        description += ", within the URDF limits"
        descriptor = ParameterDescriptor(description=description, read_only=True)
        pose = tuple(self.declare_parameter(name, list(default), descriptor).value)
        limits = [JOINT_LIMITS[joint] for joint in ARM_JOINT_NAMES]
        if len(pose) != len(limits) or not all(
            lo <= v <= hi for v, (lo, hi) in zip(pose, limits, strict=True)
        ):
            raise ValueError(f"{name} {pose}: {description}")
        return pose

    def _declare_planner(self):
        """The planner's grasp.* parameters (06 §6), as GraspPlanner kwargs; the defaults are its own."""
        d = {name: p.default for name, p in inspect.signature(GraspPlanner).parameters.items()}
        description = "Approach tilts from vertical, tried in order, deg, each in [0, 90)"
        descriptor = ParameterDescriptor(description=description, read_only=True)
        tilts = list(self.declare_parameter("grasp.tilts_deg", list(d["tilts_deg"]), descriptor).value)
        if not tilts or not all(0.0 <= t < 90.0 for t in tilts):
            raise ValueError(f"grasp.tilts_deg {tilts}: {description}")
        return {
            "tilts_deg": tilts,
            "pregrasp_backoff_m": self._declare(
                "grasp.pregrasp_backoff_m",
                d["pregrasp_backoff_m"],
                "Pre-grasp back-off along the approach, m",
                0.01,
                0.1,
            ),
            "lift_dz_m": self._declare(
                "grasp.lift_dz_m", d["lift_dz_m"], "Lift: TCP above the cube center, m", 0.05, 0.2
            ),
            "drop_tcp_z_m": self._declare(
                "grasp.drop_tcp_z_m", d["drop_tcp_z_m"], "Drop: TCP above the box floor, m", 0.05, 0.2
            ),
        }

    def _cb_joint_states(self, msg: JointState):
        """Stores the arm joints, the stamp, the reception time, the gripper effort (0 if not given) and the
        gripper opening (None if not given)."""
        positions = dict(zip(msg.name, msg.position, strict=True))
        if all(name in positions for name in ARM_JOINT_NAMES):
            efforts = dict(
                zip(msg.name, msg.effort, strict=False)
            )  # no effort on the real arm: the jaws' gap judges the grasp (#69)
            q = np.array([positions[name] for name in ARM_JOINT_NAMES])
            gripper = rad_to_pct(positions["gripper"]) if "gripper" in positions else None
            self._joints = (q, msg.header.stamp, self._now(), float(efforts.get("gripper", 0.0)), gripper)

    def _cb_frame(self, name, msg: PoseStamped):
        p, o = msg.pose.position, msg.pose.orientation
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = Rotation.from_quat([o.x, o.y, o.z, o.w]).as_matrix(), [p.x, p.y, p.z]
        setattr(self, f"_{name}", T)

    def _cb_object(self, msg: PoseStamped):
        """Stores the cube detection; orientation = the cube yaw about z (01 §3.1)."""
        o = msg.pose.orientation
        yaw = float(np.arctan2(2.0 * (o.w * o.z + o.x * o.y), 1.0 - 2.0 * (o.y * o.y + o.z * o.z)))
        self._cube = self._detection(msg, yaw) or self._cube

    def _cb_box(self, msg: PoseStamped):
        self._box = self._detection(msg) or self._box

    def _detection(self, msg, yaw=None):
        """A detection received now, or None if not in base_link (the contract's frame, 01 §3.1)."""
        if msg.header.frame_id != BASE_FRAME:
            self.get_logger().warning(
                f"detection in {msg.header.frame_id!r} ignored: only {BASE_FRAME}", throttle_duration_sec=5.0
            )
            return None
        p = msg.pose.position
        return Detection(np.array([p.x, p.y, p.z]), self._now(), yaw)

    def _publish_end_effector_pose(self):
        """30 Hz: FK of the last measured joints, stamped with their read time."""
        if self._joints is None:
            return
        q, stamp = self._joints[:2]
        pose = self._kin.fk(q)
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = BASE_FRAME
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(v) for v in pose[:3, 3])
        x, y, z, w = Rotation.from_matrix(pose[:3, :3]).as_quat()
        msg.pose.orientation.x, msg.pose.orientation.y = float(x), float(y)
        msg.pose.orientation.z, msg.pose.orientation.w = float(z), float(w)
        self._pub_pose.publish(msg)

    def _srv_go_to_target(self, request, response):
        """Position IK seeded with the measured joints; starts the motion and returns at once (06 §4)."""
        target = request.target_pose
        if self._fsm.state is not State.IDLE or self._start:  # one commander at a time: the cycle's FSM
            response.success = False
            response.message = f"brain in {self._fsm.label}: /go_to_target is served in IDLE only"
            return response
        if self._move is not None:
            response.success, response.message = False, "a move of the game is running"
            return response
        if target.header.frame_id not in ("", BASE_FRAME):
            response.success = False
            response.message = f"target in {target.header.frame_id!r}: only {BASE_FRAME} is supported"
            return response
        if self._joints is None:
            response.success, response.message = False, "no /joint_states yet"
            return response
        p = np.array([target.pose.position.x, target.pose.position.y, target.pose.position.z])
        q = self._kin.ik_pos(p, q_seed=self._joints[0])
        if q is None:  # ik_pos already rejects any solution whose FK misses p by more than 1 mm
            response.success, response.message = False, f"no IK solution for {np.round(p, 3).tolist()}"
            return response
        # From the measured position, not the last command: nothing jumps if the arm lagged (06 §5).
        # A new goal replaces the running motion.
        trajectory = min_jerk(self._joints[0], q, self._p.v_max, self._p.min_motion_s)
        self._motion = (trajectory, self._now())
        error = np.linalg.norm(self._kin.fk(q)[:3, 3] - p)
        response.success = True
        response.message = f"moving for {trajectory.duration:.2f} s, IK error {error * 1000:.2f} mm"
        self.get_logger().info(f"/go_to_target {np.round(p, 3).tolist()}: {response.message}")
        return response

    def _srv_start(self, request, response):
        """A cycle from IDLE, a re-arm from ERROR; rejected otherwise (C-35). The next tick acts on it."""
        if self._start or not self._fsm.accepts_start:
            response.success, response.message = False, f"rejected: brain in {self._fsm.label}"
            return response
        if self._game:  # no cube on the game's scene, and the way to PARK crosses the grid [MES-69]
            response.success, response.message = False, "rejected: game mode, no pick & place cycle"
            return response
        self._start = True
        response.success = True
        response.message = "re-arming: IDLE" if self._fsm.state is State.ERROR else "starting: WAIT_TARGET"
        return response

    def _srv_stop(self, request, response):
        """From any state: the arm freezes, STOPPED then IDLE (01 §3.2). The next tick acts on it."""
        self._stop = True
        response.success, response.message = True, "stopping: the arm freezes, then IDLE"
        return response

    async def _srv_drop(self, request, response):
        """One move of the game, from HANDOVER to the column and back (P2-02 §2.3): replies at its end."""
        return await self._from_handover("drop", drop.drop_steps, request, response)

    async def _srv_pick(self, request, response):
        """A token taken at a station of the dispenser, from HANDOVER and back (ADR-041): replies there."""
        return await self._from_handover("pick", drop.pick_steps, request, response)

    async def _from_handover(self, name, solve, request, response):
        """A move to the request's target that starts at HANDOVER, its steps from `solve`. With check_only
        nothing moves and the arm may be anywhere: the reply says whether the move would start. During a
        move a check is `busy`: its IK would hold the tick, so the 50 Hz commands, for half a second."""
        check, frame = request.check_only, self._grid if name == "drop" else self._dispenser
        moving = self._move is not None or self._motion is not None
        refusal, steps = ("busy" if moving else None) if check else self._refusal(), None
        handover = self._wait() if check else self._handover
        if refusal is None and not check and np.max(np.abs(self._joints[0] - handover)) > drop.AT_HANDOVER:
            refusal = "not_at_handover"
        if refusal is None and (self._grid is None or frame is None):
            refusal = "no_grid_pose" if self._grid is None else "no_dispenser_pose"
        if refusal is None and handover is None:
            refusal = "unreachable"  # the dispenser, so the place to wait at
        if refusal is None:  # IK inline, as /go_to_target: nothing moves yet, and no pose is solved later
            target = np.array([request.target.x, request.target.y, request.target.z])
            steps = solve(self._kin, target, handover, self._p)
            refusal = "unreachable" if not steps else None if drop.edge_on(frame, steps) else "not_edge_on"
        refusal = refusal or self._unsafe(name, steps, handover)
        self._solved = self._now()
        if check:
            reply = (refusal is None, refusal or "")
        else:
            reply = (False, refusal) if refusal else await self._begin(name, steps)
        response.success, response.message = reply
        return response

    async def _srv_handover(self, request, response):
        """The jaws open, the arm to where it waits: replies there. After a pick cut short the arm may be
        down in a station of the dispenser: it rises straight up first. Above the grid, where a drop cut
        short leaves it, the direct way is the measured one [MES-73]."""
        refusal, wait = self._refusal(), None
        if refusal is None:
            wait = self._wait()
            refusal = None if wait is not None else "unreachable"
        if refusal is None:
            out_of_a_station = (self._kin, self._joints[0]) if self._pick_cut else ()
            steps = drop.handover_steps(wait, self._p, *out_of_a_station)
            refusal = self._unsafe("handover", steps, self._joints[0])
            self._solved = self._now()
        if refusal is None:
            self._handover = wait  # where a pick and a drop start from, until the next handover
        reply = (False, refusal) if refusal else await self._begin("handover", steps)
        response.success, response.message = reply
        return response

    def _wait(self):
        """The joints the arm waits at: beside the dispenser where the camera sees it, or
        connect4.handover_pose while it has seen none (token_source: hand); None when out of reach."""
        if self._dispenser is None:
            return self._wait_seed
        return drop.wait_pose(self._kin, self._dispenser, self._wait_seed)

    def _unsafe(self, name, steps, q):
        """unsafe_path:<step>, the first of the steps from the joints `q` that passes nearer the measured
        grid than clearance.MARGIN; None when they keep clear, or with no grid to judge them against."""
        if self._arm is None or self._grid is None:
            return None
        gaps = clearance.along(self._arm, self._kin, q, steps, self._p.open_pct, self._grid, self._boxes)
        for step, gap in gaps:  # the first step that passes too near: the ones after it are not judged
            if gap < clearance.MARGIN:
                self.get_logger().warning(
                    f"/brain/{name}: unsafe_path:{step}, {gap * 1e3:.1f} mm from the grid"
                )
                return f"unsafe_path:{step}"
        return None

    def _refusal(self):
        """Why no move of the game can start now, or None. One commander at a time."""
        if self._fsm.state is not State.IDLE:  # the tick after a /brain/stop; no cycle starts in game mode
            return f"not_idle:{self._fsm.label}"
        if self._move is not None or self._motion is not None:
            return "busy"
        # Checked here, before any pose is solved: a dead driver is refused at once, nothing commanded. The
        # joints' age counts from the end of the last solve: a solve holds the executor, a check_only one too,
        # and the joints that came meanwhile are still queued when the next request is judged (#260).
        if self._joints is None or (
            self._now() - max(self._joints[2], self._solved) > self._p.max_joint_state_age_s
        ):
            return "no_joint_states"
        return None

    def _begin(self, name, steps) -> Future:
        """Hands the steps to the tick; the future gets (success, message) at the move's end."""
        self._move = {
            "name": name,
            "move": drop.Move(steps, self._p),
            "done": Future(),
            "start": self._now(),
            "ticks": [],  # tick periods meanwhile, s
        }
        return self._move["done"]

    def _step_move(self, now, period):
        """One tick of the running move: what it commands, or its end."""
        m = self._move
        q, _, received, effort, gripper = self._joints
        if self._fsm.state is not State.IDLE:  # /brain/stop: the FSM froze the arm
            return self._end_move("stopped")
        # The joints' age counts from the move's start: solving its poses held the executor, and the joints
        # that came meanwhile are still queued at the first tick (over 0.5 s on a slow machine, #252).
        if now - max(received, m["start"]) > self._p.max_joint_state_age_s:  # the FSM ignores them in IDLE
            return self._end_move("no_joint_states")  # no command: the driver is gone
        m["ticks"].append(period)
        out = m["move"].step(now, q, effort, gripper)
        if out.command is not None or out.gripper is not None:
            self._send_joint_command(out.command, out.gripper)
        if out.result is not None:
            self._end_move(out.result)

    def _end_move(self, result):
        """The move's reply and its log line: the seconds of each step ended, the tick periods meanwhile (the
        first one holds the IK)."""
        m, self._move = self._move, None
        self._pick_cut = m["name"] == "pick" and result != ""
        ticks = np.array(m["ticks"][1:] or [0.0]) * 1000
        text = f"done in {self._now() - m['start']:.1f} s" if result == "" else result
        steps = ", ".join(f"{name} {seconds:.1f}" for name, seconds in m["move"].seconds.items())
        line = (
            f"/brain/{m['name']}: {text} ({steps}); tick period p99 {np.percentile(ticks, 99):.1f} ms, "
            f"max {ticks.max():.1f} ms over {len(ticks)} ticks"
        )
        if result == "":
            self.get_logger().info(line)
        else:
            self.get_logger().warning(line)
        m["done"].set_result((result == "", text))

    def _tick(self):
        """50 Hz: one FSM step on the stored messages and the planner's answer, then what it commands;
        in IDLE, /go_to_target's motion or the game's move."""
        now, planning = self._now(), self._fsm.state is State.PLAN
        period = None if self._last_tick is None else now - self._last_tick
        if planning and period is not None:
            self._plan_ticks.append(period)
        self._last_tick = now
        answer, seconds = self._planning.result() if planning and self._planning.done() else (None, None)
        q, _, received, effort, gripper = self._joints or (None, None, -np.inf, 0.0, None)
        inputs = Inputs(
            q=q,
            joints_received=received,
            effort=effort,
            cube=self._cube,
            box=self._box,
            start=self._start,
            stop=self._stop,
            plan=answer,
            gripper=gripper,
        )
        self._start = self._stop = False
        out = self._fsm.step(now, inputs)
        if out.transition is not None:
            self._log_transition(out.transition)
        self._follow_planning(planning, answer, seconds, out.plan_request)
        if out.transition is not None and self._fsm.state is State.LIFT:  # held: the arm model shows it now
            self._pub_markers.publish(MarkerArray(markers=[Marker(ns="cube", action=Marker.DELETE)]))
        if self._move is not None:
            self._step_move(now, period)
        elif self._fsm.state is not State.IDLE:
            self._motion = None  # the cycle commands the arm: a /go_to_target motion ends
        elif self._motion is not None:
            self._stream_motion()
        if out.command is not None or out.gripper is not None:
            self._send_joint_command(out.command, out.gripper)

    def _publish_state(self):
        self._pub_state.publish(String(data=self._fsm.label))

    def _log_transition(self, text):
        """The FSM's transition line (a warning for an ERROR or a warning), then /brain_state.

        Two call sites: rclpy fixes the severity of each one ("Logger severity cannot be changed")."""
        if self._fsm.state is State.ERROR or "warning:" in text:
            self.get_logger().warning(text)
        else:
            self.get_logger().info(text)
        self._publish_state()

    def _follow_planning(self, planning, answer, seconds, request):
        """Out of PLAN: its log line and the markers of the plan followed; on a request: the worker."""
        if planning and self._fsm.state is not State.PLAN:
            self._end_plan(seconds)
            if self._fsm.state is State.APPROACH:
                self._publish_markers(answer[0])
        if request is not None:
            self._planning = self._worker.submit(timed_plan, *request)
            self._locked = request[:2]

    def _end_plan(self, seconds):
        """Out of PLAN: the planner's time and the tick periods meanwhile. A late answer is never read: PLAN
        is only entered with a new request, submitted on the same tick."""
        answered = "no answer" if seconds is None else f"planner answered in {seconds:.2f} s"
        longest = max(self._plan_ticks, default=0.0) * 1000
        self.get_logger().info(
            f"PLAN: {answered}; longest tick period {longest:.1f} ms over {len(self._plan_ticks)} ticks"
        )
        self._plan_ticks = []

    def _publish_markers(self, plan):
        """/brain/markers of the plan the cycle follows (06 §7), at the TCP of its configurations, and the
        locked cube: 5 cm, turned by its yaw when known."""
        stamp = self.get_clock().now().to_msg()
        tcp = {}
        for name, _ in MARKERS:
            x, y, z = (float(v) for v in self._kin.fk(getattr(plan, name))[:3, 3])
            tcp[name] = Point(x=x, y=y, z=z)

        def marker(ns, kind, rgb):
            m = Marker(ns=ns, type=kind, action=Marker.ADD)
            m.header.frame_id, m.header.stamp = BASE_FRAME, stamp
            m.pose.orientation.w = 1.0
            m.color.r, m.color.g, m.color.b, m.color.a = (*rgb, 1.0)
            return m

        markers = []
        for name, rgb in MARKERS:
            sphere = marker(name, Marker.SPHERE, rgb)
            sphere.pose.position = tcp[name]
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.02
            markers.append(sphere)
        arrow = marker("approach", Marker.ARROW, MARKERS[0][1])
        arrow.points = [tcp["pre_grasp"], tcp["grasp"]]
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.004, 0.008, 0.01  # shaft, head diameters, head length
        markers.append(arrow)
        (x, y, z), yaw = self._locked
        cube = marker("cube", Marker.CUBE, (1.0, 0.0, 0.0))
        cube.pose.position = Point(x=float(x), y=float(y), z=float(z))
        if yaw is not None:
            cube.pose.orientation.z, cube.pose.orientation.w = float(np.sin(yaw / 2)), float(np.cos(yaw / 2))
        cube.scale.x = cube.scale.y = cube.scale.z = 2.0 * float(z)  # z: its center, resting on the table
        markers.append(cube)
        self._pub_markers.publish(MarkerArray(markers=markers))

    def _stream_motion(self):
        """Next sample of the running /go_to_target trajectory; reached or timed out ends it."""
        trajectory, start = self._motion
        t = self._now() - start
        self._send_joint_command(trajectory.sample(t))
        if t < trajectory.duration or self._joints is None:
            return
        joint_error = float(np.max(np.abs(self._joints[0] - trajectory.q_goal)))
        if joint_error < self._p.reach_tol:
            tcp_error = np.linalg.norm(
                self._kin.fk(self._joints[0])[:3, 3] - self._kin.fk(trajectory.q_goal)[:3, 3]
            )
            self.get_logger().info(f"target reached: TCP {tcp_error * 1000:.2f} mm from the IK solution")
            self._motion = None
        elif t > trajectory.duration + self._p.motion_margin_s:
            self.get_logger().warning(
                f"target not reached at T + {self._p.motion_margin_s:g} s: joint error {joint_error:.3f} rad"
            )
            self._motion = None

    def _send_joint_command(self, q, gripper=None):
        """Partial command: the arm joints (rad) and/or the gripper (%); the others keep their goal (06 §5).
        Stamped: send time."""
        names, positions = [], []
        if q is not None:
            names, positions = list(ARM_JOINT_NAMES), [float(v) for v in q]
        if gripper is not None:
            names.append("gripper")
            positions.append(float(gripper))
        msg = JointState(name=names, position=positions)
        msg.header.stamp = self.get_clock().now().to_msg()
        self._pub_command.publish(msg)

    def _now(self) -> float:
        """Motion time, s: steady, never steps back."""
        return self._steady.now().nanoseconds * 1e-9

    def destroy_node(self):
        self._worker.shutdown(wait=False, cancel_futures=True)  # a running plan ends, then the process
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BrainNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
