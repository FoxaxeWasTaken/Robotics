#!/usr/bin/env python3
"""brain_node.py - intelligence of the arm: kinematics and the pick & place state machine.

Never talks to the backend: reads /joint_states, commands through /joint_command. ROS glue only (06 §2):
callbacks store the last message with its reception time, a 50 Hz tick feeds them to fsm.Fsm.step and sends
what it returns; the services answer at once, their effect shows on /brain_state (06 §4, ADR-017).
Publishes the TCP pose from forward kinematics. In IDLE, /go_to_target solves a position IK and returns at
once, the tick streams the min-jerk trajectory (04 §6).
"""

from pathlib import Path

import numpy as np
import rclpy
import rclpy.node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from so101_brain.fsm import Detection, Fsm, Inputs, Params, State
from so101_brain.kinematics import Kinematics
from so101_brain.trajectory import min_jerk
from so101_common.robot import ARM_JOINT_NAMES, BASE_FRAME, JOINT_LIMITS
from so101_interfaces.srv import GoToTarget

END_EFFECTOR_POSE_HZ = 30.0  # docs/project/01 §3.1
STREAM_HZ = 50.0  # the FSM tick, /joint_command during a motion (04 §6, 06 §4)
STATE_HZ = 1.0  # /brain_state, also on each transition (01 §3.1)
MIN_MOTION_DURATION = 0.5  # s: T_min of the min-jerk duration (04 §6)
TIMEOUT_MARGIN = 1.0  # s: not reached at T + this margin -> warning, motion dropped (04 §6)


class BrainNode(rclpy.node.Node):
    """Brain of the arm: kinematics, then pick & place."""

    def __init__(self):
        super().__init__("so101_brain")
        urdf = Path(get_package_share_directory("so101_description")) / "urdf" / "so101.urdf"
        self._kin = Kinematics(str(urdf))
        # Last /joint_states: (arm joints in ARM_JOINT_NAMES order, stamp, reception time, gripper effort).
        self._joints = None
        self._cube = self._box = None  # last /object_position, /drop_box_position (fsm.Detection)
        self._start = self._stop = False  # service requests, for the next tick
        self._motion = None  # running /go_to_target trajectory: (MinJerk, start time in s)
        # Startup-only parameters (06 §6), values in so101_bringup/config/brain.yaml.
        d = Params()
        self._v_max = self._declare(
            "motion.v_max_rad_s", d.v_max, "Peak arm joint speed of a motion, rad/s", 0.05, 1.5
        )
        self._reach_tol = self._declare(
            "motion.reach_tol_rad",
            d.reach_tol,
            "Target reached when every arm joint is this close, rad",
            0.001,
            0.1,
        )
        self._fsm = Fsm(
            Params(
                park_pose=self._declare_park_pose(d.park_pose),
                v_max=self._v_max,
                min_motion_s=MIN_MOTION_DURATION,
                reach_tol=self._reach_tol,
                motion_margin_s=TIMEOUT_MARGIN,
                open_pct=self._declare("grasp.open_pct", d.open_pct, "Gripper open, %", 0.0, 100.0),
                close_pct=self._declare(
                    "grasp.close_pct", d.close_pct, "Gripper closed on the cube, %", 0.0, 100.0
                ),
                effort_threshold=self._declare(
                    "grasp.effort_threshold",
                    d.effort_threshold,
                    "Normalized gripper closing force above which the cube is held (ADR-016)",
                    0.0,
                    1.0,
                ),
                target_lock_n=self._declare(
                    "fsm.target_lock_n", d.target_lock_n, "Consistent detections that lock the target", 1, 50
                ),
                target_lock_spread_m=self._declare(
                    "fsm.target_lock_spread_m",
                    d.target_lock_spread_m,
                    "The locking detections lie within this distance of each other, m",
                    0.0005,
                    0.05,
                ),
                max_retries=self._declare(
                    "fsm.max_retries", d.max_retries, "Failed attempts before ERROR:grasp_failed", 1, 10
                ),
            )
        )

        self.create_subscription(JointState, "/joint_states", self._cb_joint_states, 10)
        self.create_subscription(PoseStamped, "/object_position", self._cb_object, 10)
        self.create_subscription(PoseStamped, "/drop_box_position", self._cb_box, 10)
        self._pub_pose = self.create_publisher(PoseStamped, "/end_effector_pose", 10)
        self._pub_command = self.create_publisher(JointState, "/joint_command", 10)
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

    def _declare_park_pose(self, default):
        description = "PARK, the arm out of the camera view: 5 arm joints in rad, within the URDF limits"
        descriptor = ParameterDescriptor(description=description, read_only=True)
        park = tuple(self.declare_parameter("fsm.park_pose", list(default), descriptor).value)
        limits = [JOINT_LIMITS[name] for name in ARM_JOINT_NAMES]
        if len(park) != len(limits) or not all(
            lo <= v <= hi for v, (lo, hi) in zip(park, limits, strict=True)
        ):
            raise ValueError(f"fsm.park_pose {park}: {description}")
        return park

    def _cb_joint_states(self, msg: JointState):
        """Stores the arm joints, the stamp, the reception time and the gripper effort (0 if not given)."""
        positions = dict(zip(msg.name, msg.position, strict=True))
        if all(name in positions for name in ARM_JOINT_NAMES):
            efforts = dict(zip(msg.name, msg.effort, strict=False))  # effort is empty on the real arm (6.4a)
            q = np.array([positions[name] for name in ARM_JOINT_NAMES])
            self._joints = (q, msg.header.stamp, self._now(), float(efforts.get("gripper", 0.0)))

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
        trajectory = min_jerk(self._joints[0], q, self._v_max, MIN_MOTION_DURATION)
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
        self._start = True
        response.success = True
        response.message = "re-arming: IDLE" if self._fsm.state is State.ERROR else "starting: WAIT_TARGET"
        return response

    def _srv_stop(self, request, response):
        """From any state: the arm freezes, STOPPED then IDLE (01 §3.2). The next tick acts on it."""
        self._stop = True
        response.success, response.message = True, "stopping: the arm freezes, then IDLE"
        return response

    def _tick(self):
        """50 Hz: one FSM step on the stored messages, then what it commands; in IDLE, /go_to_target's motion.

        The planning worker comes with step 4.2b: until then nothing answers out.plan_request, and PLAN
        ends in ERROR:timeout:PLAN."""
        q, _, received, effort = self._joints or (None, None, -np.inf, 0.0)
        inputs = Inputs(q, received, effort, self._cube, self._box, self._start, self._stop)
        self._start = self._stop = False
        out = self._fsm.step(self._now(), inputs)
        if out.transition is not None:
            if self._fsm.state is State.ERROR or "warning:" in out.transition:
                self.get_logger().warning(out.transition)
            else:
                self.get_logger().info(out.transition)
            self._publish_state()
        if self._fsm.state is not State.IDLE:
            self._motion = None  # the cycle commands the arm: a /go_to_target motion ends
        elif self._motion is not None:
            self._stream_motion()
        if out.command is not None or out.gripper is not None:
            self._send_joint_command(out.command, out.gripper)

    def _publish_state(self):
        self._pub_state.publish(String(data=self._fsm.label))

    def _stream_motion(self):
        """Next sample of the running /go_to_target trajectory; reached or timed out ends it."""
        trajectory, start = self._motion
        t = self._now() - start
        self._send_joint_command(trajectory.sample(t))
        if t < trajectory.duration or self._joints is None:
            return
        joint_error = float(np.max(np.abs(self._joints[0] - trajectory.q_goal)))
        if joint_error < self._reach_tol:
            tcp_error = np.linalg.norm(
                self._kin.fk(self._joints[0])[:3, 3] - self._kin.fk(trajectory.q_goal)[:3, 3]
            )
            self.get_logger().info(f"target reached: TCP {tcp_error * 1000:.2f} mm from the IK solution")
            self._motion = None
        elif t > trajectory.duration + TIMEOUT_MARGIN:
            self.get_logger().warning(
                f"target not reached at T + {TIMEOUT_MARGIN:.0f} s: joint error {joint_error:.3f} rad"
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
