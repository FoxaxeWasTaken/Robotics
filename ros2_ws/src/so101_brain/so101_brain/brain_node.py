#!/usr/bin/env python3
"""brain_node.py - intelligence of the arm: kinematics, then pick & place.

Never talks to the backend: reads /joint_states, commands through /joint_command.
Publishes the TCP pose from forward kinematics; /go_to_target solves a position IK and returns at
once, a 50 Hz timer streams the min-jerk trajectory (docs/project/04 §6, 06 §4-§5). Later: pick & place.
"""

from pathlib import Path

import numpy as np
import rclpy
import rclpy.node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState

from so101_brain.kinematics import Kinematics
from so101_brain.trajectory import min_jerk
from so101_common.robot import ARM_JOINT_NAMES, BASE_FRAME
from so101_interfaces.srv import GoToTarget

END_EFFECTOR_POSE_HZ = 30.0  # docs/project/01 §3.1
STREAM_HZ = 50.0  # /joint_command during a motion (04 §6)
MIN_MOTION_DURATION = 0.5  # s: T_min of the min-jerk duration (04 §6)
TIMEOUT_MARGIN = 1.0  # s: not reached at T + this margin -> warning, motion dropped (04 §6)


class BrainNode(rclpy.node.Node):
    """Brain of the arm: kinematics, then pick & place."""

    def __init__(self):
        super().__init__("so101_brain")
        urdf = Path(get_package_share_directory("so101_description")) / "urdf" / "so101.urdf"
        self._kin = Kinematics(str(urdf))
        self._joints = None  # last /joint_states: (arm joints in ARM_JOINT_NAMES order, stamp)
        self._motion = None  # running trajectory: (MinJerk, start time in s)
        # Startup-only parameters (06 §6), values in so101_bringup/config/brain.yaml.
        self._v_max = self._declare(
            "motion.v_max_rad_s", 1.0, "Peak arm joint speed of a motion, rad/s", 0.05, 1.5
        )
        self._reach_tol = self._declare(
            "motion.reach_tol_rad", 0.01, "Target reached when every arm joint is this close, rad", 0.001, 0.1
        )

        self.create_subscription(JointState, "/joint_states", self._cb_joint_states, 10)
        self._pub_pose = self.create_publisher(PoseStamped, "/end_effector_pose", 10)
        self._pub_command = self.create_publisher(JointState, "/joint_command", 10)
        self.create_timer(1.0 / END_EFFECTOR_POSE_HZ, self._publish_end_effector_pose)
        self.create_timer(1.0 / STREAM_HZ, self._stream_motion)
        self.create_service(GoToTarget, "/go_to_target", self._srv_go_to_target)

        self.get_logger().info("Brain node ready.")

    def _declare(self, name, default, description, low, high):
        """Read-only float parameter with its range; an out-of-range value stops the node at startup."""
        descriptor = ParameterDescriptor(
            description=description,
            read_only=True,
            floating_point_range=[FloatingPointRange(from_value=low, to_value=high)],
        )
        return self.declare_parameter(name, default, descriptor).value

    def _cb_joint_states(self, msg: JointState):
        """Stores the arm joints only."""
        positions = dict(zip(msg.name, msg.position, strict=True))
        if all(name in positions for name in ARM_JOINT_NAMES):
            self._joints = (np.array([positions[name] for name in ARM_JOINT_NAMES]), msg.header.stamp)

    def _publish_end_effector_pose(self):
        """30 Hz: FK of the last measured joints, stamped with their read time."""
        if self._joints is None:
            return
        q, stamp = self._joints
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

    def _stream_motion(self):
        """50 Hz: next sample of the running trajectory; reached or timed out ends it."""
        if self._motion is None:
            return
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

    def _send_joint_command(self, q):
        """Arm joints only: a partial command, the gripper keeps its goal. Stamped: send time."""
        msg = JointState(name=list(ARM_JOINT_NAMES), position=[float(v) for v in q])
        msg.header.stamp = self.get_clock().now().to_msg()
        self._pub_command.publish(msg)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


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
