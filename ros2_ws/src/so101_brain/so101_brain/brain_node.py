#!/usr/bin/env python3
"""brain_node.py - intelligence of the arm: kinematics, then pick & place.

Never talks to the backend: reads /joint_states, commands through /joint_command.
Publishes the TCP pose from forward kinematics and serves /go_to_target with position IK
(docs/project/04, 06). Later: pick & place state machine.
"""

from pathlib import Path

import numpy as np
import rclpy
import rclpy.node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState

from so101_brain.kinematics import Kinematics
from so101_common.robot import ARM_JOINT_NAMES, BASE_FRAME
from so101_interfaces.srv import GoToTarget

END_EFFECTOR_POSE_HZ = 30.0  # docs/project/01 §3.1


class BrainNode(rclpy.node.Node):
    """Brain of the arm: kinematics, then pick & place."""

    def __init__(self):
        super().__init__("so101_brain")
        urdf = Path(get_package_share_directory("so101_description")) / "urdf" / "so101.urdf"
        self._kin = Kinematics(str(urdf))
        self._joints = None  # last /joint_states: (arm joints in ARM_JOINT_NAMES order, stamp)

        self.create_subscription(JointState, "/joint_states", self._cb_joint_states, 10)
        self._pub_pose = self.create_publisher(PoseStamped, "/end_effector_pose", 10)
        self._pub_command = self.create_publisher(JointState, "/joint_command", 10)
        self.create_timer(1.0 / END_EFFECTOR_POSE_HZ, self._publish_end_effector_pose)
        self.create_service(GoToTarget, "/go_to_target", self._srv_go_to_target)

        self.get_logger().info("Brain node ready.")

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
        """Position IK seeded with the measured joints; the joint target goes to /joint_command."""
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
        self._send_joint_command(q)
        error = np.linalg.norm(self._kin.fk(q)[:3, 3] - p)
        response.success, response.message = True, f"IK error {error * 1000:.2f} mm"
        self.get_logger().info(f"/go_to_target {np.round(p, 3).tolist()}: {response.message}")
        return response

    def _send_joint_command(self, q):
        """Arm joints only: a partial command, the gripper keeps its goal."""
        self._pub_command.publish(JointState(name=list(ARM_JOINT_NAMES), position=[float(v) for v in q]))


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
