"""Component test (launch_testing): brain_node alone. A fake arm in the test echoes /joint_command into
/joint_states, so no driver and no sim are needed (the sim-backed version is in so101_bringup).
No perception either: a fake one publishes the cube and the box when a test asks; the real planner runs."""

import re
import time
import unittest
from itertools import pairwise

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.clock import Clock, ClockType
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from so101_common.robot import pct_to_rad
from so101_interfaces.srv import GoToTarget

STARTUP_TIMEOUT = 20.0  # s
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
V_MAX = 1.0  # rad/s: default of motion.v_max_rad_s
PARK = [1.9, -1.7, 1.5, 0.7, 0.0]  # default of fsm.park_pose
CUBE, BOX = (0.22, 0.05, 0.025), (0.05, -0.25, 0.0)  # where the fake perception sees them by default
LATCHED = QoSProfile(
    depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
)


@pytest.mark.launch_test
def generate_test_description():
    brain = launch_ros.actions.Node(package="so101_brain", executable="brain", output="screen")
    return launch.LaunchDescription([brain, launch_testing.actions.ReadyToTest()]), {"brain": brain}


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class TestBrain(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_brain")
        cls.q = dict.fromkeys(JOINTS, 0.0)  # the fake arm: at q = 0, follows every command at once
        cls.alive = True  # the fake driver publishes /joint_states
        cls.commands, cls.arrivals, cls.poses, cls.sent, cls.markers = [], [], [], [], []
        cls.states = []  # /brain_state as (arrival, monotonic s; data)
        cls.pub_states = cls.node.create_publisher(JointState, "/joint_states", 10)
        cls.node.create_subscription(JointState, "/joint_command", cls.on_command, 100)
        cls.node.create_subscription(PoseStamped, "/end_effector_pose", cls.poses.append, 10)
        cls.node.create_subscription(MarkerArray, "/brain/markers", cls.markers.append, 10)
        cls.node.create_subscription(
            String, "/brain_state", lambda m: cls.states.append((time.monotonic(), m.data)), LATCHED
        )
        # Steady clock: a WSL2 clock step would pause the fake driver, and the brain would see it as dead.
        cls.node.create_timer(1.0 / 30.0, cls.publish_states, clock=Clock(clock_type=ClockType.STEADY_TIME))
        cls.client = cls.node.create_client(GoToTarget, "/go_to_target")
        cls.pub_cube = cls.node.create_publisher(PoseStamped, "/object_position", 1)
        cls.pub_box = cls.node.create_publisher(PoseStamped, "/drop_box_position", 1)
        cls.cube_frame, cls.cube_at = (
            None,
            CUBE,
        )  # the fake perception: publishes at 10 Hz once a frame is set
        cls.node.create_timer(0.1, cls.publish_detections, clock=Clock(clock_type=ClockType.STEADY_TIME))
        cls.start = cls.node.create_client(Trigger, "/brain/start")
        cls.stop = cls.node.create_client(Trigger, "/brain/stop")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def on_command(cls, msg):
        cls.commands.append(msg)
        cls.arrivals.append(time.monotonic())
        positions = dict(zip(msg.name, msg.position, strict=True))
        if "gripper" in positions:  # % in /joint_command, rad in /joint_states, as the driver (01 §3.1)
            positions["gripper"] = pct_to_rad(positions["gripper"])
        cls.q.update(positions)

    @classmethod
    def publish_detections(cls):
        if cls.cube_frame is None:
            return
        for pub, frame, xyz, yaw in (
            (cls.pub_cube, cls.cube_frame, cls.cube_at, 0.3),
            (cls.pub_box, "base_link", BOX, 0.0),
        ):
            msg = PoseStamped()
            msg.header.frame_id = frame
            msg.header.stamp = cls.node.get_clock().now().to_msg()
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = xyz
            msg.pose.orientation.z, msg.pose.orientation.w = np.sin(yaw / 2), np.cos(yaw / 2)
            pub.publish(msg)

    @classmethod
    def publish_states(cls):
        if not cls.alive:
            return
        cls.last_state_sent = time.monotonic()
        msg = JointState(name=JOINTS, position=[cls.q[n] for n in JOINTS])
        msg.header.stamp = cls.node.get_clock().now().to_msg()
        cls.sent.append(msg.header.stamp)
        cls.pub_states.publish(msg)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def wait_for_pose(self):
        end = time.monotonic() + STARTUP_TIMEOUT
        while not self.poses and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(self.poses, "no /end_effector_pose")

    def call(self, frame_id, xyz):
        self.assertTrue(self.client.wait_for_service(timeout_sec=STARTUP_TIMEOUT), "/go_to_target not served")
        request = GoToTarget.Request()
        request.target_pose.header.frame_id = frame_id
        p = request.target_pose.pose.position
        p.x, p.y, p.z = xyz
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.assertIsNotNone(future.result(), "no answer from /go_to_target")
        return future.result()

    def trigger(self, client):
        self.assertTrue(client.wait_for_service(timeout_sec=STARTUP_TIMEOUT), f"{client.srv_name} not served")
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.assertIsNotNone(future.result(), f"no answer from {client.srv_name}")
        return future.result()

    def wait_for_fresh_joint_states(self):
        """Until a /end_effector_pose comes from /joint_states sent from now on: the brain sees the fake
        driver again (a start before that is rightly refused, see test_7)."""
        sent, end = len(self.sent), time.monotonic() + STARTUP_TIMEOUT
        while not any(pose.header.stamp in self.sent[sent:] for pose in self.poses[-3:]):
            self.assertLess(time.monotonic(), end, "the brain does not see /joint_states")
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def wait_for_state(self, data, timeout=5.0):
        """Arrival time of the first /brain_state `data` received from now on."""
        seen, end = len(self.states), time.monotonic() + timeout
        while time.monotonic() < end:
            for arrival, state in self.states[seen:]:
                if state == data:
                    return arrival
            rclpy.spin_once(self.node, timeout_sec=0.01)
        self.fail(f"no /brain_state {data} in {timeout} s: {[s for _, s in self.states[-5:]]}")

    def arm(self):
        return np.array([self.q[n] for n in JOINTS[:5]])

    def tcp(self):
        p = self.poses[-1].pose.position
        return np.array([p.x, p.y, p.z])

    def test_1_pose_is_the_fk_of_the_joint_states(self):
        self.wait_for_pose()
        pose = self.poses[-1]
        self.assertEqual(pose.header.frame_id, "base_link")
        self.assertIn(pose.header.stamp, self.sent)  # stamped with the joint states it comes from
        np.testing.assert_allclose(self.tcp(), [0.391, 0.0, 0.226], atol=1e-3)  # q = 0 (04 §1.1)

    def test_2_go_to_target_streams_until_reached(self):
        self.wait_for_pose()
        target = np.array([0.2, 0.0, 0.1])
        start = len(self.commands)
        response = self.call("base_link", target)
        self.assertTrue(response.success, response.message)
        end = time.monotonic() + 6.0
        while np.linalg.norm(self.tcp() - target) > 1e-3 and time.monotonic() < end:
            self.spin_for(0.1)
        self.assertLess(np.linalg.norm(self.tcp() - target), 1e-3)
        streamed = self.commands[start:]
        self.assertGreater(len(streamed), 20)
        self.assertTrue(
            all(list(m.name) == JOINTS[:5] for m in streamed)
        )  # arm only: the gripper keeps its goal
        speeds = [
            np.max(np.abs(np.subtract(b.position, a.position))) / (stamp(b) - stamp(a))
            for a, b in pairwise(streamed)
        ]
        self.assertLessEqual(max(speeds), V_MAX * 1.05)

    def test_3_rejected_targets(self):
        self.wait_for_pose()
        self.assertFalse(self.call("world", (0.2, 0.0, 0.1)).success)
        self.assertFalse(self.call("base_link", (1.0, 0.0, 0.0)).success)

    def test_4_start_parks_the_arm_and_rejects_a_second_start(self):
        self.wait_for_pose()
        response = self.trigger(self.start)
        self.assertTrue(response.success, response.message)
        self.wait_for_state("WAIT_TARGET")
        self.assertFalse(self.trigger(self.start).success)  # busy: rejected (C-35)
        self.assertFalse(self.call("base_link", (0.2, 0.0, 0.1)).success)  # /go_to_target: in IDLE only
        end = time.monotonic() + 6.0
        while np.max(np.abs(self.arm() - PARK)) > 0.01 and time.monotonic() < end:
            self.spin_for(0.1)
        np.testing.assert_allclose(self.arm(), PARK, atol=0.01)

    def test_5_stop_freezes_then_idles(self):
        self.wait_for_state("WAIT_TARGET")  # latched: the current state, still waiting for a target
        self.spin_for(0.5)  # the end of the park motion
        sent = len(self.commands)
        response = self.trigger(self.stop)
        self.assertTrue(response.success, response.message)
        self.wait_for_state("STOPPED")
        self.wait_for_state("IDLE")
        self.spin_for(0.2)
        [freeze] = self.commands[sent:]  # one command: the measured position as the target
        self.assertEqual(list(freeze.name), JOINTS[:5])
        np.testing.assert_allclose(freeze.position, PARK, atol=0.01)
        self.assertTrue(self.call("base_link", (0.2, 0.0, 0.1)).success)  # IDLE again: /go_to_target served

    def test_6_a_late_subscriber_gets_the_current_state(self):
        [info] = self.node.get_publishers_info_by_topic("/brain_state")
        self.assertEqual(info.qos_profile.durability, DurabilityPolicy.TRANSIENT_LOCAL)  # 01 §3.1
        self.assertEqual(info.qos_profile.reliability, ReliabilityPolicy.RELIABLE)
        late = []
        self.node.create_subscription(String, "/brain_state", late.append, LATCHED)
        end = time.monotonic() + 2.0
        while not late and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)
        self.assertEqual([m.data for m in late][:1], ["IDLE"])

    def test_7_joint_states_lost_in_a_cycle_stop_the_brain_within_1_s(self):
        self.assertTrue(self.trigger(self.start).success)
        self.wait_for_state("WAIT_TARGET")
        type(self).alive = False
        try:
            error = self.wait_for_state("ERROR:no_joint_states")
            self.assertLess(error - self.last_state_sent, 1.0)
            self.spin_for(1.0)
            self.assertFalse([a for a in self.arrivals if a > error + 0.2])  # the commands stop
        finally:
            type(self).alive = True
        response = self.trigger(self.start)
        self.assertTrue(response.success, response.message)  # re-arms
        self.wait_for_state("IDLE")

    def test_8_published_detections_lock_the_target(self, proc_output, brain):
        def wait_for_log(pattern, timeout=2.0):  # spins meanwhile: assertWaitFor would stop the fake arm
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                text = b"".join(io.text for io in proc_output[brain]).decode()
                if re.search(pattern, text):
                    return
                rclpy.spin_once(self.node, timeout_sec=0.01)
            self.fail(f"no {pattern!r} in the brain's output")

        self.wait_for_fresh_joint_states()  # test_7 stopped the fake driver
        type(self).cube_frame = "world"  # out of the contract: ignored
        try:
            self.assertTrue(self.trigger(self.start).success)
            end = time.monotonic() + 6.0
            while np.max(np.abs(self.arm() - PARK)) > 0.01 and time.monotonic() < end:
                self.spin_for(0.1)
            self.spin_for(1.0)  # parked, the cube only in the wrong frame: no lock
            self.assertEqual(self.states[-1][1], "WAIT_TARGET")
            wait_for_log("detection in 'world' ignored")
            type(self).cube_frame = "base_link"
            self.wait_for_state("PLAN", timeout=2.0)
            wait_for_log(r"target locked: cube \[0\.22\s+0\.05\s+0\.025\], yaw 0\.30 rad, box \[\s*0\.05")
            self.wait_for_state("APPROACH", timeout=4.0)  # planned off the tick
            wait_for_log(
                r"PLAN: planner answered in \d+\.\d\d s; longest tick period \d+\.\d ms over \d+ ticks"
            )
        finally:
            type(self).cube_frame = None
        self.check_markers()
        streamed = len(self.commands)
        self.spin_for(0.5)
        self.assertGreater(len(self.commands) - streamed, 15, "APPROACH streams at 50 Hz")
        self.wait_for_state("DESCEND", timeout=8.0)  # the arm at the pre-grasp and the gripper read open
        self.assertTrue(self.trigger(self.stop).success)
        self.wait_for_state("IDLE")

    def check_markers(self):
        """/brain/markers of the plan (06 §7): pre-grasp yellow, grasp green, drop blue spheres, approach
        arrow, at the TCP positions of the plan's configurations (IK checked by FK within 1 mm)."""
        end = time.monotonic() + 2.0
        while not self.markers and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(self.markers, "no /brain/markers")
        markers = {m.ns: m for m in self.markers[-1].markers}
        self.assertEqual(sorted(markers), ["approach", "drop", "grasp", "pre_grasp"])
        self.assertEqual({m.header.frame_id for m in markers.values()}, {"base_link"})
        colors = {
            ns: (m.color.r, m.color.g, m.color.b) for ns, m in markers.items() if m.type == Marker.SPHERE
        }
        self.assertEqual(
            colors, {"pre_grasp": (1.0, 1.0, 0.0), "grasp": (0.0, 1.0, 0.0), "drop": (0.0, 0.0, 1.0)}
        )

        def xyz(point):
            return np.array([point.x, point.y, point.z])

        grasp, pre, drop = (xyz(markers[ns].pose.position) for ns in ("grasp", "pre_grasp", "drop"))
        np.testing.assert_allclose(grasp, CUBE, atol=0.002)  # TCP at the cube center
        np.testing.assert_allclose(drop, np.add(BOX, (0.0, 0.0, 0.09)), atol=0.002)  # 9 cm above the floor
        self.assertAlmostEqual(np.linalg.norm(pre - grasp), 0.04, delta=0.002)  # 4 cm back-off
        arrow = markers["approach"]
        self.assertEqual(arrow.type, Marker.ARROW)
        np.testing.assert_allclose([xyz(p) for p in arrow.points], [pre, grasp], atol=1e-9)

    def test_9_an_unreachable_cube_is_an_error_without_any_command(self):
        type(self).cube_at, type(self).cube_frame = (0.5, 0.0, 0.025), "base_link"
        try:
            self.assertTrue(self.trigger(self.start).success)
            planning = self.wait_for_state("PLAN", timeout=10.0)
            self.wait_for_state("ERROR:unreachable", timeout=4.0)
        finally:
            type(self).cube_frame, type(self).cube_at = None, CUBE
        self.assertFalse([a for a in self.arrivals if a > planning], "a command after PLAN")
        self.assertTrue(self.trigger(self.start).success)  # re-arms
        self.wait_for_state("IDLE")


@launch_testing.post_shutdown_test()
class TestBrainShutdown(unittest.TestCase):
    def test_sigint_stops_the_brain_without_traceback(self, proc_info, proc_output, brain):
        launch_testing.asserts.assertExitCodes(proc_info, process=brain)
        self.assertNotIn(b"Traceback", b"".join(io.text for io in proc_output[brain]))
