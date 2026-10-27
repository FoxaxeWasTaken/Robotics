"""Component test (launch_testing): sim driver publishes /joint_states and its camera, applies commands,
clamps them to the joint limits and rejects invalid ones (03 §4)."""

import math
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
from rcl_interfaces.msg import Log, ParameterType
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import CameraInfo, Image, JointState

STARTUP_TIMEOUT = 30.0  # s: MuJoCo loads the scene before the first message
ARM = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")  # URDF order, by hand
CLOSED, OPEN = -0.174533, 1.74533  # URDF gripper limits (rad): 0 % and 100 %
PAN_MAX = 1.91986  # URDF upper limit of shoulder_pan (rad)
CAMERA_FRAME = "external_cam_optical_frame"  # so101_common, by hand
# Scene fovy; principal point on OpenCV pixel centers, (W - 1)/2 and (H - 1)/2 [MES-41].
K_320X240 = [246.036, 0.0, 159.5, 0.0, 246.036, 119.5, 0.0, 0.0, 1.0]
PARAMETER_DEFAULTS = {  # docs/project/01 §3.3
    "control_rate_hz": 50.0,
    "joint_state_rate_hz": 30.0,
    "camera_rate_hz": 10.0,
    "max_joint_velocity": 1.5,
    "max_gripper_velocity": 200.0,
    "camera_width": 320,
    "camera_height": 240,
    "seed": 0,
    "max_relative_target_arm": 0.2,
    "max_relative_target_gripper": 25.0,
    "shutdown_pose": [1.9, -1.7, 1.5, 0.7, 0.0],  # = the brain's PARK
    "camera_device": "",  # the sim render; a webcam on the real arm (driver_real.yaml)
    "camera_info_file": "",
    "expose_ground_truth": False,  # tests and bench only (C-06)
}
VALUE_FIELD = {
    ParameterType.PARAMETER_BOOL: "bool_value",
    ParameterType.PARAMETER_INTEGER: "integer_value",
    ParameterType.PARAMETER_DOUBLE: "double_value",
    ParameterType.PARAMETER_DOUBLE_ARRAY: "double_array_value",
    ParameterType.PARAMETER_STRING: "string_value",
}


def header_key(msg):
    return msg.header.stamp.sec, msg.header.stamp.nanosec, msg.header.frame_id


@pytest.mark.launch_test
def generate_test_description():
    driver = launch_ros.actions.Node(
        package="so101_driver", executable="driver", parameters=[{"use_sim": True}], output="screen"
    )
    return launch.LaunchDescription([driver, launch_testing.actions.ReadyToTest()]), {"driver": driver}


class TestDriver(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_driver")
        cls.msgs = []
        cls.node.create_subscription(JointState, "/joint_states", cls.msgs.append, 10)
        cls.images = []
        cls.node.create_subscription(Image, "/external_cam/image_raw", cls.images.append, 10)
        cls.infos = []
        cls.node.create_subscription(CameraInfo, "/external_cam/camera_info", cls.infos.append, 10)
        cls.logs = []
        cls.node.create_subscription(Log, "/rosout", cls.logs.append, 100)
        cls.pub = cls.node.create_publisher(JointState, "/joint_command", 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def wait_for_joint_states(self):
        end = time.monotonic() + STARTUP_TIMEOUT
        while not self.msgs and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(self.msgs, "no /joint_states received")

    def wait_until(self, condition, timeout, message):
        end = time.monotonic() + timeout
        while not condition() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(condition(), message)

    def wait_for_command_subscriber(self):
        self.wait_until(lambda: self.pub.get_subscription_count() > 0, 5.0, "/joint_command not subscribed")

    def send_until_rejected(self, msg, reason):
        """Publishes msg until the driver logs its rejection: the warning is throttled to one per second."""
        start = len(self.logs)
        end = time.monotonic() + 5.0
        while time.monotonic() < end:
            self.pub.publish(msg)
            self.spin_for(0.3)
            if any(f"/joint_command rejected: {reason}" in log.msg for log in self.logs[start:]):
                return
        self.fail(f"no rejection logged for {reason!r}")

    def watch(self, count=30):
        """The next `count` /joint_states (1 s at 30 Hz). Counted, not timed: when the system clock steps
        back (WSL2 time sync), the driver's timers pause and a timed window can stay empty."""
        start = len(self.msgs)
        self.wait_until(lambda: len(self.msgs) - start >= count, 10.0, "no /joint_states while watching")
        return self.msgs[start:]

    def assert_no_motion(self, before):
        moved = max(abs(p - before[n]) for m in self.watch() for n, p in zip(m.name, m.position, strict=True))
        print(f"largest joint motion after the rejected command: {moved * 1000:.2f} mrad")
        self.assertLess(moved, 0.005)

    def latest(self):
        return dict(zip(self.msgs[-1].name, self.msgs[-1].position, strict=True))

    @staticmethod
    def crossing(samples, level):
        """Time at which the (t, position) samples first reach `level`, interpolated between two samples."""
        for (t0, p0), (t1, p1) in pairwise(samples):
            if p0 < level <= p1:
                return t0 + (level - p0) / (p1 - p0) * (t1 - t0)
        raise AssertionError(f"position {level:.3f} never reached")

    def test_parameters_are_declared_with_their_defaults(self):
        client = self.node.create_client(GetParameters, "/so101_driver/get_parameters")
        self.assertTrue(client.wait_for_service(timeout_sec=STARTUP_TIMEOUT), "/so101_driver not started")
        future = client.call_async(GetParameters.Request(names=list(PARAMETER_DEFAULTS)))
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        values = {
            name: getattr(v, VALUE_FIELD[v.type])
            for name, v in zip(PARAMETER_DEFAULTS, future.result().values, strict=True)
        }
        values["shutdown_pose"] = list(values["shutdown_pose"])  # array('d')
        self.assertEqual(values, PARAMETER_DEFAULTS)

    def test_sim_tools_by_default_randomize_but_no_ground_truth(self):
        def names(listing):
            return [name for name, _ in listing()]

        self.wait_until(
            lambda: (
                "/sim/randomize" in names(self.node.get_service_names_and_types)
                and "/external_cam/camera_info" in names(self.node.get_topic_names_and_types)
            ),
            STARTUP_TIMEOUT,
            "driver service and topics not discovered",
        )
        self.assertFalse([t for t in names(self.node.get_topic_names_and_types) if "ground_truth" in t])

    def test_joint_states_rate_is_at_least_20_hz(self):
        self.wait_for_joint_states()
        start = len(self.msgs)
        self.spin_for(5.0)
        stamps = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in self.msgs[start:]]
        rate = (len(stamps) - 1) / (stamps[-1] - stamps[0])
        print(f"/joint_states rate: {rate:.1f} Hz")
        # the nightly trend (#60, 08 §2): the timer period, camera on; its executor runs the control step
        p50, p99 = 1000 * np.percentile(np.diff(stamps), [50, 99])
        print(f"TREND control_period_ms p50={p50:.2f} p99={p99:.2f}")
        self.assertGreaterEqual(rate, 20.0)
        effort = self.msgs[-1].effort  # normalized load, same order as name (03 §5)
        self.assertEqual(len(effort), 6)
        self.assertTrue(all(-1.0 <= e <= 1.0 for e in effort))

    def test_partial_command_moves_only_the_named_joint(self):
        self.wait_for_joint_states()
        self.spin_for(1.0)  # let the arm settle on its startup pose
        before = self.latest()
        self.wait_for_command_subscriber()
        target = before["shoulder_pan"] + 0.5
        start = len(self.msgs)
        self.pub.publish(JointState(name=["shoulder_pan"], position=[target]))
        # 0.5 rad at 1.5 rad/s: 0.33 s, then settling
        self.wait_until(
            lambda: abs(self.latest()["shoulder_pan"] - target) < 0.005, 5.0, "target not reached"
        )
        after = self.latest()
        pan = [(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.position[0]) for m in self.msgs[start:]]
        # 20 % and 80 % of the move: always between two samples taken while moving (30 Hz, ~0.05 rad apart).
        t20, t80 = (self.crossing(pan, before["shoulder_pan"] + d) for d in (0.10, 0.40))
        speed = 0.3 / (t80 - t20)
        print(
            f"shoulder_pan {before['shoulder_pan']:.4f} -> {after['shoulder_pan']:.4f} (target {target:.4f})"
        )
        print(f"shoulder_pan mean speed 20-80 %: {speed:.2f} rad/s (limiter: 1.5)")
        self.assertAlmostEqual(after["shoulder_pan"], target, delta=0.02)
        # The limiter bounds the setpoint at 1.5 rad/s; the sim joint oscillates around that ramp
        # (peaks 2.5 rad/s, mean 1.48 over 20-80 %) and moves at 3.9 rad/s without the limiter [MES-33].
        self.assertLess(speed, 2.0)
        for name in before.keys() - {"shoulder_pan"}:
            self.assertLess(abs(after[name] - before[name]), 0.005, name)

    def test_unnamed_command_follows_joint_order_and_gripper_comes_back_in_rad(self):
        self.wait_for_joint_states()
        self.spin_for(1.0)
        arm = [self.latest()[n] for n in ARM]
        for pct, rad in ((100.0, OPEN), (0.0, CLOSED)):
            self.pub.publish(JointState(position=[*arm, pct]))  # no names: JOINT_NAMES order, gripper in %
            self.wait_until(lambda r=rad: abs(self.latest()["gripper"] - r) < 0.05, 5.0, "gripper not there")
            print(f"gripper {pct:.0f} % -> {self.latest()['gripper']:.4f} rad (URDF limit {rad})")
            self.assertAlmostEqual(self.latest()["gripper"], rad, delta=0.05)

    def test_out_of_range_command_stops_at_the_limit_without_winding_up(self):
        self.wait_for_joint_states()
        self.wait_for_command_subscriber()
        start_pan = self.latest()["shoulder_pan"]
        self.pub.publish(JointState(name=["shoulder_pan"], position=[5.0]))
        self.wait_until(lambda: self.latest()["shoulder_pan"] > PAN_MAX - 0.01, 5.0, "limit never reached")
        pan = [m.position[0] for m in self.watch()]  # holds at the limit
        print(f"shoulder_pan at the limit: {min(pan):.4f} to {max(pan):.4f} (URDF {PAN_MAX})")
        self.assertLess(max(abs(p - PAN_MAX) for p in pan), 0.01)
        # The sim's ctrlrange would stop the joint there too; the driver's clamp is what keeps the setpoint at
        # the limit: commanded back, the joint leaves at once instead of waiting for a setpoint at 5 rad.
        sent = self.node.get_clock().now().nanoseconds * 1e-9
        start = len(self.msgs)
        self.pub.publish(JointState(name=["shoulder_pan"], position=[start_pan]))
        self.wait_until(lambda: self.latest()["shoulder_pan"] < PAN_MAX - 0.05, 5.0, "never left the limit")
        left = next(m for m in self.msgs[start:] if m.position[0] < PAN_MAX - 0.05)
        delay = left.header.stamp.sec + left.header.stamp.nanosec * 1e-9 - sent
        print(f"commanded back: 0.05 rad away from the limit after {delay * 1000:.0f} ms")
        self.assertLess(delay, 0.3)  # 0.05 rad at 1.5 rad/s: 33 ms, plus the sim lag
        self.wait_until(lambda: abs(self.latest()["shoulder_pan"] - start_pan) < 0.01, 5.0, "start pose lost")

    def test_nan_command_is_rejected_whole(self):
        self.wait_for_joint_states()
        self.wait_for_command_subscriber()
        before = self.latest()
        command = JointState(name=["shoulder_pan", "elbow_flex"], position=[math.nan, 0.3])
        self.send_until_rejected(command, "non-finite position")
        self.assert_no_motion(before)

    def test_size_mismatch_command_is_rejected(self):
        self.wait_for_joint_states()
        self.wait_for_command_subscriber()
        before = self.latest()
        self.send_until_rejected(
            JointState(name=["shoulder_pan", "elbow_flex"], position=[0.3]), "2 names, 1"
        )
        self.assert_no_motion(before)

    def test_camera_publishes_rgb8_images(self):
        self.wait_for_joint_states()
        self.spin_for(3.0)  # the first frames render slower (OpenGL start-up) [MES-34]
        start = len(self.images)
        self.spin_for(3.0)
        stamps = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in self.images[start:]]
        self.assertGreater(len(stamps), 2, "no /external_cam/image_raw received")
        rate = (len(stamps) - 1) / (stamps[-1] - stamps[0])
        print(f"/external_cam/image_raw rate: {rate:.1f} Hz (target 10)")
        last = self.images[-1]
        self.assertEqual((last.encoding, last.width, last.height), ("rgb8", 320, 240))
        self.assertEqual(last.header.frame_id, CAMERA_FRAME)
        self.assertEqual(len(last.data), 320 * 240 * 3)
        # Alive and periodic, not a performance check: 10 Hz on a developer machine, 4.9 Hz on the
        # 4-vCPU CI runner (2 render threads, coverage on) [MES-34]; a period bug would give 1 Hz.
        self.assertGreaterEqual(rate, 3.0)

    def test_camera_info_shares_the_header_of_each_image(self):
        self.wait_for_joint_states()
        self.spin_for(3.0)  # the first frames render slower (OpenGL start-up) [MES-34]
        start = len(self.infos)
        self.spin_for(3.0)
        infos = self.infos[start:]
        self.assertGreater(len(infos), 2, "no /external_cam/camera_info received")
        image_headers = {header_key(m) for m in self.images}
        for info in infos[:-1]:  # the image of the last one may still be in flight
            self.assertIn(header_key(info), image_headers)
        info = infos[-1]
        self.assertEqual(info.header.frame_id, CAMERA_FRAME)
        self.assertEqual((info.width, info.height, info.distortion_model), (320, 240, "plumb_bob"))
        for got, expected in zip(info.k, K_320X240, strict=True):
            self.assertAlmostEqual(got, expected, delta=0.01)
        self.assertEqual(list(info.d), [0.0] * 5)
        self.assertEqual(list(info.r), [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        k = list(info.k)
        self.assertEqual(list(info.p), [*k[0:3], 0.0, *k[3:6], 0.0, *k[6:9], 0.0])


@launch_testing.post_shutdown_test()
class TestDriverShutdown(unittest.TestCase):
    def test_sigint_parks_the_arm_then_stops_the_driver_without_traceback(
        self, proc_info, proc_output, driver
    ):
        launch_testing.asserts.assertExitCodes(proc_info, process=driver)
        output = b"".join(io.text for io in proc_output[driver])
        print([line for line in output.decode().splitlines() if "Shutdown pose" in line])
        # Moved before disconnect() cuts the torque (03 §8). Reached, or given up after 4 s: from the startup
        # pose the pan alone needs 3.8 s, too close on a loaded runner (test_shutdown.py checks the move).
        self.assertIn(b"Shutdown pose", output)
        self.assertNotIn(b"Traceback", output)
