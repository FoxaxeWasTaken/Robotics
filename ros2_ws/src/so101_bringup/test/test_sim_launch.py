"""Component test (launch_testing): sim.launch.py starts the driver and the brain, arguments forwarded."""

import os
import time
import unittest

import launch
import launch.launch_description_sources
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.srv import GetParameters
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

STARTUP_TIMEOUT = 30.0  # s: MuJoCo loads the scene before the driver answers
PORT = "/dev/ttyTEST"  # forwarded to the driver, never opened in sim


@pytest.mark.launch_test
def generate_test_description():
    sim = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("so101_bringup"), "launch", "sim.launch.py")
        ),
        launch_arguments={"use_sim": "true", "port": PORT, "expose_ground_truth": "true"}.items(),
    )
    return launch.LaunchDescription([sim, launch_testing.actions.ReadyToTest()])


class TestSimLaunch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_sim_launch")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def get_parameters(self, node_name, names):
        client = self.node.create_client(GetParameters, f"/{node_name}/get_parameters")
        self.assertTrue(client.wait_for_service(timeout_sec=STARTUP_TIMEOUT), f"/{node_name} not started")
        future = client.call_async(GetParameters.Request(names=names))
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.assertIsNotNone(future.result(), f"/{node_name} did not answer")
        return future.result().values

    def test_driver_gets_the_forwarded_arguments(self):
        use_sim, port, truth = self.get_parameters("so101_driver", ["use_sim", "port", "expose_ground_truth"])
        self.assertTrue(use_sim.bool_value)
        self.assertEqual(port.string_value, PORT)
        self.assertTrue(truth.bool_value)  # the bench's switch (08 §3); false unless asked

    def test_brain_is_started(self):
        (use_sim_time,) = self.get_parameters("so101_brain", ["use_sim_time"])
        self.assertFalse(use_sim_time.bool_value)

    def test_the_ghost_arm_follows_the_driver_setpoint(self):
        """#200: /joint_setpoint (6 joints, rad) goes to a commanded pan; the ghost's TF tree exists."""
        setpoints = []
        self.node.create_subscription(
            JointState, "/joint_setpoint", setpoints.append, qos_profile_sensor_data
        )
        command = self.node.create_publisher(JointState, "/joint_command", 10)
        tf = Buffer()
        TransformListener(tf, self.node)

        def pan():
            return dict(zip(setpoints[-1].name, setpoints[-1].position, strict=True))["shoulder_pan"]

        deadline = time.monotonic() + STARTUP_TIMEOUT
        while not (setpoints and abs(pan() - 0.5) < 1e-3) and time.monotonic() < deadline:
            command.publish(JointState(name=["shoulder_pan"], position=[0.5]))
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(setpoints, "no /joint_setpoint")
        self.assertEqual(len(setpoints[-1].name), 6)
        self.assertAlmostEqual(pan(), 0.5, places=3)
        while not tf.can_transform("base_link", "ghost/gripper_frame_link", rclpy.time.Time()):
            self.assertLess(time.monotonic(), deadline, "no TF base_link -> ghost/gripper_frame_link")
            rclpy.spin_once(self.node, timeout_sec=0.1)
