"""Component test (launch_testing): sim.launch.py starts the driver and the brain, arguments forwarded."""

import os
import unittest

import launch
import launch.launch_description_sources
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.srv import GetParameters

STARTUP_TIMEOUT = 30.0  # s: MuJoCo loads the scene before the driver answers
PORT = "/dev/ttyTEST"  # forwarded to the driver, never opened in sim


@pytest.mark.launch_test
def generate_test_description():
    sim = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("so101_bringup"), "launch", "sim.launch.py")
        ),
        launch_arguments={"use_sim": "true", "port": PORT}.items(),
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
        use_sim, port = self.get_parameters("so101_driver", ["use_sim", "port"])
        self.assertTrue(use_sim.bool_value)
        self.assertEqual(port.string_value, PORT)

    def test_brain_is_started(self):
        (use_sim_time,) = self.get_parameters("so101_brain", ["use_sim_time"])
        self.assertFalse(use_sim_time.bool_value)
