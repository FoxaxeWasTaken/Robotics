"""C-41 measurement in sim (docs/project/04 §7): 20 random targets through /go_to_target, driver + brain.

True TCP = MuJoCo FK (`gripperframe` site of the provided MJCF) of the measured /joint_states: independent of
the ikpy/URDF model the brain uses. Sim-backed, so here and not in so101_brain (C-05 guard).
The cube and the box are obstacles at these heights: /sim/randomize (seed 0 in driver.yaml) puts them at known
places, read from the ground truth (allowed in tests), and targets too close to them are drawn again.
"""

import time
import unittest
from pathlib import Path

import launch
import launch_ros.actions
import launch_testing.actions
import mujoco
import numpy as np
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from so101_sim import DEFAULT_SCENE
from std_srvs.srv import Trigger

from so101_common.robot import ARM_JOINT_NAMES, rad_to_pct
from so101_interfaces.srv import GoToTarget

STARTUP_TIMEOUT = 30.0  # s
N_TARGETS = 20
STILL = 1e-4  # rad: largest joint change between two /joint_states once settled
# s: the longest motion turns wrist_roll over its whole range, 5.6 rad at 1 rad/s peak: T = 10.5 s (04 §6)
SETTLE_TIMEOUT = 15.0
# m, horizontal, from the obstacle center: its half-diagonal + 4.5 cm for the jaws (cube 3.5 cm, box 7.4 cm)
CLEARANCE = {"object": 0.08, "drop_box": 0.12}


@pytest.mark.launch_test
def generate_test_description():
    config = Path(get_package_share_directory("so101_bringup")) / "config"  # as sim.launch.py, + ground truth
    driver = launch_ros.actions.Node(
        package="so101_driver",
        executable="driver",
        parameters=[str(config / "driver.yaml"), {"use_sim": True, "expose_ground_truth": True}],
    )
    brain = launch_ros.actions.Node(
        package="so101_brain", executable="brain", parameters=[str(config / "brain.yaml")]
    )
    return launch.LaunchDescription([driver, brain, launch_testing.actions.ReadyToTest()])


def targets(n, obstacles):
    """Seeded, in the workspace: r 0.16-0.30 m from base_link, azimuth -90° to 60° (cube and box areas,
    03 §7), z 3-12 cm (04 §7); clear of the obstacles {name: center xy}."""
    rng = np.random.default_rng(41)
    chosen = []
    while len(chosen) < n:
        r, a, z = rng.uniform(0.16, 0.30), np.radians(rng.uniform(-90.0, 60.0)), rng.uniform(0.03, 0.12)
        p = np.array([r * np.cos(a), r * np.sin(a), z])
        if all(np.linalg.norm(p[:2] - xy) > CLEARANCE[name] for name, xy in obstacles.items()):
            chosen.append(p)
    return chosen


class TestIkAccuracy(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_ik_accuracy")
        cls.states, cls.commands = [], []  # /joint_command as arrival times (monotonic)
        cls.node.create_subscription(JointState, "/joint_states", cls.states.append, 10)
        cls.node.create_subscription(
            JointState, "/joint_command", lambda _: cls.commands.append(time.monotonic()), 100
        )
        cls.pub = cls.node.create_publisher(JointState, "/joint_command", 10)
        cls.client = cls.node.create_client(GoToTarget, "/go_to_target")
        cls.truth = {name: [] for name in CLEARANCE}
        for name in CLEARANCE:
            cls.node.create_subscription(PoseStamped, f"/sim/ground_truth/{name}", cls.truth[name].append, 10)
        cls.model = mujoco.MjModel.from_xml_path(str(DEFAULT_SCENE))  # our own copy: FK only
        cls.data = mujoco.MjData(cls.model)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def wait_until(self, condition, timeout, message):
        end = time.monotonic() + timeout
        while not condition() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)
        self.assertTrue(condition(), message)

    def joints(self, msg):
        return dict(zip(msg.name, msg.position, strict=True))

    def settled(self):
        """The brain stopped streaming 0.3 s ago and the arm stays still over the last 10 /joint_states."""
        if not self.commands or time.monotonic() - self.commands[-1] < 0.3 or len(self.states) < 10:
            return False
        q = np.array([[self.joints(m)[n] for n in ARM_JOINT_NAMES] for m in self.states[-10:]])
        return float(np.max(np.abs(np.diff(q, axis=0)))) < STILL

    def measured_tcp(self):
        q = self.joints(self.states[-1])
        for name in ARM_JOINT_NAMES:
            self.data.qpos[self.model.joint(name).qposadr[0]] = q[name]
        mujoco.mj_kinematics(self.model, self.data)
        return self.data.site("gripperframe").xpos.copy()

    def obstacles_after_randomize(self):
        """Seeded /sim/randomize, then the cube and box centers (xy) from ground truth stamped after it."""
        randomize = self.node.create_client(Trigger, "/sim/randomize")
        self.assertTrue(randomize.wait_for_service(timeout_sec=STARTUP_TIMEOUT), "/sim/randomize not served")
        called = self.node.get_clock().now().nanoseconds * 1e-9
        future = randomize.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.assertTrue(future.result().success)

        def after(name):
            fresh = [
                m for m in self.truth[name] if m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 > called
            ]
            return fresh[-1] if fresh else None

        self.wait_until(lambda: all(after(name) for name in CLEARANCE), 5.0, "no ground truth")
        return {
            name: np.array([after(name).pose.position.x, after(name).pose.position.y]) for name in CLEARANCE
        }

    def test_1_twenty_random_targets_within_1_cm(self):
        self.assertTrue(self.client.wait_for_service(timeout_sec=STARTUP_TIMEOUT), "/go_to_target not served")
        self.wait_until(lambda: len(self.states) > 10, STARTUP_TIMEOUT, "no /joint_states")
        obstacles = self.obstacles_after_randomize()
        print(", ".join(f"{name} at {xy.round(3)}" for name, xy in obstacles.items()))
        print("target,x,y,z,measured_x,measured_y,measured_z,error_mm")
        errors = []
        for i, target in enumerate(targets(N_TARGETS, obstacles), start=1):
            request = GoToTarget.Request()
            request.target_pose.header.frame_id = "base_link"
            p = request.target_pose.pose.position
            p.x, p.y, p.z = (float(v) for v in target)
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
            self.assertTrue(
                future.result().success, f"target {i} {target.round(3)}: {future.result().message}"
            )
            self.wait_until(
                lambda: self.commands and time.monotonic() - self.commands[-1] < 0.1, 2.0, "no stream"
            )
            self.wait_until(self.settled, SETTLE_TIMEOUT, f"target {i} not settled")
            measured = self.measured_tcp()
            errors.append(float(np.linalg.norm(measured - target)))
            print(f"{i},{','.join(f'{v:.4f}' for v in (*target, *measured))},{errors[-1] * 1000:.3f}")
        errors_mm = np.array(errors) * 1000
        print(
            f"IK accuracy over {N_TARGETS} targets: mean {errors_mm.mean():.3f} mm, "
            f"p95 {np.percentile(errors_mm, 95):.3f} mm, max {errors_mm.max():.3f} mm"
        )
        self.assertLess(errors_mm.max(), 10.0)  # C-41: < 1 cm

    def test_2_gripper_follows_0_50_100_percent(self):
        self.wait_until(lambda: len(self.states) > 10, STARTUP_TIMEOUT, "no /joint_states")
        for pct in (0.0, 50.0, 100.0):

            def reached(pct=pct):
                """Republishes until reached: the first command may go out before discovery (idempotent)."""
                self.pub.publish(JointState(name=["gripper"], position=[pct]))
                return abs(rad_to_pct(self.joints(self.states[-1])["gripper"]) - pct) < 3.0

            self.wait_until(reached, 5.0, f"gripper never within 3 % of {pct:.0f} %")
            start = len(self.states)
            self.wait_until(lambda s=start: len(self.states) - s >= 15, 2.0, "no /joint_states")
            worst = max(abs(rad_to_pct(self.joints(m)["gripper"]) - pct) for m in self.states[start:])
            print(f"gripper {pct:.0f} %: measured within {worst:.2f} % over 15 /joint_states")
            self.assertLess(worst, 3.0)
