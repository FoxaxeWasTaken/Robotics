"""Component test (launch_testing): a backward step of the ROS clock does not rewind a motion (#112).

The brain runs with use_sim_time: its ROS clock is the /clock published here, which steps back 2 s in
mid-motion, like the WSL2 system clock every 30 s. A fake arm echoes /joint_command into /joint_states.
"""

import time
import unittest
from itertools import pairwise

import launch
import launch_ros.actions
import launch_testing.actions
import numpy as np
import pytest
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from rclpy.clock import Clock, ClockType
from rosgraph_msgs.msg import Clock as ClockMsg
from sensor_msgs.msg import JointState

from so101_interfaces.srv import GoToTarget

STARTUP_TIMEOUT = 20.0  # s
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
STEP_BACK = 2.0  # s


@pytest.mark.launch_test
def generate_test_description():
    brain = launch_ros.actions.Node(
        package="so101_brain", executable="brain", parameters=[{"use_sim_time": True}], output="screen"
    )
    return launch.LaunchDescription([brain, launch_testing.actions.ReadyToTest()]), {"brain": brain}


class TestClockStep(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_clock_step")
        steady = Clock(clock_type=ClockType.STEADY_TIME)  # the test itself must not follow the system clock
        cls.offset = 0.0  # s subtracted from the published /clock
        cls.start = time.time() - time.monotonic()
        cls.q = dict.fromkeys(JOINTS, 0.0)  # fake arm: follows every command at once
        cls.commands, cls.poses = [], []  # commands as (arrival time, monotonic s; positions)
        cls.pub_clock = cls.node.create_publisher(ClockMsg, "/clock", 10)
        cls.pub_states = cls.node.create_publisher(JointState, "/joint_states", 10)
        cls.node.create_subscription(JointState, "/joint_command", cls.on_command, 100)
        cls.node.create_subscription(PoseStamped, "/end_effector_pose", cls.poses.append, 10)
        cls.node.create_timer(0.01, cls.publish_clock, clock=steady)
        cls.node.create_timer(1.0 / 30.0, cls.publish_states, clock=steady)
        cls.client = cls.node.create_client(GoToTarget, "/go_to_target")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def publish_clock(cls):
        t = cls.start + time.monotonic() - cls.offset
        cls.pub_clock.publish(ClockMsg(clock=Time(sec=int(t), nanosec=int(t % 1.0 * 1e9))))

    @classmethod
    def on_command(cls, msg):
        cls.commands.append((time.monotonic(), np.array(msg.position)))
        cls.q.update(zip(msg.name, msg.position, strict=True))

    @classmethod
    def publish_states(cls):
        cls.pub_states.publish(JointState(name=JOINTS, position=[cls.q[n] for n in JOINTS]))

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def test_a_clock_step_in_mid_motion_neither_rewinds_nor_pauses_it(self):
        end = time.monotonic() + STARTUP_TIMEOUT
        while not self.poses and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(self.poses, "no /end_effector_pose")
        self.assertTrue(self.client.wait_for_service(timeout_sec=5.0), "/go_to_target not served")
        request = GoToTarget.Request()
        request.target_pose.header.frame_id = "base_link"
        p = request.target_pose.pose.position
        p.x, p.y, p.z = 0.2, 0.0, 0.1  # 2.2 s motion (wrist_flex 0 -> 1.17 rad at 1 rad/s)
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.assertTrue(future.result().success, future.result().message)
        start = len(self.commands)
        self.spin_for(0.8)
        type(self).offset = STEP_BACK  # the ROS clock steps back in mid-motion
        self.spin_for(3.0)
        streamed = self.commands[start:]
        self.assertGreater(len(streamed), 50, "no trajectory streamed")
        goal = streamed[-1][1]
        distance = [np.linalg.norm(q - goal) for _, q in streamed]  # min-jerk: never increases
        back = max(b - a for a, b in pairwise(distance))
        gap = max(b[0] - a[0] for a, b in pairwise(streamed))
        print(f"{len(streamed)} commands; largest move back {back * 1000:.2f} mrad", end="; ")
        print(f"largest gap {gap * 1000:.0f} ms")
        self.assertLess(back, 1e-6, "the motion was rewound")
        self.assertLess(gap, 0.1, "the stream paused")
