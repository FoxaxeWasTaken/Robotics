"""The robustness demo's runner (#103) against a fake stack in the same process: a node named like the driver
serves /sim/randomize and /brain/start, publishes /end_effector_pose and, on each start, the /brain_state
sequence of one run, as the brain would."""

import os
import signal
import subprocess
import sys
import threading
import time

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from so101_bringup import robustness_demo

RUN_1 = [
    "WAIT_TARGET",
    "PLAN",
    "APPROACH",
    "DESCEND",
    "GRASP",
    "RETRY",
    "HOME_RETRY",
    "WAIT_TARGET",
    "PLAN",
    "APPROACH",
    "DESCEND",
    "GRASP",
    "LIFT",
    "TRANSPORT",
    "RELEASE",
    "HOME",
    "VERIFY",
    "DONE",
]
RUN_4 = ["WAIT_TARGET", "PLAN", "ERROR:unreachable"]
RUN_5 = ["WAIT_TARGET", "PLAN", "APPROACH", "DESCEND", "GRASP", "LIFT", "TRANSPORT", "VERIFY", "DONE"]


class FakeStack:
    def __init__(self, run):
        self.run, self.randomized = run, 0
        self.context = rclpy.Context()
        rclpy.init(context=self.context)
        node = self.node = rclpy.create_node("so101_driver", context=self.context)
        latched = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.state = node.create_publisher(String, "/brain_state", latched)
        self.pose = node.create_publisher(PoseStamped, "/end_effector_pose", 10)
        node.create_service(Trigger, "/sim/randomize", self._randomize)
        node.create_service(Trigger, "/brain/start", self._start)
        self.queue = ["IDLE"]
        node.create_timer(0.05, self._tick)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def _tick(self):
        self.pose.publish(PoseStamped())
        if self.queue:
            self.state.publish(String(data=self.queue.pop(0)))

    def _randomize(self, request, response):
        self.randomized += 1
        response.success, response.message = True, "cube and box moved"
        return response

    def _start(self, request, response):
        self.queue += self.run
        response.success, response.message = True, "started"
        return response

    def close(self):
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)
        self.thread.join(timeout=5.0)


@pytest.fixture(autouse=True)
def fast(monkeypatch):
    for name, value in (
        ("SETTLE_S", 0.0),
        ("LOG_TAIL_S", 0.2),
        ("DISCOVERY_S", 0.5),
        ("CYCLE_TIMEOUT_S", 30.0),
    ):
        monkeypatch.setattr(robustness_demo, name, value)


@pytest.fixture
def stack(request):
    fake = FakeStack(request.param)
    yield fake
    fake.close()


@pytest.mark.parametrize("stack", [RUN_1], indirect=True)
def test_case_1_moves_the_cube_on_approach_and_is_handled(stack, capsys):
    assert robustness_demo.main(["--case", "1"]) == 0
    out = capsys.readouterr().out
    assert "VERDICT case 1: handled" in out
    assert stack.randomized == 2  # before /brain/start, then on entering APPROACH


@pytest.mark.parametrize("stack", [RUN_4], indirect=True)
def test_case_4_fakes_an_unreachable_target_and_is_handled(stack, capsys):
    assert robustness_demo.main(["--case", "4"]) == 0
    out = capsys.readouterr().out
    assert "fake cube at (0.5, 0.0, 0.025)" in out
    assert "VERDICT case 4: handled" in out


@pytest.mark.parametrize("stack", [RUN_5], indirect=True)
def test_case_5_counts_the_cycles_that_end_in_done(stack, capsys):
    assert robustness_demo.main(["--case", "5", "--episodes", "1"]) == 0
    out = capsys.readouterr().out
    assert "episode 1: DONE" in out
    assert "VERDICT case 5: handled (1/1 cycles ended in DONE)" in out


def test_no_stack_exits_1(monkeypatch, capsys):
    monkeypatch.setattr(robustness_demo, "STARTUP_S", 0.5)
    assert robustness_demo.main(["--case", "1"]) == 1
    assert "is the stack up?" in capsys.readouterr().err


def test_stop_perception_stops_only_the_perception_of_its_domain(monkeypatch):
    """A process with perception's command line on this ROS domain gets SIGINT; one on another does not."""
    domain = os.environ.get("ROS_DOMAIN_ID", "0")
    other = str((int(domain) + 1) % 100)
    sleeper = [
        sys.executable,
        "-c",
        "import time; time.sleep(30)",
        "/install/lib/so101_perception/perception",
    ]
    mine = subprocess.Popen(sleeper, env={**os.environ, "ROS_DOMAIN_ID": domain})
    theirs = subprocess.Popen(sleeper, env={**os.environ, "ROS_DOMAIN_ID": other})
    monkeypatch.setenv("ROS_DOMAIN_ID", domain)
    try:
        time.sleep(0.3)
        stopped = robustness_demo.stop_perception()
        assert mine.pid in stopped
        assert theirs.pid not in stopped
        assert mine.wait(timeout=5.0) != 0  # KeyboardInterrupt
        assert theirs.poll() is None
    finally:
        for proc in (mine, theirs):
            if proc.poll() is None:
                proc.send_signal(signal.SIGKILL)
                proc.wait()
