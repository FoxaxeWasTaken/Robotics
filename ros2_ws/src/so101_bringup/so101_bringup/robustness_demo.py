"""robustness_demo - one case of the robustness demo (step 5.4a, #103, docs/project/06 §10), run against the
demo's own stack, which robustness_demo.launch.py starts with it (the command, the everyday stack stopped:
results/robustness_demo.md):

    ros2 launch so101_bringup robustness_demo.launch.py case:=<1|2|3a|3b|4|5> [lighting:=dim] [episodes:=5]

It injects the case's fault (so101_bringup.robustness), prints each /brain_state with its time and the brain's
reason for each transition (read on /rosout), then the verdict. The faults: /sim/randomize (sim only), the
demo stack's perception node stopped (SIGINT, only the one on this ROS domain), or a fake target published
while perception is stopped (as test_9 of so101_brain). No ground truth (C-06). Case 5 runs `episodes` cycles
and counts, while the brain waits for its target, the images perception turned into a cube detection, and
into a box detection: the brain locks its target only with both.
"""

import argparse
import os
import signal
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import Log
from rclpy.clock import Clock, ClockType
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from so101_bringup.robustness import CASES, END_STATES, Injector, end_state, lighting_verdict, verdict

UNREACHABLE_CUBE = (0.5, 0.0, 0.025)  # m, base_link: out of reach, as test_9 of so101_brain
FAKE_BOX = (0.05, -0.25, 0.0)
FAKE_RATE_HZ = 10.0
CYCLE_TIMEOUT_S = 120.0
STARTUP_S = 60.0  # the stack is starting with the demo: the brain's first /end_effector_pose (#132)
SETTLE_S = 2.0  # after /sim/randomize: the cube settles before the brain looks
LOG_TAIL_S = 1.0  # after the end state: its transition's line on /rosout comes just after it
DISCOVERY_S = 3.0  # node names arrive late (DDS discovery, AGENTS.md): what the "one driver" check waits


class Demo:
    def __init__(self, node):
        self.node = node
        self.states = []  # distinct /brain_state values, in order, with their time
        self.reasons = []  # the brain's transition lines on /rosout
        self.pose = False
        self.images = self.cubes = self.boxes = 0  # counted while the brain is in WAIT_TARGET (case 5)
        self.t0 = time.monotonic()
        latched = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(String, "/brain_state", self._on_state, latched)
        node.create_subscription(Log, "/rosout", self._on_log, QoSProfile(depth=100))
        node.create_subscription(PoseStamped, "/end_effector_pose", self._on_pose, 10)
        node.create_subscription(Image, "/external_cam/image_raw", self._on_image, best_effort)
        node.create_subscription(PoseStamped, "/object_position", self._on_cube, 10)
        node.create_subscription(PoseStamped, "/drop_box_position", self._on_box, 10)
        self.randomize = node.create_client(Trigger, "/sim/randomize")
        self.start = node.create_client(Trigger, "/brain/start")
        self.fake = [
            (node.create_publisher(PoseStamped, "/object_position", 10), UNREACHABLE_CUBE),
            (node.create_publisher(PoseStamped, "/drop_box_position", 10), FAKE_BOX),
        ]
        self.faking = False
        steady = Clock(clock_type=ClockType.STEADY_TIME)  # WSL2 clock steps (AGENTS.md, MES-37)
        node.create_timer(1.0 / FAKE_RATE_HZ, self._publish_fake, clock=steady)

    def say(self, text):
        print(f"{time.monotonic() - self.t0:7.2f} s  {text}", flush=True)

    def _on_state(self, msg):
        if not self.states or self.states[-1] != msg.data:
            self.states.append(msg.data)
            self.say(msg.data)

    def _on_log(self, msg):
        if msg.name == "so101_brain" and "→" in msg.msg:
            self.reasons.append(msg.msg)
            print(f"           brain: {msg.msg}", flush=True)

    def _on_pose(self, msg):
        self.pose = True

    def _on_image(self, msg):
        self.images += self.state == "WAIT_TARGET"

    def _on_cube(self, msg):
        self.cubes += self.state == "WAIT_TARGET" and not self.faking

    def _on_box(self, msg):
        self.boxes += self.state == "WAIT_TARGET" and not self.faking

    @property
    def state(self):
        return self.states[-1] if self.states else ""

    def _publish_fake(self):
        if not self.faking:
            return
        for publisher, xyz in self.fake:
            msg = PoseStamped()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = xyz
            msg.pose.orientation.w = 1.0
            publisher.publish(msg)

    def spin_until(self, condition, timeout_s):
        deadline = time.monotonic() + timeout_s
        while not condition() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return condition()

    def call(self, client):
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f"{client.srv_name}: no service")
        future = client.call_async(Trigger.Request())
        self.spin_until(future.done, 10.0)
        if not future.done():
            raise RuntimeError(f"{client.srv_name}: no answer in 10 s")
        return future.result()

    def act(self, action):
        self.say(f">>> {action}")
        if action == "randomize":
            self.say(f"    {self.call(self.randomize).message}")
        elif action == "stop_perception":
            self.say(f"    perception stopped: pid {', '.join(map(str, stop_perception())) or 'none found'}")
        elif action == "unreachable_target":
            self.faking = True
            self.say(f"    fake cube at {UNREACHABLE_CUBE} and box at {FAKE_BOX}, {FAKE_RATE_HZ:g} Hz")

    def cycle(self, case, randomize=True):
        """One cycle: the case's faults, /brain/start, the states until an end state; the run's states."""
        injector = Injector(case)
        if self.state.startswith("ERROR"):  # after an ERROR, /brain/start only re-arms (06 §3.1)
            self.call(self.start)
            self.spin_until(lambda: self.state == "IDLE", 10.0)
        for action in injector.before_start():
            self.act(action)
        if randomize and "unreachable_target" not in case.before:
            self.act("randomize")
            time.sleep(SETTLE_S)
        first = len(self.states) - 1  # the run starts at the current state (IDLE)
        self.say(f"/brain/start: {self.call(self.start).message}")
        seen = len(self.states)
        deadline = time.monotonic() + CYCLE_TIMEOUT_S
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            while seen < len(self.states):  # one by one: an action spins too, states may arrive meanwhile
                seen += 1
                if action := injector.on_state(self.states[seen - 1]):
                    self.act(action)
            if len(self.states) > first + 1 and self.state.startswith(END_STATES):
                break
        self.spin_until(lambda: False, LOG_TAIL_S)  # the reason of the last transition, on /rosout
        return self.states[first:]


def stop_perception() -> list[int]:
    """SIGINT to the perception nodes started on this ROS domain (the demo's), never another stack's."""
    domain = f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')}".encode()
    stopped = []
    for proc in Path("/proc").iterdir():
        try:
            if not proc.name.isdigit() or b"lib/so101_perception/" not in (proc / "cmdline").read_bytes():
                continue
            if domain in (proc / "environ").read_bytes().split(b"\0"):
                os.kill(int(proc.name), signal.SIGINT)
                stopped.append(int(proc.name))
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
    return stopped


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--case", required=True, choices=sorted(CASES))
    parser.add_argument("--episodes", type=int, default=5, help="case 5: cycles under the changed light")
    argv = sys.argv if argv is None else ["robustness_demo", *argv]
    args = parser.parse_args(remove_ros_args(argv)[1:])  # a launch Node adds --ros-args -r __node:=...
    case = CASES[args.case]
    rclpy.init(args=argv)
    node = rclpy.create_node("robustness_demo")
    demo = Demo(node)
    try:
        print(f"case {args.case}: {case.question}\nexpected: {case.expected}", flush=True)
        if not demo.spin_until(lambda: demo.pose, STARTUP_S):
            print(f"no /end_effector_pose in {STARTUP_S:g} s: is the stack up?", file=sys.stderr)
            return 1

        def drivers():
            return [n for n, _ in node.get_node_names_and_namespaces() if n == "so101_driver"]

        demo.spin_until(lambda: drivers(), 10.0)  # node names come after the data (DDS discovery)
        demo.spin_until(lambda: False, DISCOVERY_S)  # a second driver, if any, shows up meanwhile
        if len(drivers()) != 1:
            print(f"{len(drivers())} drivers on this ROS domain: run the demo on its own", file=sys.stderr)
            return 1
        if args.case == "5":
            successes = 0
            for episode in range(1, args.episodes + 1):
                demo.images = demo.cubes = demo.boxes = 0
                states = demo.cycle(case)
                end = end_state(states)
                successes += end == "DONE"
                demo.say(
                    f"episode {episode}: {end}; in WAIT_TARGET, {demo.images} images, detections: "
                    f"cube {demo.cubes}, box {demo.boxes}"
                )
            result, why = lighting_verdict(successes, args.episodes)
        else:
            result, why = verdict(case, demo.cycle(case))
        print(f"VERDICT case {args.case}: {result}{f' ({why})' if why else ''}", flush=True)
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
