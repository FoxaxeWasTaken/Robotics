"""pick_place_demo - the console of `tools/so101 demo` (#282): pick & place cycles in a terminal.

    ros2 run so101_bringup pick_place_demo

A first cycle by itself, then a key: n a new cycle, s stop, q quit. The screen, redrawn in place, shows the
cube, the box and the TCP as the nodes publish them, the cycle's steps with their durations, and the count of
cycles. Through a pipe: one line per step.

Simulation only: each cycle puts the cube at a new place (/sim/randomize), as `tools/so101 cycle`. No ground
truth: the cube and the box are perception's, and a success is the brain's DONE, which checks the cube in the
box through the camera (docs/project/06 §3.1). The bench measures with the truth: `tools/so101 bench`.
"""

import sys
import termios
import time
import tty
from functools import partial

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger

from so101_connect4.client import call
from so101_connect4.play import read_key, show

KEYS = "SO-101 pick & place, simulation        n: a new cycle   s: stop   q: quit"
NO_STATE = "no /brain_state yet: is the stack up, and built? (tools/so101 up, then tools/so101 wait)"
ENDS = ("DONE", "ERROR", "STOPPED")
POSES = {"cube": "/object_position", "box": "/drop_box_position", "hand": "/end_effector_pose"}
SEEN_BY = {"cube": "seen by the camera", "box": "seen by the camera", "hand": "computed from the joints"}
GREEN, YELLOW, RED = "\x1b[32m", "\x1b[33m", "\x1b[31m"


class Report:
    """The steps of the cycle shown and the count of cycles, from the brain's states."""

    def __init__(self):
        self.steps = []  # (state, when it began) of the cycle shown
        self.times = []  # s, each cycle that reached DONE
        self.cycle, self.failed, self.retries, self.running = 0, 0, 0, False
        self._began = 0.0

    @property
    def done(self) -> int:
        return len(self.times)

    def begin(self, now):
        """A cycle the brain accepted."""
        self.steps, self.running, self._began = [], True, now
        self.cycle += 1

    def on_state(self, state, now) -> bool:
        """The brain's state, said at each change and again each second; True when it is a new step."""
        head = state.split(":")[0]
        if not self.running or head == "IDLE" or (self.steps and self.steps[-1][0] == state):
            return False
        if not self.steps and head in ENDS:  # the end of the cycle before, said again as this one starts
            return False
        self.steps.append((state, now))
        self.retries += head == "RETRY"
        if head in ENDS:
            self.running = False
            if head == "DONE":
                self.times.append(now - self._began)
            self.failed += head == "ERROR"
        return True

    def timeline(self, now) -> list:
        """[(state, its duration in s, over)]: the running step's time so far; an end has none."""
        found = []
        for i, (state, start) in enumerate(self.steps):
            if i + 1 < len(self.steps):
                found.append((state, self.steps[i + 1][1] - start, True))
            elif self.running:
                found.append((state, now - start, False))
            else:
                found.append((state, 0.0, True))
        return found

    def counts(self) -> str:
        text = f"{self.done} done, {self.failed} failed, {self.retries} retries"
        if self.times:
            mean = sum(self.times) / len(self.times)
            text += f"     last {self.times[-1]:.1f} s   mean {mean:.1f} s   best {min(self.times):.1f} s"
        return text


def render(report, now, state, poses, message, color=False) -> str:
    """The whole screen."""

    def paint(code, text):
        return f"{code}{text}\x1b[0m" if color else text

    lines = [KEYS, ""]
    for name, xyz in poses.items():
        where = "not seen" if xyz is None else "x {:+.3f}  y {:+.3f}  z {:+.3f} m".format(*xyz)
        lines.append(f"  {name:<5} {where:<38} {SEEN_BY.get(name, '')}")
    lines += ["", f"  cycle {report.cycle}" + " " * 24 + f"brain: {state}"]
    for step, duration, over in report.timeline(now):
        head = step.split(":")[0]
        if head in ENDS:
            lines.append(
                paint(GREEN if head == "DONE" else RED, f"    {'ok' if head == 'DONE' else '!!'}  {step}")
            )
        elif over:
            lines.append(f"    {paint(GREEN, 'ok')}  {step:<12} {duration:5.1f} s")
        else:
            lines.append(paint(YELLOW, f"    >   {step:<12} {duration:5.1f} s"))
    lines += ["", "  " + report.counts()]
    if message:
        lines += ["", "  " + message]
    return "\n".join(lines)


class Console:
    """One step: a key, the brain's state, the screen. `arm` gives `state`, `poses`, randomize(), start() and
    stop(); the last two reply (success, message)."""

    def __init__(self, arm, write=show, clock=time.monotonic, live=False):
        self.arm, self.report, self.message = arm, Report(), ""
        self._write, self._clock, self._live = write, clock, live
        self._auto = True  # the first cycle starts by itself, once
        self._shown, self._drawn = None, float("-inf")
        self._born = clock()

    def step(self, key=None) -> bool:
        """False at q."""
        if key == "q":
            return False
        now, state = self._clock(), self.arm.state
        if state is None:
            if now - self._born > 2.0:  # the brain's latched state takes a moment to come (DDS discovery)
                self._say(NO_STATE)
        else:
            if self.message == NO_STATE:
                self._say("")
            if self.report.on_state(state, now) and not self._live:
                self._write(state)
                if not self.report.running:
                    self._write(self.report.counts())
            if key == "s":
                self._say("stop: " + self._reply(self.arm.stop))
            elif key == "n" and self.report.running:
                self._say("a cycle is running: s stops it")
            elif key == "n" or self._auto:
                self._start(state, now)
        if self._live:
            text = render(self.report, now, state, self.arm.poses, self.message, color=True)
            if text != self._shown and now - self._drawn >= 0.1:  # 10 Hz at most: the TCP moves at 30 Hz
                self._shown, self._drawn = text, now
                self._write("\x1b[H\x1b[J" + text)
        return True

    def _start(self, state, now):
        self._auto = False
        try:
            if state.startswith("ERROR"):  # a start in ERROR only re-arms the brain (06 §3.1)
                self.arm.start()
            self.arm.randomize()
            started, why = self.arm.start()
        except RuntimeError as error:
            self._say(str(error))
            return
        if started:
            self.report.begin(now)
        self._say("" if started else f"start refused: {why}")

    def _reply(self, service) -> str:
        try:
            return service()[1]
        except RuntimeError as error:
            return str(error)

    def _say(self, message):
        if message != self.message and message and not self._live:
            self._write(message)
        self.message = message


class Arm:
    """The brain and the sim over ROS, as the console wants them."""

    def __init__(self, node):
        self.state, self._poses = None, {}
        self._spin = lambda: rclpy.spin_once(node, timeout_sec=0.05)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String, "/brain_state", self._on_state, latched)
        for name, topic in POSES.items():
            node.create_subscription(PoseStamped, topic, partial(self._on_pose, name), 10)
        self._randomize = node.create_client(Trigger, "/sim/randomize")
        self._start = node.create_client(Trigger, "/brain/start")
        self._stop = node.create_client(Trigger, "/brain/stop")

    def _on_state(self, msg):
        self.state = msg.data

    def _on_pose(self, name, msg):
        position = msg.pose.position
        self._poses[name] = ((position.x, position.y, position.z), time.monotonic())

    @property
    def poses(self) -> dict:
        """{name: (x, y, z)}, None when not published for 1 s: perception says a cube only while it sees it"""
        now = time.monotonic()
        seen = {name: self._poses.get(name) for name in POSES}
        return {name: pose[0] if pose and now - pose[1] < 1.0 else None for name, pose in seen.items()}

    def randomize(self):
        call(self._spin, self._randomize, timeout_s=30.0)

    def start(self):
        return call(self._spin, self._start, timeout_s=30.0)

    def stop(self):
        return call(self._spin, self._stop, timeout_s=30.0)


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("pick_place_demo")
    live = sys.stdout.isatty()  # a terminal: the screen redrawn in place; a pipe: one line per step
    console = Console(Arm(node), live=live)
    if not live:
        show(KEYS)
    saved = termios.tcgetattr(sys.stdin) if sys.stdin.isatty() else None
    try:
        if saved:
            tty.setcbreak(sys.stdin)  # a key at a time, no Enter
        while console.step(read_key()):
            rclpy.spin_once(node, timeout_sec=0.05)
    except (KeyboardInterrupt, ExternalShutdownException):  # Ctrl-C: rclpy's own handler may shut down first
        pass
    finally:
        if saved:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, saved)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
