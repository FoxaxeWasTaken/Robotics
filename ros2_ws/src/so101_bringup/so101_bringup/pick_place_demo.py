"""pick_place_demo - the console of `tools/so101 demo` (#282): pick & place cycles in a terminal.

    ros2 run so101_bringup pick_place_demo

A first cycle by itself, then a key: n a new cycle, s stop, q quit. The screen, redrawn in place with the look
of the game's console (so101_common.terminal), shows where the camera sees the cube and the box, a progress
bar over the cycle's ten steps, each step in plain words with its duration, and how many cubes reached the
box and how fast. Through a pipe: one line per step.

Simulation only: each cycle puts the cube at a new place (/sim/randomize), as `tools/so101 cycle`. No ground
truth: the cube and the box are perception's, and a success is the brain's DONE, which checks the cube in the
box through the camera (docs/project/06 §3.1). The bench measures with the truth: `tools/so101 bench`.
"""

import sys
import time
from functools import partial

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger

from so101_common import terminal
from so101_common.terminal import Pen, read_key, show
from so101_connect4.client import call

TITLE = "Pick & place, simulation"
KEYS = [("n", "new cycle"), ("s", "stop"), ("q", "quit")]
NO_STATE = "no /brain_state yet: tools/so101 up, then tools/so101 wait"
ENDS = ("DONE", "ERROR", "STOPPED")
POSES = {"cube": "/object_position", "box": "/drop_box_position"}
STEPS = {  # the brain's states of a cycle, in order (06 §3.1), as a jury reads them
    "WAIT_TARGET": "Looking for the cube",
    "PLAN": "Planning",
    "APPROACH": "Moving above the cube",
    "DESCEND": "Going down",
    "GRASP": "Closing the gripper",
    "LIFT": "Lifting",
    "TRANSPORT": "Carrying to the box",
    "RELEASE": "Dropping",
    "HOME": "Returning",
    "VERIFY": "Checking with the camera",
}
CAUSES = {  # the brain's ERROR:<cause>, by its first word
    "grasp_failed": "the cube was not grasped",
    "object_lost": "the camera did not find the cube",
    "no_joint_states": "the arm's joint positions stopped coming",
    "timeout": "a move took too long",
}


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

    @property
    def attempt(self) -> int:
        """Which try of the cycle is shown: a missed grasp starts again at WAIT_TARGET."""
        return max(1, sum(state == "WAIT_TARGET" for state, _ in self.steps))

    def progress(self, now) -> list:
        """[(state, its duration in s or None, mark)] for the ten steps of the attempt in hand; mark: done,
        run, fail (a retry, an error or a stop came next) or todo."""
        timeline, found = self.timeline(now), {}
        starts = [i for i, (state, _, _) in enumerate(timeline) if state == "WAIT_TARGET"]
        attempt = timeline[starts[-1] :] if starts else timeline
        for i, (state, duration, over) in enumerate(attempt):
            after = attempt[i + 1] if i + 1 < len(attempt) else None
            failed = after is not None and after[0].split(":")[0] in ("RETRY", *ENDS[1:])
            mark = "done" if over else "run"
            found[state] = (duration, "fail" if failed else mark)
        return [(state, *found.get(state, (None, "todo"))) for state in STEPS]

    def elapsed(self, now) -> float:
        """The time of the cycle shown, its retries included; it stops at the cycle's end."""
        if not self.steps:
            return 0.0
        return (now if self.running else self.steps[-1][1]) - self._began

    def counts(self) -> str:
        ended = self.done + self.failed
        if not ended:
            return "no cycle finished yet"
        text = f"{self.done} of {ended} cube{'s' * (ended > 1)} in the box"
        if self.times:
            mean = sum(self.times) / len(self.times)
            text += f"   last {self.times[-1]:.1f} s   average {mean:.1f} s   best {min(self.times):.1f} s"
        return text + (f"   retries {self.retries}" if self.retries else "")


def status(report, state, refused=False) -> tuple:
    """(done, go, run or fail; one sentence): what happens, in plain words. `refused`: the first cycle was."""
    if state is None:
        return "run", "waiting for the brain's state"
    if not report.cycle:
        return ("go", "n: a new cycle") if refused else ("run", "starting the first cycle")
    last = report.steps[-1][0] if report.steps else ""
    head, _, cause = last.partition(":")
    if report.running:
        doing = "missed the cube, trying again" if "RETRY" in head else STEPS.get(head, "starting").lower()
        return "run", f"cycle {report.cycle}: {doing}"
    if head == "DONE":
        return "done", f"cube in the box, checked by the camera ({report.times[-1]:.1f} s); n: a new cycle"
    if head == "ERROR":
        return "fail", f"{CAUSES.get(cause.split(':')[0], last)}; n: a new cycle"
    return "run", "stopped; n: a new cycle"


def render(report, now, state, poses, message, pen) -> str:
    """The whole screen, 17 lines whatever happens: nothing jumps between two frames."""
    lines = [pen.header(TITLE, state or "no state"), pen.rule("scene")]
    for name, xyz in poses.items():
        seen = "not seen by the camera"
        if xyz:
            seen = f"x {xyz[0] * 100:5.1f} cm   y {xyz[1] * 100:5.1f} cm         seen by the camera"
        lines.append(f"  {name:<6} {seen}")
    progress = report.progress(now)
    done = sum(mark == "done" for _, _, mark in progress)
    current = next((mark for _, _, mark in progress if mark in ("run", "fail")), None)
    title = f"cycle {report.cycle}" + (f", attempt {report.attempt}" if report.attempt > 1 else "")
    lines.append(pen.rule(title if report.cycle else "no cycle yet"))
    bar = pen.bar(done, len(STEPS), 5, current)
    lines.append(f"  {bar}  {done:>2}/{len(STEPS)}  {report.elapsed(now):6.1f} s")

    def cell(step):
        state, duration, mark = step
        took = "" if duration is None else f"{duration:.1f} s"
        text = f"{STEPS[state]:<25}{took:>7}"
        if mark in ("run", "fail"):  # the step in hand, or the one that failed: the whole cell in its color
            return pen.paint(mark, f"{pen.glyph(mark)} {text}")
        return f"{pen.paint(mark, pen.glyph(mark))} {text}"

    half = len(STEPS) // 2  # the pick on the left, the place on the right
    lines += [
        f"  {cell(pick)}   {cell(place)}"
        for pick, place in zip(progress[:half], progress[half:], strict=True)
    ]
    lines += [pen.rule("total"), "  " + report.counts(), pen.rule()]
    lines += [
        pen.status(*status(report, state, bool(message))),
        "   " + message if message else "",
        pen.keys(KEYS),
    ]
    return "\n".join(line if "\x1b" in line else line.rstrip() for line in lines)


class Console:
    """One step: a key, the brain's state, the screen. `arm` gives `state`, `poses`, randomize(), start() and
    stop(); the last two reply (success, message)."""

    def __init__(self, arm, write=show, clock=time.monotonic, live=False, pen=None):
        """`live`: a terminal, the screen redrawn in place with `pen`; else one line per step."""
        self.arm, self.report, self.message = arm, Report(), ""
        self._write, self._clock, self._live, self._pen = write, clock, live, pen or Pen()
        self._auto = True  # the first cycle starts by itself, once
        self._shown, self._drawn = None, float("-inf")
        self._born = clock()

    def step(self, key=None) -> bool:
        """False at q."""
        if key == "q":
            return False
        now, state = self._clock(), self.arm.state
        if state is not None:
            self._act(state, now, key)
        elif now - self._born > 2.0:  # the brain's latched state takes a moment to come (DDS discovery)
            self._say(NO_STATE)
        if self._live:
            self._draw(state, now)
        return True

    def _act(self, state, now, key):
        """The brain's state into the report, then the key."""
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

    def _draw(self, state, now):
        text = render(self.report, now, state, self.arm.poses, self.message, self._pen)
        if text != self._shown and now - self._drawn >= 0.1:  # 10 frames a second at most
            self._shown, self._drawn = text, now
            self._write(terminal.frame(text))

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
    console = Console(Arm(node), live=live, pen=Pen(terminal.pretty()))
    if not live:
        show(f"SO-101 {TITLE}: " + ", ".join(f"{key} {label}" for key, label in KEYS))
    try:
        with terminal.session():
            while console.step(read_key()):
                rclpy.spin_once(node, timeout_sec=0.05)
    except (KeyboardInterrupt, ExternalShutdownException):  # Ctrl-C: rclpy's own handler may shut down first
        pass
    finally:
        if live:
            show("")  # the shell's prompt under the last frame, not over it
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
