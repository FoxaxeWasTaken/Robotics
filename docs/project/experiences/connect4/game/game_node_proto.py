"""Shape of game_node for ticket #86: stored messages, a 10 Hz tick, async clients polled, no wait anywhere.

Stand-ins, the real interfaces do not exist yet: /connect4/board is JSON in a String, /brain/drop is ChangeState
(transition.id = column, 255 home). Run by run_proto.sh. Usage: game_node_proto.py process|thread
"""

import json
import multiprocessing
import signal
import statistics
import sys
import threading
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor

import rclpy
import rclpy.node
from lifecycle_msgs.srv import ChangeState  # any srv with an int: stands in for /brain/drop
from geometry_msgs.msg import PoseStamped
from rclpy.clock import Clock, ClockType
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

sys.path.insert(0, ".")
import game
from turns import Board, Inputs, State, Turns

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
TICK_HZ = 10.0


def ignore_sigint():
    signal.signal(signal.SIGINT, signal.SIG_IGN)  # Ctrl-C stops the node, and the node the worker


class GameNode(rclpy.node.Node):
    def __init__(self, worker):
        super().__init__("game_node")
        self._turns = Turns()
        self._board, self._pose_seen, self._brain_states = None, False, []
        self._new_game = self._robot_first = self._human_done = self._stop = False
        self._search = self._drop = None
        self._refused = None
        if worker == "process":
            context = multiprocessing.get_context("forkserver")
            self._pool = ProcessPoolExecutor(1, mp_context=context, initializer=ignore_sigint)
        else:
            self._pool = ThreadPoolExecutor(1)
        self.ticks, self._last_tick, self._state_label = [], None, None
        self.create_subscription(String, "/connect4/board", self._cb_board, 1)
        self.create_subscription(PoseStamped, "/end_effector_pose", lambda _: setattr(self, "_pose_seen", True), 1)
        self.create_subscription(String, "/brain_state", lambda m: self._brain_states.append(m.data), LATCHED)
        self._pub = self.create_publisher(String, "/connect4/state", LATCHED)
        self.create_service(SetBool, "/connect4/new_game", self._srv_new_game)
        self.create_service(Trigger, "/connect4/human_done", self._srv_human_done)
        self.create_service(Trigger, "/connect4/stop", self._srv_stop)
        self._client_drop = self.create_client(ChangeState, "/brain/drop")
        self._client_stop = self.create_client(Trigger, "/brain/stop")
        self._steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / TICK_HZ, self._tick, clock=self._steady)
        self.create_timer(1.0, self._publish, clock=self._steady)
        self._publish()

    def _now(self):
        return self._steady.now().nanoseconds * 1e-9

    def _cb_board(self, msg):
        d = json.loads(msg.data)
        self._board = Board(tuple(d["cells"]), d["valid"], d["reason"], self._now())

    @property
    def _ready(self):
        return self._pose_seen and self._client_drop.service_is_ready()

    def _srv_new_game(self, request, response):
        response.success = not self._new_game and self._turns.accepts_new_game(self._ready)
        response.message = "" if response.success else f"refused in {self._turns.label}, ready={self._ready}"
        if response.success:
            self._new_game, self._robot_first = True, request.data
        return response

    def _srv_human_done(self, request, response):
        response.success = not self._human_done and self._turns.accepts_human_done
        response.message = "" if response.success else f"refused in {self._turns.label}"
        self._human_done = response.success
        return response

    def _srv_stop(self, request, response):
        self._stop = response.success = True
        return response

    def _tick(self):
        now = self._now()
        if self._turns.state is State.THINK and self._last_tick is not None:
            self.ticks.append(now - self._last_tick)
        self._last_tick = now
        engine = None
        if self._search is not None and self._search.done():
            engine, self._search = self._search.result().column, None
        reply = self._refused
        self._refused = None
        if self._drop is not None and self._drop.done():
            result, self._drop = self._drop.result(), None
            reply = (result.success, "refused") if result is not None else (False, "no answer")
        states, self._brain_states = tuple(self._brain_states), []
        inputs = Inputs(
            ready=self._ready, board=self._board, new_game=self._new_game, robot_first=self._robot_first,
            human_done=self._human_done, stop=self._stop, engine=engine, drop_reply=reply, brain_states=states,
        )  # fmt: skip
        self._new_game = self._human_done = self._stop = False
        out = self._turns.step(now, inputs)
        if out.transition:
            self.get_logger().info(out.transition)
        if self._turns.state is not State.THINK:
            self._search = None  # a late answer is never read: THINK only starts with a new request
        if out.think:
            self._search = self._pool.submit(game.best_move, *out.think, 1.0)
        if out.stop_brain and self._client_stop.service_is_ready():
            self._client_stop.call_async(Trigger.Request())
        if out.drop is not None or out.home:
            if self._client_drop.service_is_ready():
                request = ChangeState.Request()
                request.transition.id = 255 if out.home else out.drop
                self._drop = self._client_drop.call_async(request)
            else:
                self._refused = (False, "no /brain/drop")
        if self._turns.label != self._state_label:
            self._publish()

    def _publish(self):
        self._state_label = self._turns.label
        self._pub.publish(String(data=json.dumps({"ready": self._ready, **self._turns.snapshot()})))


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    node = GameNode(sys.argv[1])
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    while not stop.is_set():
        executor.spin_once(timeout_sec=0.1)
    p = sorted(node.ticks)
    if p:
        print(
            f"{sys.argv[1]}: tick period in THINK, ms: n {len(p)}, median {statistics.median(p) * 1000:.1f}, "
            f"p99 {p[int(0.99 * len(p))] * 1000:.1f}, max {p[-1] * 1000:.1f}",
            flush=True,
        )
    node._pool.shutdown(wait=False, cancel_futures=True)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
