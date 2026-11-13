#!/usr/bin/env python3
"""game_node.py - a Connect Four game, from the board reader to the brain (docs/project/phase2/P2-02 §5).

The node around turns.py, as brain_node around fsm.py: callbacks only store messages, a 10 Hz timer hands them
to turns.step() and does what it returns. Nothing waits: the engine searches in a worker process, the brain's
services are called asynchronously and their reply is read by a later tick.

- /connect4/board, /connect4/grid_pose (board_reader), /end_effector_pose, /brain_state (brain): inputs.
- /connect4/new_game (data: the robot plays first), /connect4/human_done, /connect4/stop: for the player.
- /connect4/state: JSON, latched, at each change: turns.snapshot(), `ready` and the level, `max_depth`.
- /brain/handover at a new game, /brain/drop for a move, aimed at board.column_target on the grid's pose as
  the camera sees it; /brain/stop on a stop or a move's timeout. Every motion is the brain's (ADR-037).
"""

import json
import multiprocessing
import os
import signal
import threading
from concurrent.futures import ProcessPoolExecutor
from dataclasses import fields

import rclpy
import rclpy.node
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from so101_connect4 import board, game, layout, turns
from so101_interfaces.msg import Connect4Board
from so101_interfaces.srv import Drop

TICK_HZ = 10.0
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def ignore_sigint():
    signal.signal(signal.SIGINT, signal.SIG_IGN)  # Ctrl-C stops the node, and the node the worker


DELAYS = {  # turns.Params, each a read-only parameter in seconds
    "read_timeout_s": "READ_BOARD: no stable board with the move this long, then FIX_BOARD, s",
    "board_fresh_s": "A read this recent, unchanged at the read timeout: the robot's token is lost, s",
    "think_timeout_s": "THINK: no answer of the engine this long, then ERROR, s",
    "move_timeout_s": "MOVE: no reply of the brain this long, then /brain/stop and ERROR, s",
}


class GameNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("game_node")
        delays = {
            f.name: self._declare(f.name, f.default, DELAYS[f.name], 0.1, 600.0) for f in fields(turns.Params)
        }
        # The level: set while the node runs (ros2 param set, the console), the next THINK uses it.
        level = "The engine's search depth, game.LEVELS"
        self._declare("engine.max_depth", game.LEVELS["hard"], level, 1, game.CELLS, read_only=False)
        self._budget = self._declare("engine.budget_s", 1.0, "The engine's time for a move, s", 0.01, 60.0)
        self._seed = self._declare("engine.seed", 0, "Between two equal moves as central", 0, 2**31 - 1)
        if unknown := sorted(set(self._parameter_overrides) - set(self._parameters)):  # rclpy: no public API
            raise ValueError(f"Unknown parameters {unknown}: a typo in a config file?")
        self._turns = turns.Turns(turns.Params(**delays))
        self._layout = layout.load()
        self._board = self._grid = None  # the last /connect4/board; the grid's frame in base_link
        self._brain_seen = False  # a first /end_effector_pose: the brain has /joint_states
        self._brain_states = []
        self._new_game = self._robot_first = self._human_done = self._stop = False
        self._search = self._move = None  # the pending search, the pending call of the brain
        self._think_ticks, self._last_tick, self._published = [], None, None
        # A process, not a thread: the search holds the GIL. forkserver: the node is multithreaded (DDS).
        self._worker = ProcessPoolExecutor(1, multiprocessing.get_context("forkserver"), ignore_sigint)
        self._worker.submit(os.getpid).result()  # the first THINK does not wait for the start

        self.create_subscription(Connect4Board, "/connect4/board", self._cb_board, QoSProfile(depth=1))
        self.create_subscription(PoseStamped, "/connect4/grid_pose", self._cb_grid, LATCHED)
        self.create_subscription(PoseStamped, "/end_effector_pose", self._cb_pose, 1)
        self.create_subscription(String, "/brain_state", self._cb_brain_state, LATCHED)
        self._pub_state = self.create_publisher(String, "/connect4/state", LATCHED)
        self.create_service(SetBool, "/connect4/new_game", self._srv_new_game)
        self.create_service(Trigger, "/connect4/human_done", self._srv_human_done)
        self.create_service(Trigger, "/connect4/stop", self._srv_stop)
        self._drop = self.create_client(Drop, "/brain/drop")
        self._handover = self.create_client(Trigger, "/brain/handover")
        self._brain_stop = self.create_client(Trigger, "/brain/stop")
        # Steady clock: a step back of the system clock (WSL2, MES-37) would pause a ROS-clock timer.
        self._steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / TICK_HZ, self._tick, clock=self._steady)
        self._publish()
        self.get_logger().info("Game node ready.")

    def _now(self):
        return self._steady.now().nanoseconds * 1e-9

    def _cb_board(self, msg: Connect4Board):
        # int8[] arrives as a numpy array: plain ints, for the turns' comparisons and the state's JSON
        self._board = turns.Board(tuple(int(v) for v in msg.cells), msg.valid, msg.reason, self._now())

    def _cb_grid(self, msg: PoseStamped):
        p, o = msg.pose.position, msg.pose.orientation
        self._grid = board.transform((p.x, p.y, p.z), (o.x, o.y, o.z, o.w))

    def _cb_pose(self, msg: PoseStamped):
        self._brain_seen = True

    def _cb_brain_state(self, msg: String):
        self._brain_states.append(msg.data)

    def _missing(self) -> list:
        """What a game still waits for; empty: ready."""
        waited = {
            "/end_effector_pose": self._brain_seen,
            "/connect4/grid_pose": self._grid is not None,
            "/brain/drop": self._drop.service_is_ready(),
            "/brain/handover": self._handover.service_is_ready(),
        }
        return [name for name, there in waited.items() if not there]

    def _srv_new_game(self, request, response):
        missing = self._missing()
        response.success = not self._new_game and self._turns.accepts_new_game(not missing)
        if response.success:
            self._new_game, self._robot_first = True, request.data
        else:
            response.message = f"not ready: no {', '.join(missing)}" if missing else self._refused()
        return response

    def _srv_human_done(self, request, response):
        response.success = not self._human_done and self._turns.accepts_human_done
        self._human_done = response.success
        response.message = "" if response.success else self._refused()
        return response

    def _srv_stop(self, request, response):
        self._stop = response.success = True
        return response

    def _refused(self):
        return f"refused in {self._turns.label}"

    def _tick(self):
        now = self._now()
        if self._turns.state is turns.State.THINK and self._last_tick is not None:
            self._think_ticks.append(now - self._last_tick)
        self._last_tick = now
        inputs = turns.Inputs(
            ready=not self._missing(),
            board=self._board,
            new_game=self._new_game,
            robot_first=self._robot_first,
            human_done=self._human_done,
            stop=self._stop,
            engine=self._engine(),
            move_reply=self._move_reply(),
            brain_states=tuple(self._brain_states),
        )
        self._new_game = self._human_done = self._stop = False
        self._brain_states = []
        out = self._turns.step(now, inputs)
        if out.transition:
            self.get_logger().info(out.transition)
        # A new search or a new call replaces the pending one: the late answer of a cut one is never read.
        if out.think:
            self._think_ticks = []
            self._search = self._worker.submit(
                game.best_move, *out.think, self._budget, self._max_depth(), self._seed
            )
        if out.stop_brain:
            self._brain_stop.call_async(Trigger.Request())
        if out.handover:
            self._move = self._handover.call_async(Trigger.Request())
        if out.drop is not None:
            request = Drop.Request()
            target = board.column_target(self._layout, self._grid, out.drop)
            request.target.x, request.target.y, request.target.z = (float(v) for v in target)
            self._move = self._drop.call_async(request)
        self._publish()

    def _declare(self, name, default, description, low, high, read_only=True):
        """A parameter with its range, as the phase-1 nodes': an int range for an int default, else a float
        one. A value out of it stops the node at startup."""
        bounds = (
            {"integer_range": [IntegerRange(from_value=low, to_value=high, step=1)]}
            if isinstance(default, int)
            else {"floating_point_range": [FloatingPointRange(from_value=low, to_value=high)]}
        )
        descriptor = ParameterDescriptor(description=description, read_only=read_only, **bounds)
        return self.declare_parameter(name, default, descriptor).value

    def _max_depth(self):
        return self.get_parameter("engine.max_depth").value

    def _engine(self):
        """The engine's answer once its search is done, else None."""
        if self._search is None or not self._search.done():
            return None
        choice, self._search = self._search.result(), None
        ticks = self._think_ticks or [0.0]
        self.get_logger().info(
            f"engine: column {choice.column}, score {choice.score}, depth {choice.depth}, "
            f"{choice.seconds:.2f} s; tick period max {max(ticks) * 1000:.0f} ms over {len(ticks)} ticks"
        )
        return choice

    def _move_reply(self):
        """(success, message) of the brain once the move's call is done, else None."""
        if self._move is None or not self._move.done():
            return None
        result, self._move = self._move.result(), None
        return result.success, result.message

    def _publish(self):
        """At each change, and once at startup: the topic is latched."""
        state = {"ready": not self._missing(), "max_depth": self._max_depth(), **self._turns.snapshot()}
        text = json.dumps(state)
        if text != self._published:
            self._published = text
            self._pub_state.publish(String(data=text))

    def destroy_node(self):
        self._worker.shutdown(wait=False, cancel_futures=True)  # a running search ends, then the process
        super().destroy_node()


def main(args=None):
    # SIGINT and SIGTERM only set an event, as the driver and perception (#158, #177): rclpy's handlers raise
    # at any bytecode.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())  # before the node: its "ready" log is the start
    node = GameNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while not stop.is_set():
            executor.spin_once(timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
