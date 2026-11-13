"""game_node (#86, phase2/P2-02 §5): the real node in this process, so that its coverage counts, next to a
fake stack on the same executor: a brain serving /brain/drop, /brain/handover and /brain/stop, a board reader
publishing the real Connect4Board, and the console's clients. The engine is the real one, in its worker.
The rules of the turns are test_turns.py's; here, what the node adds: the messages in and out, the calls to
the brain, and that nothing blocks the tick."""

import json
import re
import time

import numpy as np
import pytest
import rclpy
import rclpy.node
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import Log
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future
from std_msgs.msg import String, UInt8MultiArray
from std_srvs.srv import SetBool, Trigger
from test_turns import EMPTY_BOARD, with_tokens

from so101_connect4 import board, game, layout
from so101_connect4.game import COLUMNS, HUMAN, ROBOT
from so101_connect4.game_node import GameNode
from so101_interfaces.msg import Connect4Board
from so101_interfaces.srv import Drop
from so101_perception.geometry import quaternion

LAYOUT = layout.load()
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
TURN, AWAY = -0.7, (0.15, -0.12, 0.05)  # a grid turned about z and away from the base, rad and m
T_BASE_GRID = np.eye(4)
T_BASE_GRID[:2, :2] = [[np.cos(TURN), -np.sin(TURN)], [np.sin(TURN), np.cos(TURN)]]
T_BASE_GRID[:3, 3] = AWAY
TARGETS = [board.column_target(LAYOUT, T_BASE_GRID, column) for column in range(COLUMNS)]


T_BASE_DISPENSER = np.eye(4)  # a dispenser turned the other way, somewhere else
T_BASE_DISPENSER[:2, :2] = [[np.cos(0.9), -np.sin(0.9)], [np.sin(0.9), np.cos(0.9)]]
T_BASE_DISPENSER[:3, 3] = (0.12, 0.2, 0.03)
STATIONS = [
    board.station_target(LAYOUT, T_BASE_DISPENSER, rail) for rail in range(LAYOUT["dispenser"]["rails"])
]


def pose(T):
    msg = PoseStamped()
    msg.header.frame_id = "base_link"
    p, o = msg.pose.position, msg.pose.orientation
    p.x, p.y, p.z = (float(v) for v in T[:3, 3])
    o.x, o.y, o.z, o.w = (float(v) for v in quaternion(T[:3, :3]))
    return msg


class FakeStack(rclpy.node.Node):
    """The brain and the board reader as the game node sees them, and the clients of its services."""

    def __init__(self):
        super().__init__("fake_stack")
        self.cells, self.valid, self.reason = EMPTY_BOARD, True, ""  # what the reader publishes
        self.calls = []  # the brain's services called, in order: ("handover",), ("drop", column), ("stop",)
        self.replies = {
            "drop": (True, "done in 14.0 s"),
            "handover": (True, "done in 0.1 s"),
            "pick": (True, "done in 9.0 s"),
        }
        self.waiting = (1, 1, 1)  # what the reader publishes on /connect4/stations; None: nothing
        self.grid = T_BASE_GRID  # where the reader sees the grid: the drops' targets are on it
        self.checked = []  # the check_only requests, in order: ("drop", column), ("pick", rail)
        self.checks = {}  # their reply when not (True, ""), by request
        self.hold = False  # a move replies only once released: release(), or /brain/stop
        self.stop_cuts = True  # /brain/stop makes the held moves reply `stopped`
        self.held = []  # the futures of the held moves, in order
        self.lands = None  # the column the robot's token falls in, when not the one asked
        self.states, self.logs = [], []
        self.executor_ = None
        moves = ReentrantCallbackGroup()  # as the brain: a held move does not block the next request
        self.srv_drop = self.create_service(Drop, "/brain/drop", self.on_drop, callback_group=moves)
        self.create_service(Trigger, "/brain/handover", self.on_handover, callback_group=moves)
        self.create_service(Drop, "/brain/pick", self.on_pick, callback_group=moves)
        self.pub_dispenser = self.create_publisher(PoseStamped, "/connect4/dispenser_pose", LATCHED)
        self.pub_stations = self.create_publisher(UInt8MultiArray, "/connect4/stations", QoSProfile(depth=1))
        self.create_service(Trigger, "/brain/stop", self.on_stop)
        self.pub_board = self.create_publisher(Connect4Board, "/connect4/board", QoSProfile(depth=1))
        self.pub_grid = self.create_publisher(PoseStamped, "/connect4/grid_pose", LATCHED)
        self.pub_pose = self.create_publisher(PoseStamped, "/end_effector_pose", 10)
        self.pub_brain_state = self.create_publisher(String, "/brain_state", LATCHED)
        self.create_subscription(String, "/connect4/state", self.on_state, LATCHED)
        self.create_subscription(Log, "/rosout", lambda m: self.logs.append(m.msg), 100)
        # The reader publishes at each image once its read is stable: here at 20 Hz, on the steady clock.
        self.create_timer(0.05, self.publish_board, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.new_game = self.create_client(SetBool, "/connect4/new_game")
        self.human_done = self.create_client(Trigger, "/connect4/human_done")
        self.stop = self.create_client(Trigger, "/connect4/stop")

    def on_state(self, msg):
        self.states.append(json.loads(msg.data))

    def publish_board(self):
        self.pub_board.publish(Connect4Board(cells=self.cells, valid=self.valid, reason=self.reason))
        if self.waiting is not None:
            self.pub_stations.publish(UInt8MultiArray(data=list(self.waiting)))

    async def on_pick(self, request, response):
        target = np.array([request.target.x, request.target.y, request.target.z])
        rail = int(np.argmin([np.linalg.norm(target - t) for t in STATIONS]))
        np.testing.assert_allclose(target, STATIONS[rail], atol=1e-9)  # board.station_target, on its pose
        if request.check_only:
            return self.check(("pick", rail), response)
        return await self.move(("pick", rail), response)

    async def on_drop(self, request, response):
        target = np.array([request.target.x, request.target.y, request.target.z])
        targets = [board.column_target(LAYOUT, self.grid, column) for column in range(COLUMNS)]
        column = int(np.argmin([np.linalg.norm(target - t) for t in targets]))
        np.testing.assert_allclose(target, targets[column], atol=1e-9)  # board.column_target, on the pose
        if request.check_only:
            return self.check(("drop", column), response)
        return await self.move(("drop", column), response)

    def check(self, call, response):
        """As the brain's check_only: the reply at once, nothing moves."""
        self.checked.append(call)
        response.success, response.message = self.checks.get(call, (True, ""))
        return response

    async def on_handover(self, request, response):
        return await self.move(("handover",), response)

    async def move(self, call, response):
        self.calls.append(call)
        reply = self.replies[call[0]]
        if self.hold:
            self.held.append(Future())
            reply = await self.held[-1]
        response.success, response.message = reply
        if response.success and call[0] == "drop":
            self.cells = with_tokens(self.cells, (call[1] if self.lands is None else self.lands, ROBOT))
        return response

    def on_stop(self, request, response):
        self.calls.append(("stop",))
        if self.stop_cuts:
            for held in self.held:
                if not held.done():
                    held.set_result((False, "stopped"))
        response.success = True
        return response

    @property
    def label(self):
        state = self.states[-1]
        return f"{state['state']}:{state['reason']}" if state["reason"] else state["state"]

    def spin_until(self, condition, timeout=10.0):
        end = time.monotonic() + timeout
        while not condition():
            assert time.monotonic() < end, f"not in {timeout} s: state {self.states[-1:]}, calls {self.calls}"
            self.executor_.spin_once(timeout_sec=0.01)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.executor_.spin_once(timeout_sec=0.01)

    def wait_for(self, label, timeout=10.0, **fields):
        """Until /connect4/state shows `STATE`, or `STATE:reason`, with those fields; that state."""

        def shown():
            state = self.states[-1] if self.states else {}
            return state and self.label == label and all(state[k] == v for k, v in fields.items())

        self.spin_until(shown, timeout)
        return self.states[-1]

    def call(self, client, request=None):
        assert client.wait_for_service(timeout_sec=10.0), f"{client.srv_name} not served"
        future = client.call_async(request or client.srv_type.Request())
        self.executor_.spin_until_future_complete(future, timeout_sec=5.0)
        assert future.done(), f"{client.srv_name}: no reply"
        return future.result()

    def up(self):
        """The brain's first pose and the grid's: the game node is ready."""
        self.pub_pose.publish(PoseStamped())
        self.pub_grid.publish(pose(self.grid))
        self.pub_dispenser.publish(pose(T_BASE_DISPENSER))
        self.spin_until(lambda: self.states and self.states[-1]["ready"])

    def start(self, robot_first=False):
        """A new game, up to WAIT_HUMAN."""
        self.up()
        reply = self.call(self.new_game, SetBool.Request(data=robot_first))
        assert reply.success, reply.message
        return self.wait_for("WAIT_HUMAN")

    def human_plays(self, column):
        self.cells = with_tokens(self.cells, (column, HUMAN))
        reply = self.call(self.human_done)
        assert reply.success, reply.message


@pytest.fixture
def stack(request):
    """The game node and its fake stack; an indirect parameter sets the node's ROS parameters."""
    parameters = getattr(request, "param", {})
    args = [arg for name, value in parameters.items() for arg in ("-p", f"{name}:={value}")]
    rclpy.init(args=["--ros-args", *args])
    game, fake = GameNode(), FakeStack()
    fake.game = game
    fake.executor_ = SingleThreadedExecutor()
    fake.executor_.add_node(game)
    fake.executor_.add_node(fake)
    yield fake
    fake.executor_.shutdown()
    fake.destroy_node()
    game.destroy_node()
    rclpy.shutdown()


def test_the_state_is_published_at_once_not_ready_and_a_new_game_is_refused_until_the_stack_is_seen(stack):
    """ "Brain node ready" does not mean the brain has /joint_states (AGENTS.md): the first pose does."""
    stack.spin_until(lambda: stack.states)
    assert stack.states[-1]["state"] == "IDLE"
    assert stack.states[-1]["ready"] is False
    reply = stack.call(stack.new_game, SetBool.Request())
    assert not reply.success
    assert "/end_effector_pose" in reply.message
    assert "/connect4/grid_pose" in reply.message
    stack.pub_pose.publish(PoseStamped())
    stack.spin_for(0.3)
    reply = stack.call(stack.new_game, SetBool.Request())
    assert not reply.success
    assert "/end_effector_pose" not in reply.message
    assert "/connect4/grid_pose" in reply.message
    assert stack.calls == []


def test_a_new_game_calls_handover_then_reads_the_empty_board(stack):
    state = stack.start()
    assert stack.calls == [("handover",)]
    assert state["board"] == [0] * 42  # the message's int8 array, as JSON
    assert (state["turn"], state["legal"]) == ("human", list(range(COLUMNS)))


def test_a_turn_the_humans_token_then_the_engine_then_a_drop_on_the_grid_as_it_is_seen(stack):
    stack.start()
    stack.human_plays(3)
    state = stack.wait_for("WAIT_HUMAN", moves=2)  # READ_BOARD, THINK, MOVE, READ_BOARD
    (_, column), *others = stack.calls[1:]
    assert others == []  # one drop; on_drop checked its target against board.column_target on the grid's pose
    assert state["board"] == list(stack.cells)
    assert state["board"].count(ROBOT) == 1
    assert state["last"]["who"] == "robot"
    assert state["last"]["column"] == column
    assert (state["turn"], state["moves"]) == ("human", 2)


def test_the_robot_plays_first(stack):
    stack.start(robot_first=True)
    assert stack.states[-1]["turn"] == "robot"
    assert stack.call(stack.human_done).success  # its token given, nothing dropped
    state = stack.wait_for("WAIT_HUMAN", moves=1)
    assert state["board"].count(ROBOT) == 1
    assert state["turn"] == "human"


@pytest.mark.parametrize("stack", [{"engine.max_depth": 42, "engine.budget_s": 0.5}], indirect=True)
def test_the_search_runs_outside_the_tick_and_stops_at_its_budget(stack):
    """THINK tick p99 at 130 ms or less [MES-66]: here, far from the 0.5 s a search in the tick would take."""
    stack.start()
    stack.human_plays(3)
    stack.wait_for("WAIT_HUMAN", moves=2)
    (line,) = [text for text in stack.logs if text.startswith("engine: ")]
    found = re.search(r"tick period max (\d+) ms over (\d+) ticks", line)
    assert found, line
    assert int(found[1]) < 200
    assert 3 <= int(found[2]) <= 8  # 0.5 s of search at 10 Hz


def test_the_level_is_a_parameter_changed_while_the_node_runs_and_the_next_search_uses_it(stack):
    """#244: engine.max_depth; the state shows it, and the robot's last move the depth its search reached."""
    state = stack.start()
    assert state["max_depth"] == 10
    stack.human_plays(3)
    assert stack.wait_for("WAIT_HUMAN", moves=2)["last"]["depth"] == 10
    (result,) = stack.game.set_parameters([Parameter("engine.max_depth", value=1)])
    assert result.successful, result.reason
    stack.human_plays(3)
    state = stack.wait_for("WAIT_HUMAN", moves=4)
    assert (state["max_depth"], state["last"]["depth"]) == (1, 1)
    assert set(state["last"]) >= {"who", "column", "planned", "score", "depth"}


@pytest.mark.parametrize(
    ("name", "value"),
    [("engine.max_depth", 0), ("engine.max_depth", 43), ("engine.seed", 1), ("engine.budget_s", 5.0)],
)
def test_a_depth_out_of_range_and_the_startup_parameters_are_refused_while_the_node_runs(stack, name, value):
    (result,) = stack.game.set_parameters([Parameter(name, value=value)])
    assert not result.successful


@pytest.mark.parametrize("stack", [{"engine.max_depth": 4, "engine.seed": 3}], indirect=True)
def test_a_seed_and_a_depth_replay_the_same_moves(stack):
    """The engine's seed and depth come from the parameters: the moves are game.best_move's with them."""
    stack.start()
    for column in (3, 3, 3):  # after them the mirrored columns 2 and 4 tie: the seed decides
        stack.human_plays(column)
        stack.wait_for("WAIT_HUMAN", turn="human", moves=stack.cells.count(HUMAN) * 2)
    cells = EMPTY_BOARD
    for column in (3, 3, 3):
        cells = with_tokens(cells, (column, HUMAN))
        me, mask = game.from_cells(cells, ROBOT)
        cells = with_tokens(cells, (game.best_move(me, mask, None, 4, 3).column, ROBOT))
    assert stack.cells == cells


def test_no_token_in_the_jaws_passes_through_and_the_turn_stays_the_robots(stack):
    stack.start()
    stack.replies["drop"] = (False, "no_token")
    stack.human_plays(3)
    state = stack.wait_for("WAIT_HUMAN:no_token")
    assert (state["turn"], state["lost"], state["moves"]) == ("robot", 1, 1)


def test_a_refusal_of_the_brain_ends_in_error_with_its_message(stack):
    stack.start()
    stack.replies["drop"] = (False, "unreachable")
    stack.human_plays(3)
    stack.wait_for("ERROR:brain_refused:unreachable")


def test_a_token_in_another_column_is_played_from_where_the_camera_sees_it(stack):
    stack.start()
    stack.lands = 6
    stack.human_plays(0)
    state = stack.wait_for("WAIT_HUMAN", moves=2)
    assert state["last"]["column"] == 6
    assert state["missed"] == (stack.calls[-1] != ("drop", 6))


@pytest.mark.parametrize("stack", [{"move_timeout_s": 0.5}], indirect=True)
def test_a_brain_that_never_replies_ends_in_error_and_is_stopped(stack):
    stack.hold = True
    stack.stop_cuts = False
    stack.up()
    assert stack.call(stack.new_game, SetBool.Request()).success
    stack.wait_for("ERROR:timeout:MOVE")
    stack.spin_until(lambda: ("stop",) in stack.calls)


def test_a_stop_cuts_the_move_and_stops_the_brain(stack):
    stack.start()
    stack.hold = True
    stack.human_plays(3)
    stack.spin_until(lambda: stack.calls[-1][0] == "drop")
    assert stack.call(stack.stop).success
    stack.wait_for("ERROR:stopped")
    stack.spin_until(lambda: ("stop",) in stack.calls)


def test_the_late_reply_of_a_cut_move_is_not_the_next_moves(stack):
    stack.start()
    stack.hold = True
    stack.stop_cuts = False
    stack.human_plays(3)
    stack.spin_until(lambda: stack.calls[-1][0] == "drop")
    stack.call(stack.stop)
    stack.wait_for("ERROR:stopped")
    stack.cells = EMPTY_BOARD
    assert stack.call(stack.new_game, SetBool.Request()).success
    stack.spin_until(lambda: len(stack.held) == 2)  # the cut drop, and the new game's handover
    stack.held[0].set_result((False, "unreachable"))  # the cut drop replies at last
    stack.spin_for(0.5)
    assert stack.label == "MOVE"
    stack.held[1].set_result((True, "done in 0.1 s"))
    stack.wait_for("WAIT_HUMAN", moves=0)


def test_the_services_are_refused_out_of_turn(stack):
    stack.spin_until(lambda: stack.states)
    reply = stack.call(stack.human_done)
    assert (reply.success, reply.message) == (False, "refused in IDLE")
    stack.hold = True
    stack.up()
    assert stack.call(stack.new_game, SetBool.Request()).success
    stack.wait_for("MOVE")
    reply = stack.call(stack.new_game, SetBool.Request())
    assert (reply.success, reply.message) == (False, "refused in MOVE")


@pytest.mark.parametrize("stack", [{"read_timeout_s": 0.5}], indirect=True)
def test_the_arm_is_never_called_while_the_human_plays_or_the_board_is_to_fix(stack):
    """Target P2-05: the brain's services are called in MOVE only."""
    stack.start()
    stack.spin_for(0.5)
    assert stack.call(stack.human_done).success  # no token came
    stack.wait_for("FIX_BOARD:not_seen:human")
    stack.spin_for(0.5)
    assert stack.calls == [("handover",)]


def test_a_parameter_the_node_does_not_know_stops_it():
    rclpy.init(args=["--ros-args", "-p", "read_timeout:=5.0"])
    try:
        with pytest.raises(ValueError, match="read_timeout"):
            GameNode()
    finally:
        rclpy.shutdown()


# The robot takes its tokens at the dispenser (#259): token_source, /connect4/stations, /brain/pick.
DISPENSER = {"token_source": "dispenser"}


@pytest.mark.parametrize("stack", [DISPENSER], indirect=True)
def test_with_a_dispenser_a_game_waits_for_its_pose_and_its_stations_too(stack):
    stack.waiting = None
    stack.pub_pose.publish(PoseStamped())
    stack.pub_grid.publish(pose(T_BASE_GRID))
    stack.spin_for(0.5)
    reply = stack.call(stack.new_game, SetBool.Request())
    assert not reply.success
    assert reply.message == "not ready: no /connect4/dispenser_pose, /connect4/stations"
    stack.waiting = (1, 1, 1)
    stack.up()
    assert stack.states[-1]["token_source"] == "dispenser"


def test_without_a_dispenser_a_game_does_not_wait_for_one(stack):
    stack.pub_pose.publish(PoseStamped())
    stack.pub_grid.publish(pose(T_BASE_GRID))
    stack.spin_until(lambda: stack.states and stack.states[-1]["ready"])
    assert stack.states[-1]["token_source"] == "hand"


@pytest.mark.parametrize("stack", [DISPENSER], indirect=True)
def test_with_a_dispenser_a_robots_turn_is_a_pick_at_the_station_as_it_is_seen_then_the_drop(stack):
    stack.waiting = (1, 1, 0)
    stack.start()
    stack.human_plays(3)
    state = stack.wait_for("WAIT_HUMAN", moves=2)
    (pick, rail), (drop, column) = stack.calls[1:]  # on_pick checked its target on the dispenser's pose
    assert (pick, rail, drop) == ("pick", 1, "drop")
    assert state["last"] == state["last"] | {"who": "robot", "column": column}


@pytest.mark.parametrize("stack", [DISPENSER], indirect=True)
def test_an_empty_dispenser_waits_for_the_person_and_the_game_goes_on_once_it_is_filled(stack):
    stack.waiting = (0, 0, 0)
    stack.start()
    stack.human_plays(3)
    state = stack.wait_for("WAIT_HUMAN:dispenser_empty")
    assert (state["turn"], state["lost"]) == ("robot", 0)
    assert stack.calls == [("handover",)]
    stack.waiting = (1, 0, 0)
    assert stack.call(stack.human_done).success
    stack.wait_for("WAIT_HUMAN", moves=2)
    assert [call[:2] for call in stack.calls[1:2]] == [("pick", 0)]


def test_a_token_source_the_node_does_not_know_stops_it():
    rclpy.init(args=["--ros-args", "-p", "token_source:=box"])
    try:
        with pytest.raises(ValueError, match="token_source"):
            GameNode()
    finally:
        rclpy.shutdown()


# Positions from perception (#260): the brain is asked, column by column, whether it would play where the
# camera sees the grid; the game is ready once it would.
CHECKING = "checking where the grid stands"


def set_down(T, along) -> np.ndarray:
    out = T.copy()
    out[:3, 3] += along * T[:3, 0]
    return out


def test_a_game_is_ready_once_the_brain_would_play_every_column_where_the_grid_is_seen(stack):
    stack.spin_until(lambda: stack.states)
    assert stack.states[-1]["not_ready"].startswith("no /end_effector_pose, /connect4/grid_pose")
    stack.up()
    assert stack.checked == [("drop", column) for column in range(COLUMNS)]
    assert stack.states[-1]["not_ready"] == ""
    assert CHECKING in [state["not_ready"] for state in stack.states]
    assert stack.calls == []  # nothing moved


@pytest.mark.parametrize("stack", [DISPENSER], indirect=True)
def test_with_a_dispenser_the_brain_is_asked_about_every_station_too(stack):
    stack.up()
    assert stack.checked[COLUMNS:] == [("pick", rail) for rail in range(len(STATIONS))]


def test_a_grid_the_brain_would_not_play_on_is_not_ready_and_the_state_and_the_refusal_say_why(stack):
    stack.checks[("drop", 6)] = (False, "unsafe_path:column")
    stack.pub_pose.publish(PoseStamped())
    stack.pub_grid.publish(pose(stack.grid))
    stack.spin_until(lambda: stack.states and stack.states[-1]["not_ready"].startswith("column"))
    assert stack.states[-1]["ready"] is False
    assert stack.states[-1]["not_ready"] == "column 7: unsafe_path:column"
    reply = stack.call(stack.new_game, SetBool.Request())
    assert (reply.success, reply.message) == (False, "not ready: column 7: unsafe_path:column")
    assert stack.calls == []


def test_a_grid_moved_between_two_games_is_checked_and_played_where_it_now_stands(stack):
    stack.up()
    seen = len(stack.states)
    stack.grid = set_down(T_BASE_GRID, -0.02)
    stack.pub_grid.publish(pose(stack.grid))
    stack.spin_until(lambda: len(stack.checked) == 2 * COLUMNS and stack.states[-1]["ready"])
    assert CHECKING in [state["not_ready"] for state in stack.states[seen:]]
    stack.start()
    stack.human_plays(3)
    stack.wait_for("WAIT_HUMAN", moves=2)  # the fake brain got the drop at its target on the grid's new pose
    assert stack.calls[-1][0] == "drop"


def test_the_brain_is_not_asked_while_it_moves_the_arm(stack):
    stack.start()
    stack.hold = True
    stack.human_plays(3)
    stack.spin_until(lambda: stack.calls[-1][0] == "drop")
    stack.grid = set_down(T_BASE_GRID, -0.02)
    stack.pub_grid.publish(pose(stack.grid))
    stack.spin_for(1.0)
    assert len(stack.checked) == COLUMNS
    stack.held[-1].set_result((True, "done in 14.0 s"))
    stack.spin_until(lambda: len(stack.checked) == 2 * COLUMNS)
