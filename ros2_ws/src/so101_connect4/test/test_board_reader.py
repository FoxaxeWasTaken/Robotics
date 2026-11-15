"""board_reader_node (#82, phase2/P2-02 §3) with a fake camera: the reference images of the scene (test/data)
fed as the driver's topics. The real node, in-process: its callbacks and its timer's step called by hand.
/connect4/board comes only after 5 agreeing reads on new images, stamped as the first; the grid's pose is the
layout's through our extrinsics; a hidden marker, a stale frame and one bad frame are handled."""

import json
from pathlib import Path

import cv2
import numpy as np
import pytest
import rclpy
from cv_bridge import CvBridge
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from so101_connect4 import board, layout
from so101_connect4.board_reader_node import DISPENSER_FRAME, GRID_FRAME, BoardReaderNode
from so101_perception.calibration import detect_markers

LAYOUT = layout.load()
DATA = Path(__file__).parent / "data"
REFERENCE = json.loads((DATA / "reference.json").read_text())
K = np.array(REFERENCE["K"])
BRIDGE = CvBridge()


def picture(name):
    return cv2.cvtColor(cv2.imread(str(DATA / f"{name}.png")), cv2.COLOR_BGR2RGB)


def without_markers(rgb, ids):
    """The image with those markers painted over, as a hand in front of them."""
    out = rgb.copy()
    found = detect_markers(rgb, LAYOUT["markers"]["dictionary"])
    for i in ids:
        cv2.fillPoly(out, [np.round(found[i]).astype(np.int32)], (120, 90, 70))
    return out


@pytest.fixture
def reader():
    rclpy.init()
    node = BoardReaderNode()
    node.boards, node.poses, node.debug, node.transforms = [], [], [], []
    node._pub_board.publish = node.boards.append
    node._pub_pose.publish = node.poses.append
    node._pub_debug.publish = node.debug.append
    node._tf.sendTransform = node.transforms.append
    node.dispenser_poses, node.stations = [], []
    node._pub_dispenser.publish = node.dispenser_poses.append
    node._pub_stations.publish = node.stations.append
    node.markers = []
    node._pub_markers.publish = node.markers.append
    info = CameraInfo(width=640, height=480)
    info.k = K.ravel().tolist()
    node._cb_camera_info(info)
    yield node
    node.destroy_node()
    rclpy.shutdown()


def show(node, rgb, stamp_ms):
    """One image from the camera, then one step of the node's timer."""
    msg = BRIDGE.cv2_to_imgmsg(rgb, encoding="rgb8")
    msg.header.stamp = Time(nanoseconds=stamp_ms * 1_000_000).to_msg()
    node._cb_image(msg)
    node._process()


def stamp_ms(msg):
    return Time.from_msg(msg.header.stamp).nanoseconds // 1_000_000


def test_the_board_comes_at_the_fifth_agreeing_image_stamped_as_the_first(reader):
    rgb = picture("midgame")
    for i in range(4):
        show(reader, rgb, 1000 + 100 * i)
    assert reader.boards == []
    show(reader, rgb, 1400)
    (msg,) = reader.boards
    assert tuple(msg.cells) == tuple(REFERENCE["boards"]["midgame"])
    assert msg.valid
    assert msg.reason == ""
    assert msg.header.frame_id == GRID_FRAME
    assert stamp_ms(msg) == 1000
    show(reader, rgb, 1500)
    assert len(reader.boards) == 2  # the streak goes on
    assert stamp_ms(reader.boards[1]) == 1000


def test_the_grid_pose_is_the_layouts_within_5_mm_in_base_link_with_its_tf(reader):
    show(reader, picture("empty"), 1000)
    (pose,) = reader.poses
    assert pose.header.frame_id == "base_link"
    assert stamp_ms(pose) == 1000
    position = pose.pose.position
    nominal = layout.grid_pose(LAYOUT)
    assert np.linalg.norm([position.x, position.y, position.z] - nominal[:3, 3]) < 0.005
    (tf,) = [tf for tf in reader.transforms if tf.child_frame_id == GRID_FRAME]  # the dispenser's too
    assert (tf.header.frame_id, tf.child_frame_id) == ("base_link", GRID_FRAME)
    assert tf.transform.translation.x == position.x
    # x of the grid frame along the columns, at the layout's azimuth
    q = pose.pose.orientation
    yaw = 2 * np.arctan2(q.z, q.w)
    assert yaw == pytest.approx(LAYOUT["placement"]["azimuth"], abs=np.radians(1.0))


def test_a_frame_seen_again_does_not_count(reader):
    rgb = picture("late")
    for _ in range(10):
        show(reader, rgb, 1000)  # the camera stalls: the same image, the same stamp
    assert reader.boards == []
    assert len(reader.debug) == 10


def test_one_bad_frame_in_a_streak_starts_it_again(reader):
    good, bad = picture("midgame"), picture("late")
    for i in range(4):
        show(reader, good, 1000 + 100 * i)
    show(reader, bad, 1400)  # a token in flight, a hand
    for i in range(4):
        show(reader, good, 1500 + 100 * i)
    assert reader.boards == []
    show(reader, good, 1900)
    (msg,) = reader.boards
    assert stamp_ms(msg) == 1500
    assert tuple(msg.cells) == tuple(REFERENCE["boards"]["midgame"])


def test_three_markers_are_enough_to_read_the_board(reader):
    rgb = without_markers(picture("midgame"), [LAYOUT["markers"]["ids"][3]])
    for i in range(5):
        show(reader, rgb, 1000 + 100 * i)
    (msg,) = reader.boards
    assert msg.valid
    assert tuple(msg.cells) == tuple(REFERENCE["boards"]["midgame"])
    assert len(reader.poses) == 5


def test_two_markers_give_an_invalid_board_with_its_reason_and_no_pose(reader):
    rgb = without_markers(picture("midgame"), LAYOUT["markers"]["ids"][:2])
    for i in range(5):
        show(reader, rgb, 1000 + 100 * i)
    (msg,) = reader.boards
    assert not msg.valid
    assert msg.reason == "markers:2"
    assert reader.poses == []
    assert len(reader.debug) == 5


def test_a_token_over_an_empty_cell_is_published_invalid_with_its_column(reader):
    rgb = picture("empty")
    T_cam_grid, *_ = board.grid_in_camera(detect_markers(rgb, LAYOUT["markers"]["dictionary"]), LAYOUT, K)
    centres, radius = board.cell_pixels(LAYOUT, T_cam_grid, K)
    u, v = centres[2 * 7 + 5]  # row 2 of column 5, nothing under it
    cv2.circle(rgb, (round(u), round(v)), round(radius * 1.3), (204, 13, 26), -1)
    for i in range(5):
        show(reader, rgb, 1000 + 100 * i)
    (msg,) = reader.boards
    assert not msg.valid
    assert msg.reason == "gravity:5"


def test_nothing_is_read_before_the_cameras_intrinsics_and_the_game_state_shows_on_the_debug_image(reader):
    reader._camera_info = None
    show(reader, picture("empty"), 1000)
    assert reader.debug == []
    assert reader.poses == []
    info = CameraInfo(width=640, height=480)
    info.k = K.ravel().tolist()
    reader._cb_camera_info(info)
    reader._cb_state(String(data=json.dumps({"state": "WAIT_HUMAN"})))
    reader._cb_state(String(data="not json"))  # ignored, the last good state is dropped
    show(reader, picture("empty"), 1100)
    (debug,) = reader.debug
    assert (debug.width, debug.height, debug.encoding) == (640, 480, "rgb8")
    assert stamp_ms(debug) == 1100


def test_the_debug_image_says_the_robots_reason_and_circles_the_winning_four(reader, monkeypatch):
    def shown(state):
        reader._cb_state(String(data=json.dumps(state)))
        show(reader, picture("midgame"), 1000 + 100 * len(reader.debug))
        return BRIDGE.imgmsg_to_cv2(reader.debug[-1], desired_encoding="rgb8")

    quiet = shown({"state": "WAIT_HUMAN", "last": None})
    last = {"who": "robot", "column": 3, "planned": 3, "score": 2, "depth": 10}
    told = shown({"state": "WAIT_HUMAN", "last": last})
    changed = np.flatnonzero((told != quiet).any(axis=(1, 2)))
    assert changed.min() > 22, "the reason is a second line, nothing else changes"
    assert changed.max() < 44, "the reason is a second line, nothing else changes"

    won = (1, 1, 1, 1, 2, 2, 2) + (0,) * 35
    monkeypatch.setattr(board, "read", lambda *args, **kwargs: won)
    circled = shown({"state": "GAME_OVER", "last": None})
    centres, radius = board.cell_pixels(LAYOUT, reader_pose(reader), K)
    u, v = centres[0]
    r, g, b = (int(c) for c in circled[round(v) - round(radius) - 3, round(u)])
    assert g > 200
    assert r < 80
    assert b < 80


def reader_pose(reader):
    """The grid in the camera, as the reader finds it in the reference image."""
    found = detect_markers(picture("midgame"), LAYOUT["markers"]["dictionary"])
    return board.grid_in_camera(found, LAYOUT, K)[0]


def test_the_readers_qos_is_the_contracts(reader):
    """01 §3.1: Reliable, the images' subscription too (a best-effort one gets 1 image of the 10 Hz, MES-67);
    the grid's pose latched, for a game node started after it."""

    def qos(endpoints):
        (endpoint,) = endpoints
        return endpoint.qos_profile.reliability, endpoint.qos_profile.durability

    reliable, volatile, latched = (
        ReliabilityPolicy.RELIABLE,
        DurabilityPolicy.VOLATILE,
        DurabilityPolicy.TRANSIENT_LOCAL,
    )
    assert qos(reader.get_subscriptions_info_by_topic("/external_cam/image_raw")) == (reliable, volatile)
    assert qos(reader.get_publishers_info_by_topic("/connect4/board")) == (reliable, volatile)
    assert qos(reader.get_publishers_info_by_topic("/connect4/grid_pose")) == (reliable, latched)
    assert qos(reader.get_publishers_info_by_topic("/connect4/debug_image")) == (reliable, volatile)
    assert qos(reader.get_publishers_info_by_topic("/connect4/markers")) == (
        reliable,
        latched,
    )  # RViz comes later


# The token dispenser (#258): its pose and its stations, read in the same images as the grid.

DISPENSER_IDS = LAYOUT["dispenser"]["markers"]["ids"]


def test_the_dispensers_pose_is_the_layouts_within_2_mm_in_base_link_with_its_tf(reader):
    show(reader, picture("empty"), 1000)
    (pose,) = reader.dispenser_poses
    assert pose.header.frame_id == "base_link"
    assert stamp_ms(pose) == 1000
    p, o = pose.pose.position, pose.pose.orientation
    T = board.transform((p.x, p.y, p.z), (o.x, o.y, o.z, o.w))
    for rail in range(LAYOUT["dispenser"]["rails"]):
        there = [*layout.station(LAYOUT, rail), 1.0]
        assert np.linalg.norm((T @ there - layout.dispenser_pose(LAYOUT) @ there)[:3]) < 0.002, rail
    frames = {tf.child_frame_id: tf.header.frame_id for tf in reader.transforms}
    assert frames == {GRID_FRAME: "base_link", "connect4_dispenser": "base_link"}


def test_the_stations_come_at_the_fifth_agreeing_image(reader):
    rgb = picture("midgame")
    for i in range(4):
        show(reader, rgb, 1000 + 100 * i)
    assert reader.stations == []
    show(reader, rgb, 1400)
    (msg,) = reader.stations
    assert list(msg.data) == [int(there) for there in REFERENCE["stations"]["midgame"]]
    assert list(msg.data) == [0, 1, 1]  # the lowest rail gave its tokens first


def test_with_its_markers_hidden_the_dispenser_is_not_placed_and_the_grid_still_is(reader):
    rgb = without_markers(picture("midgame"), DISPENSER_IDS[:2])
    for i in range(5):
        show(reader, rgb, 1000 + 100 * i)
    assert reader.dispenser_poses == []
    assert reader.stations == []
    assert len(reader.poses) == 5
    assert len(reader.boards) == 1


def test_the_debug_image_rings_each_station_green_when_a_token_waits_there(reader):
    show(reader, picture("midgame"), 1000)
    seen = BRIDGE.imgmsg_to_cv2(reader.debug[-1], desired_encoding="rgb8").astype(int)
    found = detect_markers(picture("midgame"), LAYOUT["markers"]["dictionary"])
    T_cam_dispenser = board.dispenser_in_camera(found, LAYOUT, K)[0]
    pixels, radius = board.station_pixels(LAYOUT, T_cam_dispenser, K)
    for (u, v), there in zip(pixels, REFERENCE["stations"]["midgame"], strict=True):
        r, g, b = seen[round(v) - round(radius), round(u)]  # on the ring
        assert (g > 200 and r < 80 and b < 80) == there, (u, v, there)


# The view's shapes (#280): the grid, its tokens and the dispenser for RViz, in the frames the reader measures


def test_the_shapes_come_with_the_first_stable_board_and_again_only_when_it_changes(reader):
    for i in range(4):
        show(reader, picture("midgame"), 1000 + 100 * i)
    assert reader.markers == []
    for i in range(4, 8):
        show(reader, picture("midgame"), 1000 + 100 * i)
    (msg,) = reader.markers  # the three images after the fifth changed nothing
    added = {(marker.ns, marker.id) for marker in msg.markers if marker.action == Marker.ADD}
    cells = reader.boards[-1].cells
    assert {i for ns, i in added if ns == "tokens"} == {i for i, cell in enumerate(cells) if cell}
    assert {i for ns, i in added if ns == "stations"} == {1, 2}  # as /connect4/stations
    frames = {marker.ns: marker.header.frame_id for marker in msg.markers}
    assert frames == {
        "grid": GRID_FRAME,
        "tokens": GRID_FRAME,
        "dispenser": DISPENSER_FRAME,
        "stations": DISPENSER_FRAME,
    }
    # no stamp and locked to its frame: RViz draws each shape at the frame's latest measured pose
    assert all(marker.frame_locked and marker.header.stamp.sec == 0 for marker in msg.markers)
    token = next(
        marker for marker in msg.markers if (marker.ns, marker.id) in added and marker.ns == "tokens"
    )
    assert token.type == Marker.CYLINDER and np.isclose(token.scale.z, LAYOUT["token"]["thickness"])

    for i in range(5):  # a new game: the tokens that are gone are deleted from the view
        show(reader, picture("empty"), 2000 + 100 * i)
    assert len(reader.markers) == 2
    tokens = [marker.action for marker in reader.markers[-1].markers if marker.ns == "tokens"]
    assert tokens == [Marker.DELETE] * 42


def test_a_grid_the_arm_hides_keeps_its_tokens_in_the_view(reader):
    for i in range(5):
        show(reader, picture("midgame"), 1000 + 100 * i)
    hidden = without_markers(picture("midgame"), LAYOUT["markers"]["ids"])
    for i in range(5):
        show(reader, hidden, 2000 + 100 * i)
    assert reader.boards[-1].valid is False  # published as not read, with no cell
    assert len(reader.markers) == 1  # the view keeps the last board that was read
